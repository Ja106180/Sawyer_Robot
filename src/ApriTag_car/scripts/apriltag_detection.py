#!/usr/bin/env python3
"""
apriltag_detection.py - AprilTag 检测节点

功能：
1. 使用 apriltag 库检测图像中的 AprilTag 36h11
2. 使用相机内参计算 tag 的位姿（位置 + 朝向）
3. 发布位姿到 /apriltag_car/tag_pose 话题

Tag 尺寸：5cm × 5cm = 0.05m
"""

import cv2
import numpy as np
import rospy
import yaml
import os
import rospkg
import threading
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from cv_bridge import CvBridge
from std_msgs.msg import Header


try:
    import apriltag
except ImportError:
    rospy.logerr("apriltag 库未安装！请运行: pip install apriltag")
    rospy.signal_shutdown("缺少依赖库")


class AprilTagDetector:
    def __init__(self):
        rospy.init_node("apriltag_detection", anonymous=True)
        
        # Tag 尺寸（米）
        self.tag_size = rospy.get_param("~tag_size", 0.05)  # 5cm
        
        # 加载相机内参
        calibration_path = rospy.get_param("~calibration_path", "")
        if not calibration_path:
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('ApriTag_car')
            calibration_path = os.path.join(package_path, 'config', 'camera_intrinsics.yaml')
        
        self.load_camera_intrinsics(calibration_path)
        
        # 初始化 AprilTag 检测器
        # 尝试支持所有 tag families（包括 tag36h11, tag25h9, tag16h5 等）
        try:
            # 默认配置支持所有 families
            self.detector = apriltag.Detector()
            rospy.loginfo("AprilTag 检测器已初始化")
            rospy.loginfo("支持的 tag families: %s", [f.decode() if isinstance(f, bytes) else f for f in self.detector.families])
        except Exception as e:
            rospy.logwarn("初始化检测器失败，尝试其他配置: %s", e)
            # 如果失败，尝试指定 families
            try:
                self.detector = apriltag.Detector(families='tag36h11')
                rospy.loginfo("使用 tag36h11 family")
            except:
                self.detector = apriltag.Detector()
                rospy.logwarn("使用默认检测器配置")
        
        # CvBridge
        self.bridge = CvBridge()
        
        # 发布者
        self.pose_pub = rospy.Publisher("/apriltag_car/tag_pose", PoseStamped, queue_size=1)
        self.image_pub = rospy.Publisher("/apriltag_car/detection_image", Image, queue_size=1)
        
        # 订阅者
        image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        
        # 图像显示窗口
        self.window_name = "AprilTag Detection"
        self.display_window = False
        try:
            test_window = "test_window_" + str(rospy.get_time())
            cv2.namedWindow(test_window, cv2.WINDOW_NORMAL)
            cv2.destroyWindow(test_window)
            self.display_window = True
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            rospy.loginfo("GUI环境可用，创建检测窗口: %s", self.window_name)
        except Exception as e:
            self.display_window = False
            rospy.logwarn("GUI环境不可用，运行在服务器模式: %s", str(e))
        
        # 图像缓存（线程安全）
        self.latest_display_image = None
        self.image_lock = threading.Lock()
        self.image_received = False
        
        # 订阅图像
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1)
        
        rospy.loginfo("AprilTag 检测节点已启动")
        rospy.loginfo("Tag 尺寸: %.3f m", self.tag_size)
        rospy.loginfo("订阅图像话题: %s", image_topic)
        rospy.loginfo("等待图像数据...")
    
    def load_camera_intrinsics(self, path):
        """加载相机内参"""
        if not os.path.exists(path):
            rospy.logerr("相机内参文件不存在: %s", path)
            rospy.signal_shutdown("缺少相机内参文件")
            return
        
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        
        # 读取相机内参（YAML 格式：camera_matrix 是字典，camera_matrix_full 是矩阵）
        if 'camera_matrix_full' in data:
            # 使用完整的矩阵
            self.camera_matrix = np.array(data['camera_matrix_full'], dtype=np.float32)
        else:
            # 从字典构建矩阵
            cam_data = data['camera_matrix']
            self.camera_matrix = np.array([
                [cam_data['fx'], 0, cam_data['cx']],
                [0, cam_data['fy'], cam_data['cy']],
                [0, 0, 1]
            ], dtype=np.float32)
        
        # 读取畸变系数
        dist_data = data['distortion_coefficients']
        if isinstance(dist_data[0], list):
            # 如果是嵌套列表，取第一个
            self.dist_coeffs = np.array(dist_data[0], dtype=np.float32)
        else:
            self.dist_coeffs = np.array(dist_data, dtype=np.float32)
        
        self.fx = self.camera_matrix[0, 0]
        self.fy = self.camera_matrix[1, 1]
        self.cx = self.camera_matrix[0, 2]
        self.cy = self.camera_matrix[1, 2]
        
        rospy.loginfo("相机内参已加载:")
        rospy.loginfo("  fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", 
                     self.fx, self.fy, self.cx, self.cy)
    
    def estimate_tag_pose(self, corners, tag_size):
        """
        估计 tag 的位姿
        
        corners: 4x2 数组，tag 的四个角点（图像坐标）
        tag_size: tag 的物理尺寸（米）
        
        返回: (R, tvec) 旋转矩阵和平移向量
        """
        # tag 的 3D 坐标（tag 坐标系：中心为原点，Z 轴垂直于 tag 平面）
        objp = np.array([
            [-tag_size/2, -tag_size/2, 0],
            [tag_size/2, -tag_size/2, 0],
            [tag_size/2, tag_size/2, 0],
            [-tag_size/2, tag_size/2, 0]
        ], dtype=np.float32)
        
        # 使用 solvePnP 估计位姿
        success, rvec, tvec = cv2.solvePnP(
            objp,
            corners.astype(np.float32),
            self.camera_matrix,
            self.dist_coeffs
        )
        
        if not success:
            return None, None
        
        # 转换为旋转矩阵
        R, _ = cv2.Rodrigues(rvec)
        
        return R, tvec
    
    def image_callback(self, msg):
        """图像回调函数 - 只处理图像，不显示"""
        try:
            # 转换为 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            if cv_image is None or cv_image.size == 0:
                return
            
            # 首次收到图像时记录日志
            if not self.image_received:
                rospy.loginfo("首次收到图像，尺寸: %dx%d", cv_image.shape[1], cv_image.shape[0])
                self.image_received = True
            
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # 图像预处理以提高检测率（如果检测不到可以取消注释）
            # 1. 直方图均衡化（提高对比度）
            gray_enhanced = cv2.equalizeHist(gray)
            # 2. 高斯模糊去噪（可选）
            # gray_enhanced = cv2.GaussianBlur(gray_enhanced, (3, 3), 0)
            
            # 检测 AprilTag（尝试原始图像和增强图像）
            detections = self.detector.detect(gray)
            if len(detections) == 0:
                # 如果原始图像检测不到，尝试增强后的图像
                detections = self.detector.detect(gray_enhanced)
            
            # 准备显示图像（必须copy，因为会在主线程中使用）
            display_image = cv_image.copy()
            
            if len(detections) == 0:
                # 没有检测到 tag - 添加调试信息
                cv2.putText(display_image, "No Tag Detected", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                # 定期输出调试信息（避免日志过多）
                if not hasattr(self, '_last_debug_time'):
                    self._last_debug_time = rospy.Time.now()
                if (rospy.Time.now() - self._last_debug_time).to_sec() > 2.0:
                    rospy.loginfo_throttle(2, "未检测到 tag，请检查：1) tag 是否在视野内 2) 光照是否充足 3) tag 是否清晰")
                    self._last_debug_time = rospy.Time.now()
            else:
                # 选择第一个检测到的 tag
                detection = detections[0]
                corners = detection.corners
                
                # 估计位姿
                R, tvec = self.estimate_tag_pose(corners, self.tag_size)
                
                if R is not None and tvec is not None:
                    # 创建位姿消息
                    pose_msg = PoseStamped()
                    pose_msg.header = Header()
                    pose_msg.header.stamp = rospy.Time.now()
                    pose_msg.header.frame_id = "camera_frame"
                    
                    # 位置（相机坐标系）
                    pose_msg.pose.position.x = float(tvec[0])
                    pose_msg.pose.position.y = float(tvec[1])
                    pose_msg.pose.position.z = float(tvec[2])
                    
                    # 朝向（旋转矩阵转四元数）
                    quat = self.rotation_matrix_to_quaternion(R)
                    pose_msg.pose.orientation.x = quat[0]
                    pose_msg.pose.orientation.y = quat[1]
                    pose_msg.pose.orientation.z = quat[2]
                    pose_msg.pose.orientation.w = quat[3]
                    
                    # 发布位姿
                    self.pose_pub.publish(pose_msg)
                    
                    # 绘制检测结果
                    self.draw_detection(display_image, detection, corners, tvec)
                else:
                    # 位姿估计失败，只绘制边界框
                    self.draw_detection(display_image, detection, corners, None)
            
            # 更新显示图像缓存（线程安全）
            with self.image_lock:
                self.latest_display_image = display_image.copy()
            
            # 发布可视化图像
            vis_image = self.bridge.cv2_to_imgmsg(display_image, "bgr8")
            vis_image.header = msg.header
            self.image_pub.publish(vis_image)
            
        except Exception as e:
            rospy.logerr("图像处理错误: %s", e)
    
    def rotation_matrix_to_quaternion(self, R):
        """旋转矩阵转四元数"""
        rvec, _ = cv2.Rodrigues(R)
        angle = np.linalg.norm(rvec)
        if angle < 1e-6:
            return [0, 0, 0, 1]
        axis = rvec / angle
        qw = np.cos(angle / 2)
        qx = axis[0] * np.sin(angle / 2)
        qy = axis[1] * np.sin(angle / 2)
        qz = axis[2] * np.sin(angle / 2)
        return [qx, qy, qz, qw]
    
    def draw_detection(self, image, detection, corners, tvec):
        """在图像上绘制检测结果"""
        # 绘制 tag 边界
        corners_int = np.array(corners, dtype=np.int32)
        cv2.polylines(image, [corners_int], True, (0, 255, 0), 2)
        
        # 绘制 tag ID
        center = np.mean(corners, axis=0).astype(int)
        cv2.putText(image, f"ID: {detection.tag_id}", 
                   (center[0] - 20, center[1] - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # 绘制距离信息（如果位姿估计成功）
        if tvec is not None:
            distance = np.linalg.norm(tvec)
            cv2.putText(image, f"Dist: {distance:.2f}m",
                       (center[0] - 30, center[1] + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        else:
            cv2.putText(image, "Pose est. failed",
                       (center[0] - 40, center[1] + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        # 绘制中心点
        cv2.circle(image, tuple(center), 5, (0, 0, 255), -1)
    
    def run(self):
        """主循环：在主线程中显示图像"""
        rate = rospy.Rate(30)  # 30Hz 显示频率
        
        while not rospy.is_shutdown():
            if self.display_window:
                # 从缓存中获取最新图像（线程安全）
                with self.image_lock:
                    display_image = self.latest_display_image
                
                if display_image is not None:
                    # 在主线程中显示图像
                    cv2.imshow(self.window_name, display_image)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        rospy.loginfo("收到退出信号")
                        rospy.signal_shutdown("用户退出")
                        break
                else:
                    # 没有图像时显示等待信息
                    blank = np.zeros((480, 640, 3), dtype=np.uint8)
                    text = "Waiting for image..." if not self.image_received else "No image available"
                    cv2.putText(blank, text, (20, 240),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                    cv2.imshow(self.window_name, blank)
                    cv2.waitKey(1)
            
            rate.sleep()
        
        # 清理窗口
        if self.display_window:
            cv2.destroyAllWindows()


def main():
    try:
        detector = AprilTagDetector()
        # 在主线程中运行显示循环
        detector.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("节点异常: %s", e)
        import traceback
        rospy.logerr(traceback.format_exc())
    finally:
        try:
            cv2.destroyAllWindows()
        except:
            pass


if __name__ == "__main__":
    main()
