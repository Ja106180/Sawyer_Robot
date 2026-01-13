#!/usr/bin/env python3
"""
face_follow.py

人脸跟踪节点：
1. 使用USB摄像头读取图像
2. MediaPipe人脸检测
3. 虚拟弹簧控制算法让人脸保持在屏幕中心
4. 控制Sawyer机械臂的J4（左右）和J5（上下）关节
"""

import cv2
import rospy
import numpy as np
import mediapipe as mp
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Header
from intera_core_msgs.msg import JointCommand
import intera_interface
import time
import threading


class FaceFollowNode:
    def __init__(self):
        rospy.init_node("face_follow_node", anonymous=False)
        
        # 参数配置
        self.image_topic = rospy.get_param("~image_topic", "/camera/image_raw")  # 图像话题
        self.frame_width = rospy.get_param("~frame_width", 640)
        self.frame_height = rospy.get_param("~frame_height", 480)
        self.fps = rospy.get_param("~fps", 30.0)
        self.control_rate = rospy.get_param("~control_rate", 20.0)  # 控制频率（Hz）
        
        # 人脸检测参数
        self.face_confidence_threshold = rospy.get_param("~face_confidence_threshold", 0.75)
        self.min_face_area_ratio = rospy.get_param("~min_face_area_ratio", 0.01)  # 最小人脸面积比例
        self.max_face_area_ratio = rospy.get_param("~max_face_area_ratio", 0.5)  # 最大人脸面积比例
        
        # 虚拟弹簧控制参数（PD控制器）
        # Kp: 比例系数，控制响应速度
        self.kp_x = rospy.get_param("~kp_x", 0.005)  # 水平方向比例系数（rad/pixel）
        self.kp_y = rospy.get_param("~kp_y", 0.005)  # 垂直方向比例系数（rad/pixel）
        # Kd: 微分系数，控制阻尼，减少震荡
        self.kd_x = rospy.get_param("~kd_x", 0.0005)  # 水平方向微分系数（rad·s/pixel）
        self.kd_y = rospy.get_param("~kd_y", 0.0005)  # 垂直方向微分系数（rad·s/pixel）
        # 死区阈值：误差小于此值时不运动（像素）
        self.deadzone_x = rospy.get_param("~deadzone_x", 10.0)  # 水平死区（像素）
        self.deadzone_y = rospy.get_param("~deadzone_y", 10.0)  # 垂直死区（像素）
        # 最大速度限制（rad/s）
        self.max_velocity_x = rospy.get_param("~max_velocity_x", 0.3)  # 水平最大速度
        self.max_velocity_y = rospy.get_param("~max_velocity_y", 0.3)  # 垂直最大速度
        
        # 平滑滤波参数（低通滤波）——用于关节位置
        self.filter_alpha = rospy.get_param("~filter_alpha", 0.7)  # 滤波系数（0-1，越大越平滑）
        # 误差平滑参数：对图像误差做一层滤波，特别是垂直方向，抑制抖动
        self.error_filter_alpha_x = rospy.get_param("~error_filter_alpha_x", 0.5)   # 水平误差滤波
        self.error_filter_alpha_y = rospy.get_param("~error_filter_alpha_y", 0.9)   # 垂直误差滤波（更平滑）

        # 垂直快速区阈值：像素，大于此值认为偏差较大，直接用最大速度
        self.fast_zone_y = rospy.get_param("~fast_zone_y", 60.0)
        
        # 超时保护：如果长时间检测不到人脸，停止运动
        self.face_lost_timeout = rospy.get_param("~face_lost_timeout", 2.0)  # 超时时间（秒）
        
        # 关节限位（弧度）
        self.j4_min = rospy.get_param("~j4_min", -2.0)
        self.j4_max = rospy.get_param("~j4_max", 2.0)
        self.j5_min = rospy.get_param("~j5_min", -2.0)
        self.j5_max = rospy.get_param("~j5_max", 1.7)
        
        # CvBridge
        self.bridge = CvBridge()
        
        # 订阅图像话题
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)
        
        # MediaPipe初始化
        self.mp_face_detection = mp.solutions.face_detection
        self.mp_drawing = mp.solutions.drawing_utils
        
        self.face_detection = self.mp_face_detection.FaceDetection(
            model_selection=0,  # 0=short-range, 1=full-range
            min_detection_confidence=self.face_confidence_threshold
        )
        
        # 机械臂接口：获取当前关节角度
        self.joint_names = ["right_j4", "right_j5"]
        try:
            self.limb = intera_interface.Limb("right")
            current_angles = self.limb.joint_angles()
            # 记录当前J4和J5的角度作为初始位置
            self.current_j4 = current_angles.get("right_j4", 0.0)
            self.current_j5 = current_angles.get("right_j5", 0.0)
            rospy.loginfo("Initial joint angles - J4: %.3f rad, J5: %.3f rad", 
                         self.current_j4, self.current_j5)
        except Exception as exc:
            rospy.logerr("Failed to initialize limb interface: %s", exc)
            raise
        
        # 发布者
        self.joint_cmd_pub = rospy.Publisher("/robot/limb/right/joint_command", JointCommand, queue_size=10)
        
        # 发布图像（可选，用于调试）
        self.image_pub = rospy.Publisher("/face_follow/image", Image, queue_size=1)
        
        # 状态变量
        self.last_error_x = 0.0  # 上一次的水平误差（像素，已滤波）
        self.last_error_y = 0.0  # 上一次的垂直误差（像素，已滤波）
        self.error_x_filtered = 0.0  # 当前水平误差（滤波后）
        self.error_y_filtered = 0.0  # 当前垂直误差（滤波后）
        self.last_time = time.time()  # 上一次控制时间
        self.filtered_j4 = self.current_j4  # 滤波后的J4角度
        self.filtered_j5 = self.current_j5  # 滤波后的J5角度
        self.last_face_time = None  # 上次检测到人脸的时间
        self.face_detected = False  # 是否检测到人脸
        
        # 图像处理状态变量
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.image_received = False
        
        rospy.loginfo("Face follow node started")
        rospy.loginfo("Subscribing to image topic: %s", self.image_topic)
        rospy.loginfo("Control rate: %.1f Hz", self.control_rate)
        rospy.loginfo("Control parameters - Kp_x: %.4f, Kp_y: %.4f, Kd_x: %.4f, Kd_y: %.4f",
                     self.kp_x, self.kp_y, self.kd_x, self.kd_y)
        
        # 等待图像话题可用（最多等待10秒）
        try:
            rospy.loginfo("Waiting for image topic: %s", self.image_topic)
            rospy.wait_for_message(self.image_topic, Image, timeout=10.0)
            rospy.loginfo("Image topic is available, face tracking ready")
        except rospy.ROSException:
            rospy.logwarn("Image topic not available yet, but continuing anyway...")
            rospy.logwarn("Face tracking will start once images are received")
        
        # 设置循环频率
        self.rate = rospy.Rate(self.control_rate)
    
    def detect_face(self, frame):
        """
        检测人脸并返回人脸中心坐标
        
        Args:
            frame: OpenCV图像（BGR格式）
            
        Returns:
            (face_center_x, face_center_y, confidence) 或 (None, None, 0.0)
        """
        # 转换为RGB（MediaPipe需要）
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb_frame.flags.writeable = False
        
        # 人脸检测
        face_results = self.face_detection.process(rgb_frame)
        
        if not face_results.detections:
            return None, None, 0.0
        
        # 找到置信度最高的人脸
        best_detection = None
        best_confidence = 0.0
        
        for detection in face_results.detections:
            confidence = detection.score[0]
            if confidence > best_confidence:
                best_confidence = confidence
                best_detection = detection
        
        # 只处理置信度足够高的人脸
        if best_detection and best_confidence >= self.face_confidence_threshold:
            bbox = best_detection.location_data.relative_bounding_box
            h, w = frame.shape[:2]
            
            x = int(bbox.xmin * w)
            y = int(bbox.ymin * h)
            width = int(bbox.width * w)
            height = int(bbox.height * h)
            
            # 面积过滤：人脸面积应该合理
            face_area = width * height
            min_area = (w * h) * self.min_face_area_ratio
            max_area = (w * h) * self.max_face_area_ratio
            
            if min_area <= face_area <= max_area:
                # 计算人脸中心
                face_center_x = x + width / 2.0
                face_center_y = y + height / 2.0
                
                return face_center_x, face_center_y, best_confidence
        
        return None, None, 0.0
    
    def image_callback(self, msg):
        """
        图像回调函数（持续接收图像，驱动人脸跟踪）
        
        Args:
            msg: ROS Image 消息
        """
        try:
            # 转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            if cv_image is None or cv_image.size == 0:
                rospy.logwarn("Received empty image")
                return
            
            # 首次收到图像时记录日志
            if not self.image_received:
                rospy.loginfo("✓ First image received, size: %dx%d", cv_image.shape[1], cv_image.shape[0])
                rospy.loginfo("✓ Face tracking is now ACTIVE")
                self.image_received = True
            
            # 保存原始图像（线程安全）
            with self.frame_lock:
                self.latest_frame = cv_image
            
        except Exception as e:
            rospy.logerr("Error in image callback: %s", e)
    
    def virtual_spring_control(self, error_x, error_y, dt):
        """
        虚拟弹簧控制算法（PD控制器），在误差上做一层滤波，降低抖动
        
        Args:
            error_x: 水平方向误差（像素，正数表示人脸在右侧，原始值）
            error_y: 垂直方向误差（像素，正数表示人脸在下侧，原始值）
            dt: 时间间隔（秒）
            
        Returns:
            (j4_velocity, j5_velocity) 关节速度（rad/s）
        """
        # 对误差做一层低通滤波，抑制图像抖动引起的关节抖动
        self.error_x_filtered = (
            self.error_filter_alpha_x * self.error_x_filtered
            + (1.0 - self.error_filter_alpha_x) * error_x
        )
        self.error_y_filtered = (
            self.error_filter_alpha_y * self.error_y_filtered
            + (1.0 - self.error_filter_alpha_y) * error_y
        )

        # 使用滤波后的误差进行PD控制
        ex = self.error_x_filtered
        ey = self.error_y_filtered

        # 计算误差变化率（基于滤波后的误差）
        error_dot_x = (ex - self.last_error_x) / dt if dt > 0 else 0.0
        error_dot_y = (ey - self.last_error_y) / dt if dt > 0 else 0.0
        
        # --- 水平方向：常规PD控制（左右工作很好，作为参考） ---
        # J4 方向已反转：人脸在右侧（ex > 0）时，给负速度；反之亦然
        velocity_j4 = -(self.kp_x * ex + self.kd_x * error_dot_x)

        # --- 垂直方向：完全参照左右的方式，使用简单PD控制 ---
        # 和左右一样的简单PD控制，不搞复杂的分段和过冲检测
        velocity_j5 = self.kp_y * ey + self.kd_y * error_dot_y

        # 死区处理：误差太小时不运动（基于滤波后的误差）
        if abs(ex) < self.deadzone_x:
            velocity_j4 = 0.0
        if abs(ey) < self.deadzone_y:
            velocity_j5 = 0.0
        
        # 速度限制
        velocity_j4 = np.clip(velocity_j4, -self.max_velocity_x, self.max_velocity_x)
        velocity_j5 = np.clip(velocity_j5, -self.max_velocity_y, self.max_velocity_y)
        
        # 更新上一次的误差（滤波后的）
        self.last_error_x = ex
        self.last_error_y = ey
        
        return velocity_j4, velocity_j5
    
    def update_joint_positions(self, velocity_j4, velocity_j5, dt):
        """
        根据速度更新关节位置
        
        Args:
            velocity_j4: J4关节速度（rad/s）
            velocity_j5: J5关节速度（rad/s）
            dt: 时间间隔（秒）
        """
        # 根据速度更新位置
        new_j4 = self.filtered_j4 + velocity_j4 * dt
        new_j5 = self.filtered_j5 + velocity_j5 * dt
        
        # 关节限位检查
        new_j4 = np.clip(new_j4, self.j4_min, self.j4_max)
        new_j5 = np.clip(new_j5, self.j5_min, self.j5_max)
        
        # 低通滤波平滑
        self.filtered_j4 = self.filter_alpha * self.filtered_j4 + (1 - self.filter_alpha) * new_j4
        self.filtered_j5 = self.filter_alpha * self.filtered_j5 + (1 - self.filter_alpha) * new_j5
        
        # 更新当前角度
        self.current_j4 = self.filtered_j4
        self.current_j5 = self.filtered_j5
    
    def send_joint_command(self):
        """
        发送关节位置命令到机械臂（持续发送，保持位置或跟踪人脸）
        """
        try:
            cmd = JointCommand()
            cmd.header = Header()
            cmd.header.stamp = rospy.Time.now()
            
            cmd.names = self.joint_names
            cmd.position = [self.current_j4, self.current_j5]
            cmd.mode = JointCommand.POSITION_MODE
            
            self.joint_cmd_pub.publish(cmd)
            
            # 首次发送时记录日志
            if not hasattr(self, '_first_command_sent'):
                rospy.loginfo("First joint command sent - J4: %.3f rad, J5: %.3f rad", 
                             self.current_j4, self.current_j5)
                self._first_command_sent = True
        except Exception as e:
            rospy.logerr("Error sending joint command: %s", e)
    
    def draw_debug_info(self, frame, face_center_x, face_center_y, error_x, error_y):
        """
        在图像上绘制调试信息
        
        Args:
            frame: OpenCV图像
            face_center_x: 人脸中心X坐标
            face_center_y: 人脸中心Y坐标
            error_x: 水平误差
            error_y: 垂直误差
        """
        h, w = frame.shape[:2]
        screen_center_x = w / 2.0
        screen_center_y = h / 2.0
        
        # 绘制屏幕中心十字线
        cv2.line(frame, (int(screen_center_x - 20), int(screen_center_y)),
                 (int(screen_center_x + 20), int(screen_center_y)), (0, 255, 0), 2)
        cv2.line(frame, (int(screen_center_x), int(screen_center_y - 20)),
                 (int(screen_center_x), int(screen_center_y + 20)), (0, 255, 0), 2)
        
        # 如果检测到人脸，绘制人脸中心点和误差线
        if face_center_x is not None and face_center_y is not None:
            # 绘制人脸中心点
            cv2.circle(frame, (int(face_center_x), int(face_center_y)), 5, (0, 0, 255), -1)
            
            # 绘制误差线（从屏幕中心到人脸中心）
            cv2.line(frame, (int(screen_center_x), int(screen_center_y)),
                     (int(face_center_x), int(face_center_y)), (255, 0, 0), 2)
            
            # 显示误差信息
            error_text = f"Error X: {error_x:.1f}px, Y: {error_y:.1f}px"
            cv2.putText(frame, error_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                       0.6, (255, 255, 255), 2)
            
            # 显示关节角度
            joint_text = f"J4: {self.current_j4:.3f} rad, J5: {self.current_j5:.3f} rad"
            cv2.putText(frame, joint_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                       0.6, (255, 255, 255), 2)
        else:
            # 未检测到人脸
            cv2.putText(frame, "No face detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                       0.6, (0, 0, 255), 2)
    
    def run(self):
        """主循环：控制循环和显示（一直运行，持续跟踪人脸）"""
        rospy.loginfo("Face follow node running...")
        rospy.loginfo("Face tracking is ACTIVE and will continue running")
        rospy.loginfo("Waiting for images from topic: %s", self.image_topic)
        
        # 初始化时发送一次关节命令，确保机械臂知道当前位置
        rospy.loginfo("Initializing: sending initial joint command to maintain current position")
        self.send_joint_command()
        
        window_name = "Face Follow"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        
        while not rospy.is_shutdown():
            try:
                # 从回调函数获取最新图像
                with self.frame_lock:
                    if self.latest_frame is not None:
                        frame = self.latest_frame.copy()
                    else:
                        frame = None
                
                current_time = time.time()
                dt = current_time - self.last_time
                # 限制时间间隔，避免首次运行或长时间暂停后dt过大
                dt = min(dt, 0.1)  # 最大0.1秒
                if dt < 0.001:  # 如果dt太小，使用控制频率的倒数
                    dt = 1.0 / self.control_rate
                self.last_time = current_time
                
                if frame is None:
                    # 没有图像，等待并继续发送当前关节命令（保持位置）
                    if rospy.get_time() % 2.0 < 0.05:  # 每2秒输出一次
                        rospy.logdebug("No image received yet, keeping current joint positions")
                    # 即使没有图像，也发送当前关节命令，保持位置
                    self.send_joint_command()
                    self.rate.sleep()
                    continue
                
                # 检测人脸
                face_center_x, face_center_y, confidence = self.detect_face(frame)
                
                if face_center_x is not None and face_center_y is not None:
                    # 检测到人脸
                    self.face_detected = True
                    self.last_face_time = current_time
                    
                    # 计算误差（相对于屏幕中心）
                    h, w = frame.shape[:2]
                    screen_center_x = w / 2.0
                    screen_center_y = h / 2.0
                    
                    error_x = face_center_x - screen_center_x  # 正数表示人脸在右侧
                    error_y = face_center_y - screen_center_y  # 正数表示人脸在下侧
                    
                    # 虚拟弹簧控制
                    velocity_j4, velocity_j5 = self.virtual_spring_control(error_x, error_y, dt)
                    
                    # 更新关节位置
                    self.update_joint_positions(velocity_j4, velocity_j5, dt)
                    
                    # 发送关节命令
                    self.send_joint_command()
                    
                    # 绘制调试信息
                    self.draw_debug_info(frame, face_center_x, face_center_y, error_x, error_y)
                    
                    # 日志输出（降低频率）
                    if rospy.get_time() % 1.0 < 0.05:  # 大约每1秒输出一次
                        rospy.loginfo("Face detected - Error X: %.1f px, Y: %.1f px, "
                                     "J4: %.3f rad, J5: %.3f rad",
                                     error_x, error_y, self.current_j4, self.current_j5)
                else:
                    # 未检测到人脸
                    self.face_detected = False
                    
                    # 超时保护：如果长时间检测不到人脸，停止运动
                    if self.last_face_time is not None:
                        elapsed = current_time - self.last_face_time
                        if elapsed > self.face_lost_timeout:
                            # 停止运动（保持当前位置）
                            if rospy.get_time() % 2.0 < 0.05:  # 每2秒输出一次
                                rospy.logwarn("Face lost for %.1f seconds, keeping current position", elapsed)
                            self.last_face_time = None
                            # 重置误差，避免累积
                            self.last_error_x = 0.0
                            self.last_error_y = 0.0
                    
                    # 即使没有检测到人脸，也发送当前关节命令，保持位置
                    self.send_joint_command()
                    
                    # 绘制调试信息
                    self.draw_debug_info(frame, None, None, 0.0, 0.0)
                
                # 显示图像
                cv2.imshow(window_name, frame)
                
                # 发布图像（可选）
                try:
                    image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    image_msg.header = Header()
                    image_msg.header.stamp = rospy.Time.now()
                    image_msg.header.frame_id = "camera_frame"
                    self.image_pub.publish(image_msg)
                except Exception as e:
                    rospy.logdebug("Publish image error: %s", e)
                
                # 检查按键（'q'退出）
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    rospy.loginfo("Received exit signal")
                    break
                
            except Exception as e:
                rospy.logerr("Error processing frame: %s", e)
            
            self.rate.sleep()
        
        # 清理
        cv2.destroyAllWindows()
        rospy.loginfo("Face follow node closed")


if __name__ == "__main__":
    try:
        node = FaceFollowNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Node exception: %s", e)

