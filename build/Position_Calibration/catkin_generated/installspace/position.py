#!/usr/bin/env python3
"""
position.py - 机械臂视觉抓取标定脚本

功能：
1. 使用YOLO模型检测纸盒
2. 交互式收集5个点对（图像坐标 ↔ 机械臂坐标）
3. 计算单应性矩阵
4. 保存标定结果

使用方法：
    rosrun Position_Calibration position.py
"""

import cv2
import numpy as np
import rospy
import subprocess
import sys
import os
import yaml
import rospkg
from threading import Lock
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import intera_interface
from ultralytics import YOLO


class ROSImageStream:
    """ROS图像流包装器"""
    
    def __init__(self, topic: str, timeout: float = 30.0):
        self.topic = topic
        self.timeout = timeout
        self.bridge = CvBridge()
        self.latest_image = None
        self.lock = Lock()
        self.sub = rospy.Subscriber(topic, Image, self._image_callback, queue_size=1, buff_size=2**23)
        rospy.loginfo("订阅ROS图像话题: %s", topic)
        
    def _image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.lock:
                self.latest_image = cv_image.copy()
        except CvBridgeError as e:
            rospy.logerr("CvBridge错误: %s", e)
    
    def get_latest_image(self):
        """获取最新图像"""
        with self.lock:
            return self.latest_image.copy() if self.latest_image is not None else None


def initialize_arm_position():
    """初始化机械臂到固定位置"""
    rospy.loginfo("正在初始化机械臂到固定位置...")
    
    # 按顺序执行：j1 → j2 → j3 → j4 → j5 → j6 → j0 (gnd_set) → head_set
    commands = [
        ("j1_set.py", "-1.0"),
        ("j2_set.py", "0.0"),
        ("j3_set.py", "1.0"),
        ("j4_set.py", "0.0"),
        ("j5_set.py", "1.5"),
        ("j6_set.py", "0.0"),
        ("gnd_set.py", "1.5"),   # j0
        ("head_set.py", "-1.5"),
    ]
    
    for script_name, angle in commands:
        cmd = ["rosrun", "arm_follow", script_name, "--angle", angle]
        rospy.loginfo("执行: %s", " ".join(cmd))
        try:
            result = subprocess.run(cmd, timeout=15.0, capture_output=True, text=True)
            if result.returncode != 0:
                rospy.logwarn("命令执行失败: %s, 错误: %s", " ".join(cmd), result.stderr)
            else:
                rospy.loginfo("完成: %s", script_name)
        except subprocess.TimeoutExpired:
            rospy.logwarn("命令超时: %s", " ".join(cmd))
        except Exception as e:
            rospy.logerr("执行命令时出错: %s, 错误: %s", " ".join(cmd), str(e))
        
        rospy.sleep(0.5)  # 等待每个关节移动完成
    
    rospy.loginfo("机械臂初始化完成")
    rospy.sleep(2.0)  # 等待机械臂完全到位


def get_arm_endpoint_position():
    """获取机械臂末端执行器在base坐标系下的位置"""
    try:
        limb = intera_interface.Limb("right")
        endpoint = limb.endpoint_pose()
        x = endpoint['position'].x
        y = endpoint['position'].y
        z = endpoint['position'].z
        return x, y, z
    except Exception as e:
        rospy.logerr("获取机械臂位置失败: %s", str(e))
        return None, None, None


def calculate_homography(pixel_points, arm_points):
    """计算单应性矩阵"""
    if len(pixel_points) < 4:
        rospy.logerr("点对数量不足，至少需要4个点")
        return None
    
    pixel_pts = np.array(pixel_points, dtype=np.float32)
    arm_pts = np.array(arm_points, dtype=np.float32)
    
    # 只使用x, y坐标（忽略z）
    arm_xy = arm_pts[:, :2]
    
    # 计算单应性矩阵
    H, mask = cv2.findHomography(pixel_pts, arm_xy, method=cv2.RANSAC, ransacReprojThreshold=3.0)
    
    if H is None:
        rospy.logerr("计算单应性矩阵失败")
        return None
    
    # 计算重投影误差
    errors = []
    for i in range(len(pixel_points)):
        pixel_pt = np.array([pixel_points[i][0], pixel_points[i][1], 1.0])
        projected = H @ pixel_pt
        projected = projected / projected[2]
        error = np.linalg.norm(projected[:2] - arm_xy[i])
        errors.append(error)
    
    avg_error = np.mean(errors)
    max_error = np.max(errors)
    
    rospy.loginfo("单应性矩阵计算完成")
    rospy.loginfo("平均重投影误差: %.4f m", avg_error)
    rospy.loginfo("最大重投影误差: %.4f m", max_error)
    
    return H, avg_error, max_error


def save_calibration(H, table_height, pixel_points, arm_points, config_path):
    """保存标定结果到YAML文件"""
    calibration_data = {
        'homography_matrix': H.tolist(),
        'table_height': table_height,
        'calibration_points': {
            'pixel_points': pixel_points,
            'arm_points': arm_points
        },
        'calibration_date': rospy.Time.now().to_sec()
    }
    
    try:
        with open(config_path, 'w') as f:
            yaml.dump(calibration_data, f, default_flow_style=False, allow_unicode=True)
        rospy.loginfo("标定结果已保存到: %s", config_path)
        return True
    except Exception as e:
        rospy.logerr("保存标定结果失败: %s", str(e))
        return False


def draw_detection_result(image, box, center, point_count, status_text=""):
    """在图像上绘制检测结果和标定信息"""
    img = image.copy()
    
    if box is not None:
        # 绘制检测框
        x1, y1, x2, y2 = box
        cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        
        # 绘制中心点
        if center is not None:
            cx, cy = int(center[0]), int(center[1])
            cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)
            cv2.circle(img, (cx, cy), 10, (0, 0, 255), 2)
    
    # 绘制文字信息
    font = cv2.FONT_HERSHEY_SIMPLEX
    y_offset = 30
    line_height = 30
    
    # 已收集的点数
    cv2.putText(img, f"已收集点对: {point_count}/5", (10, y_offset), 
                font, 0.7, (255, 255, 255), 2)
    y_offset += line_height
    
    # 状态信息
    if status_text:
        cv2.putText(img, status_text, (10, y_offset), 
                    font, 0.7, (0, 255, 255), 2)
        y_offset += line_height
    
    # 操作提示
    cv2.putText(img, "按 'y' 确认识别 | 'r' 记录点对 | 'c' 清除 | 'q' 退出", 
                (10, img.shape[0] - 20), font, 0.5, (200, 200, 200), 1)
    
    return img


def main():
    rospy.init_node("position_calibration", anonymous=False)
    
    # 获取参数
    model_path = rospy.get_param("~model", "/home/mycar/YOLO/ultralytics-8.3.163/yolov8n_landmark.pt")
    camera_topic = rospy.get_param("~camera_topic", "/io/internal_camera/head_camera/image_rect_color")
    # 获取ROS包路径
    import rospkg
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('Position_Calibration')
    default_config_path = os.path.join(package_path, "config", "calibration.yaml")
    
    config_path = rospy.get_param("~config_path", default_config_path)
    table_height = rospy.get_param("~table_height", 0.746)  # 桌面高度（米）
    conf_threshold = rospy.get_param("~conf_threshold", 0.5)  # YOLO置信度阈值
    
    # 确保config目录存在
    config_dir = os.path.dirname(config_path)
    if not os.path.exists(config_dir):
        os.makedirs(config_dir)
    
    rospy.loginfo("加载YOLO模型: %s", model_path)
    try:
        model = YOLO(model_path)
        rospy.loginfo("YOLO模型加载成功")
    except Exception as e:
        rospy.logerr("加载YOLO模型失败: %s", str(e))
        return
    
    # 初始化摄像头
    rospy.loginfo("初始化摄像头...")
    cameras = intera_interface.Cameras()
    camera_name = "head_camera"
    
    if not cameras.verify_camera_exists(camera_name):
        rospy.logerr("头部摄像头 '%s' 未找到！", camera_name)
        return
    
    if not cameras.start_streaming(camera_name):
        rospy.logerr("启动摄像头流失败！")
        return
    
    rospy.loginfo("摄像头流已启动")
    
    # 设置自动曝光和增益
    cameras.set_exposure(camera_name, -1)
    cameras.set_gain(camera_name, -1)
    
    # 等待图像话题可用
    rospy.loginfo("等待图像话题可用...")
    try:
        rospy.wait_for_message(camera_topic, Image, timeout=5.0)
        rospy.loginfo("图像话题已可用")
    except rospy.ROSException:
        rospy.logwarn("无法从 %s 接收消息，继续尝试...", camera_topic)
    
    # 创建图像流
    image_stream = ROSImageStream(camera_topic, timeout=30.0)
    
    # 初始化机械臂到固定位置
    initialize_arm_position()
    
    # 标定数据
    pixel_points = []  # 图像坐标 (u, v)
    arm_points = []    # 机械臂坐标 (x, y, z)
    
    # 当前检测状态
    current_box = None
    current_center = None
    confirmed = False
    
    # 已确认的检测结果（锁定状态，不会因为摄像头移动而丢失）
    confirmed_box = None
    confirmed_center = None
    
    window_name = "Position Calibration"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    
    rospy.loginfo("=" * 50)
    rospy.loginfo("标定开始！")
    rospy.loginfo("操作说明：")
    rospy.loginfo("  1. 在摄像头视野的不同位置放置纸盒")
    rospy.loginfo("  2. 按 'y' 确认识别结果")
    rospy.loginfo("  3. 手动控制机械臂，将j6移动到纸盒中心正上方")
    rospy.loginfo("  4. 按 'r' 记录点对")
    rospy.loginfo("  5. 重复步骤1-4，共收集5个点对")
    rospy.loginfo("  按 'q' 退出，按 'c' 清除当前点")
    rospy.loginfo("=" * 50)
    
    rate = rospy.Rate(10)  # 10Hz
    
    try:
        while not rospy.is_shutdown():
            # 获取最新图像
            cv_image = image_stream.get_latest_image()
            if cv_image is None:
                rate.sleep()
                continue
            
            # YOLO检测
            results = model(cv_image, verbose=False, conf=conf_threshold)
            
            # 解析检测结果
            detected = False
            if len(results) > 0 and len(results[0].boxes) > 0:
                # 获取第一个检测框（置信度最高的）
                box = results[0].boxes.xyxy[0].cpu().numpy()  # [x1, y1, x2, y2]
                conf = results[0].boxes.conf[0].cpu().numpy()
                
                if conf >= conf_threshold:
                    current_box = box
                    # 计算中心点
                    center_x = (box[0] + box[2]) / 2
                    center_y = (box[1] + box[3]) / 2
                    current_center = (center_x, center_y)
                    detected = True
                    # 只有在未确认状态下，检测到新物体才需要重新确认
                    if not confirmed:
                        confirmed = False  # 检测到新物体，需要重新确认
            
            # 绘制检测结果
            # 如果已确认，使用已确认的数据；否则使用当前检测的数据
            display_box = confirmed_box if confirmed else current_box
            display_center = confirmed_center if confirmed else current_center
            
            status_text = ""
            if len(pixel_points) < 5:
                if detected and not confirmed:
                    status_text = "检测到物体！按 'y' 确认"
                elif confirmed:
                    status_text = "已确认！移动机械臂后按 'r' 记录（已锁定检测结果）"
                else:
                    status_text = "等待检测物体..."
            else:
                status_text = "已收集5个点对，准备计算单应性矩阵..."
            
            display_image = draw_detection_result(
                cv_image, display_box, display_center, 
                len(pixel_points), status_text
            )
            cv2.imshow(window_name, display_image)
            
            # 键盘控制
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                rospy.loginfo("用户退出标定")
                break
            elif key == ord('y'):
                if detected and current_center is not None:
                    confirmed = True
                    # 锁定并保存当前检测结果
                    if current_box is not None:
                        confirmed_box = current_box.copy()
                    else:
                        confirmed_box = None
                    confirmed_center = (current_center[0], current_center[1])
                    rospy.loginfo("已确认识别结果，中心点: (%.1f, %.1f) [已锁定]", 
                                 confirmed_center[0], confirmed_center[1])
                    rospy.loginfo("现在可以移动机械臂，检测结果已保存，不会丢失")
                else:
                    rospy.logwarn("未检测到物体，无法确认")
            elif key == ord('r'):
                if confirmed and confirmed_center is not None:
                    # 获取机械臂位置
                    x, y, z = get_arm_endpoint_position()
                    if x is not None and y is not None:
                        # 使用已确认并锁定的检测结果
                        pixel_points.append([confirmed_center[0], confirmed_center[1]])
                        arm_points.append([x, y, z])
                        rospy.loginfo("=" * 50)
                        rospy.loginfo("点对 %d/5 已记录:", len(pixel_points))
                        rospy.loginfo("  图像坐标: (%.1f, %.1f)", 
                                     confirmed_center[0], confirmed_center[1])
                        rospy.loginfo("  机械臂坐标: (%.4f, %.4f, %.4f)", x, y, z)
                        rospy.loginfo("=" * 50)
                        
                        # 清除确认状态和保存的数据，准备下一个点
                        confirmed = False
                        confirmed_box = None
                        confirmed_center = None
                        current_box = None
                        current_center = None
                        
                        # 如果还没收集够5个点，初始化机械臂位置
                        if len(pixel_points) < 5:
                            rospy.loginfo("准备收集下一个点对，初始化机械臂位置...")
                            initialize_arm_position()
                        else:
                            rospy.loginfo("已收集5个点对，准备计算单应性矩阵...")
                    else:
                        rospy.logerr("获取机械臂位置失败，无法记录点对")
                else:
                    rospy.logwarn("请先按 'y' 确认识别结果")
            elif key == ord('c'):
                if len(pixel_points) > 0:
                    pixel_points.pop()
                    arm_points.pop()
                    rospy.loginfo("已清除最后一个点对，当前点数: %d", len(pixel_points))
                elif confirmed:
                    # 如果已确认但未记录，清除确认状态
                    confirmed = False
                    confirmed_box = None
                    confirmed_center = None
                    rospy.loginfo("已清除确认状态，可以重新检测")
                else:
                    rospy.logwarn("没有可清除的点对或确认状态")
            
            # 如果收集够5个点，计算单应性矩阵
            if len(pixel_points) == 5:
                rospy.loginfo("开始计算单应性矩阵...")
                result = calculate_homography(pixel_points, arm_points)
                
                if result is not None:
                    H, avg_error, max_error = result
                    
                    # 保存标定结果
                    if save_calibration(H, table_height, pixel_points, arm_points, config_path):
                        rospy.loginfo("=" * 50)
                        rospy.loginfo("标定完成！")
                        rospy.loginfo("标定结果已保存到: %s", config_path)
                        rospy.loginfo("平均重投影误差: %.4f m", avg_error)
                        rospy.loginfo("最大重投影误差: %.4f m", max_error)
                        rospy.loginfo("=" * 50)
                        
                        # 显示标定结果
                        status_text = f"标定完成！误差: {avg_error:.4f}m"
                        display_image = draw_detection_result(
                            cv_image, None, None, 5, status_text
                        )
                        cv2.imshow(window_name, display_image)
                        cv2.waitKey(3000)  # 显示3秒
                        
                        break
                    else:
                        rospy.logerr("保存标定结果失败")
                else:
                    rospy.logerr("计算单应性矩阵失败，请重新标定")
            
            rate.sleep()
    
    except KeyboardInterrupt:
        rospy.loginfo("标定被中断")
    except Exception as e:
        rospy.logerr("标定过程中出错: %s", str(e))
        import traceback
        traceback.print_exc()
    finally:
        # 清理
        rospy.loginfo("停止摄像头流...")
        try:
            cameras.stop_streaming(camera_name)
        except:
            pass
        cv2.destroyAllWindows()
        rospy.loginfo("标定脚本退出")


if __name__ == "__main__":
    main()

