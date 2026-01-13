#!/usr/bin/env python3
"""
grasp.py - 机械臂视觉抓取主脚本

功能：
1. 加载标定结果（单应性矩阵）
2. YOLO检测纸盒
3. 检测条件判断（连续检测 + 位置稳定）
4. 坐标转换（图像坐标 → 机械臂坐标）
5. 执行抓取流程

使用方法：
    rosrun visual_grasping grasp.py
"""

import cv2
import numpy as np
import rospy
import subprocess
import sys
import os
import yaml
import rospkg
from collections import deque
from threading import Lock
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Point, Quaternion
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


class DetectionTracker:
    """检测跟踪器：判断是否满足抓取条件"""
    
    def __init__(self, stable_time=5.0, position_threshold=10.0, min_detections=5):
        self.stable_time = stable_time  # 位置稳定时间（秒）
        self.position_threshold = position_threshold  # 位置稳定阈值（像素）
        self.min_detections = min_detections  # 最小连续检测帧数
        
        # 检测历史：[(center, timestamp), ...]
        self.detection_history = deque(maxlen=100)
    
    def add_detection(self, center):
        """添加一次检测结果"""
        if center is not None:
            self.detection_history.append((center, rospy.Time.now().to_sec()))
    
    def is_ready_to_grasp(self):
        """判断是否满足抓取条件"""
        if len(self.detection_history) < self.min_detections:
            return False, None
        
        current_time = rospy.Time.now().to_sec()
        
        # 获取最近stable_time秒内的检测
        recent_detections = [
            (center, ts) for center, ts in self.detection_history
            if current_time - ts <= self.stable_time
        ]
        
        if len(recent_detections) < self.min_detections:
            return False, None
        
        # 检查位置稳定性：所有检测点的位置变化都 < threshold
        centers = [center for center, _ in recent_detections]
        center_array = np.array(centers)
        
        # 计算位置变化（相对于平均位置）
        mean_center = np.mean(center_array, axis=0)
        distances = np.linalg.norm(center_array - mean_center, axis=1)
        max_distance = np.max(distances)
        
        if max_distance < self.position_threshold:
            # 使用最新的中心点作为抓取目标
            target_center = centers[-1]
            return True, target_center
        
        return False, None
    
    def reset(self):
        """重置检测历史"""
        self.detection_history.clear()


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


def load_calibration(config_path):
    """加载标定结果"""
    # 由于标定文件包含numpy对象，直接读取文件并解析homography_matrix和table_height
    try:
        import re
        with open(config_path, 'r') as f:
            lines = f.readlines()
        
        # 找到homography_matrix开始的行
        hm_start = None
        for i, line in enumerate(lines):
            if 'homography_matrix:' in line:
                hm_start = i
                break
        
        if hm_start is None:
            raise Exception("无法在标定文件中找到homography_matrix")
        
        # 读取接下来的9行（每3行组成矩阵的一行）
        # 格式：- - num1
        #       - num2
        #       - num3
        H = []
        row = []
        for i in range(hm_start + 1, min(hm_start + 10, len(lines))):
            line = lines[i].strip()
            # 提取数字
            nums = re.findall(r'[-+]?[\d.]+(?:[eE][-+]?\d+)?', line)
            if nums:
                row.append(float(nums[0]))
                # 如果收集到3个数字，完成一行
                if len(row) == 3:
                    H.append(row)
                    row = []
                    # 如果收集到3行，完成矩阵
                    if len(H) == 3:
                        break
        
        if len(H) != 3:
            raise Exception("homography_matrix格式错误，应该是3x3矩阵，实际读取到%d行" % len(H))
        
        H = np.array(H)
        
        # 找到table_height
        table_height = None
        for line in lines:
            if 'table_height:' in line:
                th_match = re.search(r'table_height:\s*([\d.]+)', line)
                if th_match:
                    table_height = float(th_match.group(1))
                    break
        
        if table_height is None:
            raise Exception("无法在标定文件中找到table_height")
        
        rospy.loginfo("标定结果加载成功")
        rospy.loginfo("桌面高度: %.3f m", table_height)
        rospy.loginfo("单应性矩阵形状: %s", H.shape)
        
        return H, table_height
        
    except Exception as e:
        rospy.logerr("加载标定结果失败: %s", str(e))
        import traceback
        traceback.print_exc()
        return None, None


def pixel_to_arm_coordinate(pixel_center, H, table_height, object_height=0.085):
    """将图像像素坐标转换为机械臂坐标"""
    u, v = pixel_center
    
    # 转换为齐次坐标
    pixel_homogeneous = np.array([u, v, 1.0])
    
    # 应用单应性矩阵
    arm_homogeneous = H @ pixel_homogeneous
    
    # 归一化
    x = arm_homogeneous[0] / arm_homogeneous[2]
    y = arm_homogeneous[1] / arm_homogeneous[2]
    
    # z坐标：桌面高度 + 物体高度的一半
    z = table_height + object_height / 2.0
    
    return x, y, z


def move_to_position(limb, target_position, speed=0.3, timeout=10.0, use_current_orientation=True):
    """使用逆运动学移动到目标位置，如果失败则尝试分段移动"""
    try:
        # 设置移动速度
        limb.set_joint_position_speed(speed)
        
        # 获取当前姿态和位置
        current_pose = limb.endpoint_pose()
        current_pos = [
            current_pose['position'].x,
            current_pose['position'].y,
            current_pose['position'].z
        ]
        
        # 创建目标位姿
        target_pose = Pose()
        target_pose.position = Point(
            x=target_position[0],
            y=target_position[1],
            z=target_position[2]
        )
        
        # 使用当前姿态（保持姿态不变，只改变位置）
        target_pose.orientation = current_pose['orientation']
        
        # 尝试直接移动到目标位置
        joint_angles = limb.ik_request(target_pose, "right_hand")
        
        if joint_angles is False or joint_angles is None:
            rospy.logwarn("直接移动到目标位置失败，尝试分段移动...")
            rospy.logwarn("目标位置: (%.3f, %.3f, %.3f)", 
                         target_position[0], target_position[1], target_position[2])
            rospy.logwarn("当前位置: (%.3f, %.3f, %.3f)", 
                         current_pos[0], current_pos[1], current_pos[2])
            
            # 分段移动策略1：先移动x, y，保持当前z
            intermediate_pos = [target_position[0], target_position[1], current_pos[2]]
            rospy.loginfo("步骤1: 先移动到 (%.3f, %.3f, %.3f)...", 
                         intermediate_pos[0], intermediate_pos[1], intermediate_pos[2])
            
            intermediate_pose = Pose()
            intermediate_pose.position = Point(
                x=intermediate_pos[0],
                y=intermediate_pos[1],
                z=intermediate_pos[2]
            )
            intermediate_pose.orientation = current_pose['orientation']
            
            joint_angles = limb.ik_request(intermediate_pose, "right_hand")
            if joint_angles is not False and joint_angles is not None:
                limb.move_to_joint_positions(joint_angles, timeout=timeout, threshold=0.01)
                rospy.sleep(0.5)
                
                # 更新当前位置
                current_pose = limb.endpoint_pose()
                current_pos = [
                    current_pose['position'].x,
                    current_pose['position'].y,
                    current_pose['position'].z
                ]
                
                # 步骤2：再移动到目标z
                rospy.loginfo("步骤2: 再移动到目标z (%.3f)...", target_position[2])
                target_pose.position = Point(
                    x=target_position[0],
                    y=target_position[1],
                    z=target_position[2]
                )
                target_pose.orientation = current_pose['orientation']
                
                joint_angles = limb.ik_request(target_pose, "right_hand")
                if joint_angles is False or joint_angles is None:
                    rospy.logwarn("分段移动也失败，尝试只移动z坐标...")
                    # 尝试只改变z，保持x, y不变
                    z_only_pose = Pose()
                    z_only_pose.position = Point(
                        x=current_pos[0],
                        y=current_pos[1],
                        z=target_position[2]
                    )
                    z_only_pose.orientation = current_pose['orientation']
                    
                    joint_angles = limb.ik_request(z_only_pose, "right_hand")
                    if joint_angles is False or joint_angles is None:
                        rospy.logerr("所有移动策略都失败")
                        return False
            else:
                rospy.logwarn("分段移动第一步也失败")
                return False
        
        # 执行移动
        limb.move_to_joint_positions(joint_angles, timeout=timeout, threshold=0.01)
        rospy.loginfo("已移动到位置: (%.3f, %.3f, %.3f)", 
                     target_position[0], target_position[1], target_position[2])
        return True
        
    except Exception as e:
        rospy.logerr("移动到目标位置失败: %s", str(e))
        import traceback
        traceback.print_exc()
        return False


def control_gripper(action):
    """控制爪子（打开/关闭）"""
    cmd = ["rosrun", "arm_follow", "j_zha.py", "--" + action]
    try:
        result = subprocess.run(cmd, timeout=5.0, capture_output=True, text=True)
        if result.returncode == 0:
            rospy.loginfo("爪子%s成功", "打开" if action == "open" else "关闭")
            return True
        else:
            rospy.logwarn("爪子%s失败: %s", "打开" if action == "open" else "关闭", result.stderr)
            return False
    except Exception as e:
        rospy.logerr("控制爪子时出错: %s", str(e))
        return False


def execute_grasp(limb, grasp_position, pre_grasp_offset=0.15, lift_offset=0.15):
    """执行抓取流程"""
    rospy.loginfo("=" * 50)
    rospy.loginfo("开始执行抓取流程")
    rospy.loginfo("抓取位置: (%.3f, %.3f, %.3f)", 
                 grasp_position[0], grasp_position[1], grasp_position[2])
    
    # 获取当前位置
    current_pose = limb.endpoint_pose()
    current_pos = [
        current_pose['position'].x,
        current_pose['position'].y,
        current_pose['position'].z
    ]
    rospy.loginfo("当前位置: (%.3f, %.3f, %.3f)", 
                 current_pos[0], current_pos[1], current_pos[2])
    rospy.loginfo("=" * 50)
    
    # 1. 移动到预抓取位置（物体上方）
    # 策略：先移动到x, y位置，使用一个安全的z高度
    rospy.loginfo("步骤1: 移动到物体上方...")
    
    # 计算一个安全的z高度：抓取位置上方10-15cm，但不超过0.85m
    safe_z = min(grasp_position[2] + 0.12, 0.85)  # 抓取位置上方12cm，但不超过0.85m
    # 如果当前位置z已经在这个范围内，使用当前位置z
    if current_pos[2] >= grasp_position[2] + 0.08 and current_pos[2] <= 0.85:
        safe_z = current_pos[2]
        rospy.loginfo("使用当前位置z: %.3f", safe_z)
    
    # 先移动到x, y位置，使用安全的z
    intermediate_pos = [grasp_position[0], grasp_position[1], safe_z]
    rospy.loginfo("移动到位置: (%.3f, %.3f, %.3f)...", 
                 intermediate_pos[0], intermediate_pos[1], intermediate_pos[2])
    
    if not move_to_position(limb, intermediate_pos, speed=0.3):
        rospy.logerr("移动到物体上方失败")
        return False
    rospy.sleep(1.0)
    
    # 现在已经在物体上方，检查是否需要调整到预抓取位置
    # 预抓取位置：抓取位置上方8-10cm
    pre_grasp_z = grasp_position[2] + 0.08  # 只高8cm
    current_pose = limb.endpoint_pose()
    current_z_after_move = current_pose['position'].z
    
    # 如果当前位置比预抓取位置高很多（>3cm），才下降
    if current_z_after_move > pre_grasp_z + 0.03:
        rospy.loginfo("步骤1.2: 下降到预抓取位置 (z=%.3f)...", pre_grasp_z)
        pre_grasp_position = [grasp_position[0], grasp_position[1], pre_grasp_z]
        if not move_to_position(limb, pre_grasp_position, speed=0.2):
            rospy.logwarn("下降到预抓取位置失败，使用当前位置")
            # 如果失败，使用当前位置
            pre_grasp_position[2] = current_z_after_move
    else:
        rospy.loginfo("当前位置已合适，无需调整")
        pre_grasp_position = [grasp_position[0], grasp_position[1], current_z_after_move]
    
    rospy.sleep(1.0)
    
    # 2. 打开爪子
    rospy.loginfo("步骤2: 打开爪子...")
    if not control_gripper("open"):
        rospy.logerr("打开爪子失败")
        return False
    rospy.sleep(1.0)
    
    # 3. 下降到抓取位置
    rospy.loginfo("步骤3: 下降到抓取位置...")
    if not move_to_position(limb, grasp_position, speed=0.1):
        rospy.logerr("下降到抓取位置失败")
        return False
    rospy.sleep(1.0)
    
    # 4. 关闭爪子（抓取）
    rospy.loginfo("步骤4: 关闭爪子（抓取）...")
    if not control_gripper("close"):
        rospy.logerr("关闭爪子失败")
        return False
    rospy.sleep(1.0)
    
    # 5. 抬起
    lift_position = [
        grasp_position[0],
        grasp_position[1],
        grasp_position[2] + lift_offset
    ]
    rospy.loginfo("步骤5: 抬起...")
    if not move_to_position(limb, lift_position, speed=0.3):
        rospy.logerr("抬起失败")
        return False
    rospy.sleep(1.0)
    
    # 6. 放开物体
    rospy.loginfo("步骤6: 放开物体...")
    if not control_gripper("open"):
        rospy.logerr("放开物体失败")
        return False
    rospy.sleep(1.0)
    
    rospy.loginfo("抓取流程完成")
    return True


def draw_detection_result(image, box, center, status_text, grasp_count):
    """在图像上绘制检测结果"""
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
    
    # 状态信息
    cv2.putText(img, status_text, (10, y_offset), 
                font, 0.7, (255, 255, 255), 2)
    y_offset += line_height
    
    # 已抓取次数
    cv2.putText(img, f"已抓取次数: {grasp_count}", (10, y_offset), 
                font, 0.7, (255, 255, 255), 2)
    y_offset += line_height
    
    # 操作提示
    cv2.putText(img, "按 'q' 退出", 
                (10, img.shape[0] - 20), font, 0.5, (200, 200, 200), 1)
    
    return img


def main():
    rospy.init_node("visual_grasping", anonymous=False)
    
    # 获取参数
    model_path = rospy.get_param("~model", "/home/mycar/YOLO/ultralytics-8.3.163/yolov8n_landmark.pt")
    camera_topic = rospy.get_param("~camera_topic", "/io/internal_camera/head_camera/image_rect_color")
    
    # 获取标定结果路径
    rospack = rospkg.RosPack()
    calibration_package_path = rospack.get_path('Position_Calibration')
    default_calibration_path = os.path.join(calibration_package_path, "config", "calibration.yaml")
    calibration_path = rospy.get_param("~calibration_path", default_calibration_path)
    
    table_height = rospy.get_param("~table_height", 0.746)  # 桌面高度（米）
    object_height = rospy.get_param("~object_height", 0.085)  # 物体高度（米）
    conf_threshold = rospy.get_param("~conf_threshold", 0.5)  # YOLO置信度阈值
    
    # 检测参数
    stable_time = rospy.get_param("~stable_time", 5.0)  # 位置稳定时间（秒）
    position_threshold = rospy.get_param("~position_threshold", 10.0)  # 位置稳定阈值（像素）
    min_detections = rospy.get_param("~min_detections", 5)  # 最小连续检测帧数
    
    rospy.loginfo("加载YOLO模型: %s", model_path)
    try:
        model = YOLO(model_path)
        rospy.loginfo("YOLO模型加载成功")
    except Exception as e:
        rospy.logerr("加载YOLO模型失败: %s", str(e))
        return
    
    # 加载标定结果
    rospy.loginfo("加载标定结果: %s", calibration_path)
    H, calib_table_height = load_calibration(calibration_path)
    if H is None:
        rospy.logerr("无法加载标定结果，退出")
        return
    
    # 使用标定文件中的桌面高度，如果参数指定了则使用参数
    if calib_table_height is not None:
        table_height = calib_table_height
    
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
    
    # 初始化机械臂
    limb = intera_interface.Limb("right")
    initialize_arm_position()
    
    # 创建检测跟踪器
    tracker = DetectionTracker(stable_time, position_threshold, min_detections)
    
    # 状态变量
    grasp_count = 0
    is_grasping = False
    
    window_name = "Visual Grasping"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    
    rospy.loginfo("=" * 50)
    rospy.loginfo("视觉抓取系统启动！")
    rospy.loginfo("系统将自动检测并抓取物体")
    rospy.loginfo("按 'q' 退出")
    rospy.loginfo("=" * 50)
    
    rate = rospy.Rate(10)  # 10Hz
    
    try:
        while not rospy.is_shutdown():
            # 如果正在抓取，跳过检测
            if is_grasping:
                rate.sleep()
                continue
            
            # 获取最新图像
            cv_image = image_stream.get_latest_image()
            if cv_image is None:
                rate.sleep()
                continue
            
            # YOLO检测
            results = model(cv_image, verbose=False, conf=conf_threshold)
            
            # 解析检测结果
            detected = False
            current_box = None
            current_center = None
            
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
            
            # 更新检测跟踪器
            if detected:
                tracker.add_detection(current_center)
            else:
                # 如果检测不到，重置跟踪器
                tracker.reset()
            
            # 检查是否满足抓取条件
            ready, target_center = tracker.is_ready_to_grasp()
            
            # 绘制检测结果
            status_text = ""
            if is_grasping:
                status_text = "抓取中..."
            elif ready:
                status_text = "准备抓取！位置稳定"
            elif detected:
                recent_count = len([d for d in tracker.detection_history 
                                   if rospy.Time.now().to_sec() - d[1] <= tracker.stable_time])
                status_text = f"检测到物体，稳定中... ({recent_count}/{min_detections}帧)"
            else:
                status_text = "等待检测物体..."
            
            display_image = draw_detection_result(
                cv_image, current_box, current_center, 
                status_text, grasp_count
            )
            cv2.imshow(window_name, display_image)
            
            # 键盘控制
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                rospy.loginfo("用户退出")
                break
            
            # 如果满足抓取条件，执行抓取
            if ready and target_center is not None and not is_grasping:
                is_grasping = True
                rospy.loginfo("检测到稳定物体，开始抓取...")
                
                # 计算抓取位置
                x, y, z = pixel_to_arm_coordinate(target_center, H, table_height, object_height)
                rospy.loginfo("计算得到抓取位置: (%.3f, %.3f, %.3f)", x, y, z)
                
                # 执行抓取
                if execute_grasp(limb, [x, y, z]):
                    grasp_count += 1
                    rospy.loginfo("抓取成功！已抓取次数: %d", grasp_count)
                else:
                    rospy.logerr("抓取失败")
                
                # 回到初始化位置
                rospy.loginfo("回到初始化位置...")
                initialize_arm_position()
                
                # 重置检测跟踪器
                tracker.reset()
                is_grasping = False
            
            rate.sleep()
    
    except KeyboardInterrupt:
        rospy.loginfo("程序被中断")
    except Exception as e:
        rospy.logerr("程序运行出错: %s", str(e))
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
        rospy.loginfo("程序退出")


if __name__ == "__main__":
    main()

