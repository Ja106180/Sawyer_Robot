#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import intera_interface
import os
import math
import rospkg
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose, Point, Quaternion
from ultralytics import YOLO
import signal
import sys

# ==========================================
# 核心配置参数 (Configuration)
# ==========================================
CAMERA_INDEX = 0  # USB 摄像头设备号 (通常为 0 或 1，对应 /dev/video0)

# 高度配置 (相对高度模式)
# 根据您的最新反馈，完全真相大白了！
# 您读到的 35cm 是 right_hand (末端法兰) 的高度，而夹爪长 16.5cm。
# 目标：让夹爪尖端距离桌面 5cm。
# 计算：当前尖端高度是 35 - 16.5 = 18.5cm。要降到 3cm (根据反馈再降1cm)，必须下降 18.5 - 3 = 15.5cm。
DESCEND_DISTANCE = 0.155   # 抓取时向下伸长的距离 (0.155m = 15.5cm)
LIFT_DISTANCE = 0.10       # 抓取完成后抬高的距离 (0.10m = 10cm)

# 摄像头物理安装偏移补偿 (如果使用单应性矩阵 H，通常设为 0，因为标定已包含偏移)
# 但如果标定后存在固定的整体平移误差（例如整体偏上/偏左一格），可以通过微调下面两个值来修正！
GRASP_OFFSET_X = 0.0    # 抓取时的 X 方向微调 (例如 0.04 表示向前微调 4cm，-0.04 表示向后)
GRASP_OFFSET_Y = 0.0    # 抓取时的 Y 方向微调 (例如 0.04 表示向左微调 4cm，-0.04 表示向右)

# 以下是兼容旧模式的参数
CAMERA_OFFSET_X = 0.0 
CAMERA_OFFSET_Y = 0.0

# 物理网格参数
GRID_PHYSICAL_DIST = 0.07          # 相邻数字中心物理距离 7cm (0.07m)

class NumberGraspSkill:
    def __init__(self):
        rospy.init_node('number_grasp_skill')
        
        self.limb = intera_interface.Limb("right")
        self.gripper = intera_interface.Gripper("right")
        self.head = intera_interface.Head()
        
        # 状态变量
        self.target_number = None
        self.pixel_to_meter_ratio = None
        self.state = "INIT"
        
        # 订阅目标数字
        rospy.Subscriber('/sawyer_target_number', Int32, self.target_callback)
        
        # 加载 YOLOv8 模型
        rospack = rospkg.RosPack()
        weights_path = os.path.join(rospack.get_path('openclaw_sawyer'), 'weights', 'best.pt')
        
        rospy.loginfo(f"Loading YOLOv8 model from {weights_path}...")
        try:
            self.model = YOLO(weights_path)
            rospy.loginfo("Model loaded successfully.")
        except Exception as e:
            rospy.logerr(f"Failed to load YOLO model: {e}")
            return
            
        # 尝试加载单应性标定矩阵 (完美的抓取坐标映射)
        self.H = None
        try:
            calib_path = os.path.join(rospack.get_path('Position_Calibration'), 'config', 'calibration.yaml')
            import yaml
            with open(calib_path, 'r') as f:
                calib_data = yaml.safe_load(f)
                self.H = np.array(calib_data['homography_matrix'])
                rospy.loginfo(f"Successfully loaded Homography Matrix from {calib_path}")
        except Exception as e:
            rospy.logwarn(f"Could not load calibration.yaml: {e}. Will fallback to basic ratio math.")
            
        # 打开 USB 摄像头
        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        if not self.cap.isOpened():
            rospy.logerr(f"Cannot open USB camera with index {CAMERA_INDEX}! Try changing CAMERA_INDEX to 1.")
            return
            
        rospy.loginfo("Number Grasp Skill initialized. Starting control loop.")
        self.run()

    def target_callback(self, msg):
        self.target_number = msg.data
        rospy.loginfo(f"Received target number: {self.target_number}")

    def perform_init_sequence(self):
        """严格串行的安全初始化序列"""
        rospy.loginfo("Starting STRICT SAFE initialization sequence...")
        try:
            # 调低速度，保证安全 (0.1 = 10% 速度)
            self.limb.set_joint_position_speed(0.1)

            # 步骤 1: 头部归正
            rospy.loginfo("Step 1: Head Pan to 0.0")
            self.head.set_pan(0.0)
            rospy.sleep(1.5)
            
            # 获取当前所有关节角度，避免意外移动
            current = self.limb.joint_angles()
            
            # 步骤 2: J0 归正
            rospy.loginfo("Step 2: Joint 0 to 0.0")
            current['right_j0'] = 0.0
            self.limb.move_to_joint_positions(current)
            rospy.sleep(0.5)
            
            # 步骤 3: J1 归正
            rospy.loginfo("Step 3: Joint 1 to -0.5")
            current['right_j1'] = -0.5
            self.limb.move_to_joint_positions(current)
            rospy.sleep(0.5)
            
            # 步骤 4: 剩余关节同时移动到观测姿态
            rospy.loginfo("Step 4: Remaining joints to observation pose")
            full_pose = {
                'right_j0': 0.0,
                'right_j1': -0.5,
                'right_j2': 0.0,
                'right_j3': 1.0,
                'right_j4': 0.0,
                'right_j5': 1.1,
                'right_j6': -1.3
            }
            self.limb.move_to_joint_positions(full_pose)
            rospy.sleep(0.5)
            
            # 恢复正常抓取速度 (可以根据需要调回 0.3 或保持 0.1)
            self.limb.set_joint_position_speed(0.2)
            
            # 确保夹爪是松开的
            self.gripper.open()
            
            # 清理摄像头积压的旧缓存帧，避免识别到移动过程中的残影
            if hasattr(self, 'cap') and self.cap.isOpened():
                for _ in range(10):
                    self.cap.grab()
                    
            rospy.loginfo("Initialization complete. Arm is in observation pose.")
            return True
        except Exception as e:
            rospy.logerr(f"Initialization failed: {e}")
            return False

    def calculate_ratio(self, detections):
        """全数字动态平均自动标定"""
        if len(detections) < 2:
            return None
            
        ratios = []
        # 两两配对，找出所有相邻的数字
        for i in range(len(detections)):
            for j in range(i+1, len(detections)):
                n1 = detections[i]['num']
                n2 = detections[j]['num']
                
                # 九宫格行列索引 (1-indexed: 1 到 9)
                col1 = (n1 - 1) % 3
                row1 = (n1 - 1) // 3
                col2 = (n2 - 1) % 3
                row2 = (n2 - 1) // 3
                
                # 判断是否是水平或垂直相邻 (曼哈顿距离为 1)
                if abs(col1 - col2) + abs(row1 - row2) == 1:
                    dx = detections[i]['cx'] - detections[j]['cx']
                    dy = detections[i]['cy'] - detections[j]['cy']
                    pixel_dist = math.sqrt(dx**2 + dy**2)
                    
                    if pixel_dist > 10: # 避免除以 0 或极小噪声
                        ratio = GRID_PHYSICAL_DIST / pixel_dist
                        ratios.append(ratio)
        
        if len(ratios) > 0:
            # 过滤掉可能的极端异常值，取平均
            avg_ratio = sum(ratios) / len(ratios)
            return avg_ratio
        return None

    def execute_grasp(self, target_cx, target_cy, w, h):
        rospy.loginfo(f"Executing grasp for pixel ({target_cx:.1f}, {target_cy:.1f})")
        try:
            current_pose = self.limb.endpoint_pose()
            cx = current_pose['position'].x
            cy = current_pose['position'].y
            cz = current_pose['position'].z
            ori = current_pose['orientation'] # This is a Quaternion
            
            # 使用超级精准的单应性矩阵映射坐标！
            if self.H is not None:
                rospy.loginfo("Using Homography matrix for precise absolute Cartesian mapping!")
                pixel_pt = np.array([target_cx, target_cy, 1.0])
                arm_pt = self.H @ pixel_pt
                
                # 加上全局微调偏移量 (解决固定偏移一格的问题)
                target_x = (arm_pt[0] / arm_pt[2]) + GRASP_OFFSET_X
                target_y = (arm_pt[1] / arm_pt[2]) + GRASP_OFFSET_Y
            else:
                # 兼容模式：如果没有标定文件，则退回到旧的比例计算法
                rospy.logwarn("Homography matrix missing! Falling back to linear ratio math.")
                du = target_cx - w / 2.0
                dv = target_cy - h / 2.0
                offset_x = -dv * self.pixel_to_meter_ratio 
                offset_y = -du * self.pixel_to_meter_ratio
                target_x = cx + offset_x + CAMERA_OFFSET_X
                target_y = cy + offset_y + CAMERA_OFFSET_Y
            
            rospy.loginfo(f"Calculated Absolute Target -> X: {target_x:.4f}, Y: {target_y:.4f}")
            
            # 动态高度计算
            observe_h = cz
            safe_grasp_h = cz - DESCEND_DISTANCE
            lift_h = safe_grasp_h + LIFT_DISTANCE
            
            # Helper to create Pose
            def create_pose(tx, ty, tz, orientation):
                p = Pose()
                p.position.x = tx
                p.position.y = ty
                p.position.z = tz
                p.orientation.x = orientation.x
                p.orientation.y = orientation.y
                p.orientation.z = orientation.z
                p.orientation.w = orientation.w
                return p
            
            # 移动步 1: 在观测高度水平对准
            rospy.loginfo(f"Move 1: Horizontal align at Z = {observe_h:.3f}m...")
            self.limb.set_joint_position_speed(0.2) # 恢复正常速度 20%
            pose1 = create_pose(target_x, target_y, observe_h, ori)
            # 致命修正：传入 right_hand，与 current_pose 的坐标系对齐！
            ik1 = self.limb.ik_request(pose1, "right_hand")
            if not ik1:
                rospy.logerr("IK failed for horizontal align.")
                return
            self.limb.move_to_joint_positions(ik1)
            
            # 对齐后等待 1 秒
            rospy.loginfo("Waiting 1 second after horizontal align...")
            rospy.sleep(1.0)
            
            # 移动步 2: 垂直下降抓取 (速度放慢 5%)
            rospy.loginfo(f"Move 2: Descend to Z = {safe_grasp_h:.3f}m...")
            self.limb.set_joint_position_speed(0.15) # 下降速度减慢到 15%
            pose2 = create_pose(target_x, target_y, safe_grasp_h, ori)
            ik2 = self.limb.ik_request(pose2, "right_hand")
            if not ik2:
                rospy.logerr("IK failed for descend.")
                return
            self.limb.move_to_joint_positions(ik2)
            
            # 下降后等待 1 秒
            rospy.loginfo("Waiting 1 second after descend...")
            rospy.sleep(1.0)
            
            # 动作 1: 闭合夹爪
            rospy.loginfo("Action: Close gripper")
            self.gripper.close()
            
            # 闭合夹爪后等待 1 秒
            rospy.loginfo("Waiting 1 second after closing gripper...")
            rospy.sleep(1.0)
            
            # 移动步 3: 抬高 10cm
            rospy.loginfo(f"Move 3: Lift up by {LIFT_DISTANCE}m...")
            self.limb.set_joint_position_speed(0.2) # 抬起恢复正常速度 20%
            pose3 = create_pose(target_x, target_y, lift_h, ori)
            ik3 = self.limb.ik_request(pose3, "right_hand")
            if not ik3:
                rospy.logwarn("IK failed for lift. Lifting to observe height instead.")
                self.limb.move_to_joint_positions(ik1)
            else:
                self.limb.move_to_joint_positions(ik3)
            
            # 等待 3 秒钟
            rospy.loginfo("Waiting 3 seconds before release...")
            rospy.sleep(3.0)
            
            # 动作 2: 松开夹爪 (释放物体)
            rospy.loginfo("Action: Open gripper (Release)")
            self.gripper.open()
            rospy.sleep(1.0)
            
            rospy.loginfo("Grasp sequence complete. Returning to observation center.")
            
            # 返回正中心的观测点 (不需要重头进行全套初始化，直接关节归位即可)
            full_pose = {
                'right_j0': 0.0,
                'right_j1': -0.5,
                'right_j2': 0.0,
                'right_j3': 1.0,
                'right_j4': 0.0,
                'right_j5': 1.1,
                'right_j6': -1.3
            }
            self.limb.move_to_joint_positions(full_pose)
            rospy.sleep(0.5)
            
        except Exception as e:
            rospy.logerr(f"Grasp execution failed: {e}")
            # 发生异常则彻底复位恢复安全姿态
            self.perform_init_sequence()

    def run(self):
        # 启动时执行严格安全初始化
        if not self.perform_init_sequence():
            return
            
        self.state = "WAITING"
        rate = rospy.Rate(10) # 10Hz 处理循环
        
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn_throttle(2, "Failed to read from camera. Check USB connection.")
                rate.sleep()
                continue
                
            if self.state == "WAITING":
                if self.target_number is not None:
                    self.state = "SEARCHING"
                else:
                    # 在等待状态时显示实时画面，表明正在等待
                    cv2.putText(frame, "WAITING FOR COMMAND...", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.imshow("YOLO Vision", frame)
                    cv2.waitKey(1)
                    
                    # 允许用户点击 X 强行关闭窗口并退出程序 (专门解决关不掉的问题)
                    try:
                        if cv2.getWindowProperty("YOLO Vision", cv2.WND_PROP_VISIBLE) < 1:
                            rospy.loginfo("User clicked X on window. Exiting...")
                            break
                    except:
                        pass
                    
            elif self.state == "SEARCHING":
                # 推理图像
                results = self.model(frame, verbose=False)
                
                # 绘制带 YOLO 框的图像以供显示
                annotated_frame = results[0].plot()
                cv2.imshow("YOLO Vision", annotated_frame)
                cv2.waitKey(1)
                
                try:
                    if cv2.getWindowProperty("YOLO Vision", cv2.WND_PROP_VISIBLE) < 1:
                        rospy.loginfo("User clicked X on window. Exiting...")
                        break
                except:
                    pass
                
                detections_dict = {} # 使用字典去重，保留置信度最高的
                target_cx, target_cy = None, None
                
                # 解析 YOLO 结果
                for r in results:
                    boxes = r.boxes
                    for box in boxes:
                        cls_id = int(box.cls[0])
                        num = cls_id + 1 # YOLO index 0 是数字 1
                        conf = float(box.conf[0])
                        
                        x1, y1, x2, y2 = box.xyxy[0]
                        cx = float((x1 + x2) / 2)
                        cy = float((y1 + y2) / 2)
                        
                        # 去重逻辑：如果同一个数字被识别出多次，保留置信度最高的框
                        if num not in detections_dict or conf > detections_dict[num]['conf']:
                            detections_dict[num] = {'num': num, 'cx': cx, 'cy': cy, 'conf': conf}
                
                detections = list(detections_dict.values())
                
                # 寻找目标数字
                for det in detections:
                    if det['num'] == self.target_number:
                        target_cx, target_cy = det['cx'], det['cy']
                        break
                
                # 动态自动标定 (依然保留，用于在没有H矩阵时计算比例)
                new_ratio = self.calculate_ratio(detections)
                if new_ratio is not None:
                    self.pixel_to_meter_ratio = new_ratio
                    rospy.loginfo_throttle(1, f"Auto-calibrated Ratio: {self.pixel_to_meter_ratio:.6f} m/pixel")
                
                # 检查目标是否找到
                # 如果有单应性矩阵 H，就不需要 pixel_to_meter_ratio 也能直接抓！
                can_grasp = (self.H is not None) or (self.pixel_to_meter_ratio is not None)
                
                if target_cx is not None and can_grasp:
                    h_img, w_img, _ = frame.shape
                    
                    rospy.loginfo(f"Target '{self.target_number}' found at ({target_cx:.1f}, {target_cy:.1f}). Initiating grasp...")
                    
                    # 明确在画面上显示已锁定，暂停识别
                    cv2.putText(annotated_frame, "TARGET LOCKED - VISION PAUSED", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    cv2.imshow("YOLO Vision", annotated_frame)
                    cv2.waitKey(1)
                    
                    self.execute_grasp(target_cx, target_cy, w_img, h_img)
                
                    # 抓取完成后清空摄像头缓冲，等待下一个指令循环
                    for _ in range(10):
                        self.cap.grab()
                    rospy.sleep(0.5)
                    self.target_number = None
                    self.state = "WAITING"
            else:
                    if target_cx is None:
                        rospy.loginfo_throttle(2, f"Searching for target {self.target_number}...")
                    else:
                        rospy.loginfo_throttle(2, "Target found, but need at least 2 adjacent numbers to auto-calibrate first!")
                        
            rate.sleep()
            
        # 退出时释放摄像头资源和关闭窗口
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()

# 全局变量以供信号处理器使用
skill_node = None

def cleanup_handler(sig, frame):
    """处理 SIGTERM 信号 (网页端停止时发送) 和 SIGINT 信号 (Ctrl+C)"""
    rospy.loginfo("Received termination signal. Cleaning up...")
    if skill_node is not None and hasattr(skill_node, 'cap') and skill_node.cap.isOpened():
        skill_node.cap.release()
    cv2.destroyAllWindows()
    sys.exit(0)

if __name__ == "__main__":
    # 注册系统信号，确保不论是被键盘 Ctrl+C 还是被网页端强制杀死，都能关闭视觉窗口
    signal.signal(signal.SIGINT, cleanup_handler)
    signal.signal(signal.SIGTERM, cleanup_handler)
    
    try:
        skill_node = NumberGraspSkill()
    except rospy.ROSInterruptException:
        pass 
    finally:
        cleanup_handler(None, None)
