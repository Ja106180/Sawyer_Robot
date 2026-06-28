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
from ultralytics import YOLO

# ==========================================
# 核心配置参数 (Configuration)
# ==========================================
CAMERA_INDEX = 0  # USB 摄像头设备号 (通常为 0 或 1，对应 /dev/video0)

# 高度配置 (相对于机械臂基座)
TABLE_Z = 0.75                     # 桌面高度 Z=0.75m
OBSERVE_HEIGHT = TABLE_Z + 0.09    # 观测高度 Z=0.84m (距离桌面 9cm)
SAFE_GRASP_HEIGHT = TABLE_Z + 0.05 # 抓取高度 Z=0.80m (距离桌面 5cm)

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
            
            # 步骤 3: J1 归正
            rospy.loginfo("Step 3: Joint 1 to -0.5")
            current['right_j1'] = -0.5
            self.limb.move_to_joint_positions(current)
            
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
            
            # 确保夹爪是松开的
            self.gripper.open()
            
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
            avg_ratio = sum(ratios) / len(ratios)
            return avg_ratio
        return None

    def execute_grasp(self, offset_x, offset_y):
        rospy.loginfo(f"Executing grasp with offset X: {offset_x:.4f}m, Y: {offset_y:.4f}m")
        try:
            current_pose = self.limb.endpoint_pose()
            cx = current_pose['position'].x
            cy = current_pose['position'].y
            ori = current_pose['orientation']
            
            target_x = cx + offset_x
            target_y = cy + offset_y
            
            # 移动步 1: 在 9cm 高度水平对准
            rospy.loginfo("Move 1: Horizontal align at OBSERVE_HEIGHT...")
            p1 = {'position': (target_x, target_y, OBSERVE_HEIGHT), 'orientation': ori}
            ik1 = self.limb.ik_request(p1)
            if not ik1:
                rospy.logerr("IK failed for horizontal align.")
                return
            self.limb.move_to_joint_positions(ik1)
            
            # 移动步 2: 垂直下降到 5cm
            rospy.loginfo("Move 2: Descend to SAFE_GRASP_HEIGHT...")
            p2 = {'position': (target_x, target_y, SAFE_GRASP_HEIGHT), 'orientation': ori}
            ik2 = self.limb.ik_request(p2)
            if not ik2:
                rospy.logerr("IK failed for descend.")
                return
            self.limb.move_to_joint_positions(ik2)
            
            # 动作 1: 闭合夹爪
            rospy.loginfo("Action: Close gripper")
            self.gripper.close()
            rospy.sleep(1.0)
            
            # 移动步 3: 抬起到 9cm
            rospy.loginfo("Move 3: Lift up to OBSERVE_HEIGHT...")
            self.limb.move_to_joint_positions(ik1)
            
            # 动作 2: 松开夹爪 (释放物体)
            rospy.loginfo("Action: Open gripper (Release)")
            self.gripper.open()
            rospy.sleep(1.0)
            
            rospy.loginfo("Grasp sequence complete. Returning to observation center.")
            # 可选：抓完后回到中心观察点
            self.perform_init_sequence()
            
        except Exception as e:
            rospy.logerr(f"Grasp execution failed: {e}")
            self.perform_init_sequence() # 发生异常则复位恢复安全姿态

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
                    
            elif self.state == "SEARCHING":
                # 推理图像
                results = self.model(frame, verbose=False)
                
                detections = []
                target_cx, target_cy = None, None
                
                # 解析 YOLO 结果
                for r in results:
                    boxes = r.boxes
                    for box in boxes:
                        cls_id = int(box.cls[0])
                        num = cls_id + 1 # YOLO index 0 是数字 1
                        
                        x1, y1, x2, y2 = box.xyxy[0]
                        cx = float((x1 + x2) / 2)
                        cy = float((y1 + y2) / 2)
                        
                        detections.append({'num': num, 'cx': cx, 'cy': cy})
                        
                        if num == self.target_number:
                            target_cx, target_cy = cx, cy
                
                # 执行动态自动标定
                new_ratio = self.calculate_ratio(detections)
                if new_ratio is not None:
                    self.pixel_to_meter_ratio = new_ratio
                    rospy.loginfo_throttle(1, f"Auto-calibrated Ratio: {self.pixel_to_meter_ratio:.6f} m/pixel")
                
                # 检查目标是否找到，并且比例是否已经标定
                if target_cx is not None and self.pixel_to_meter_ratio is not None:
                    h, w, _ = frame.shape
                    
                    # 像素偏移量 (相对画面中心)
                    du = target_cx - w / 2.0
                    dv = target_cy - h / 2.0
                    
                    # --- 坐标转换映射 (重点关注) ---
                    # 注意：正负号取决于摄像头在夹爪上的具体物理安装朝向。
                    # 假设：画面上方是机械臂正前方(+X)，画面右方是机械臂右方(-Y)
                    # 如果真机运行发现左右/前后反了，请修改此处的正负号。
                    offset_x = -dv * self.pixel_to_meter_ratio 
                    offset_y = -du * self.pixel_to_meter_ratio
                    
                    rospy.loginfo(f"Target '{self.target_number}' found at ({target_cx:.1f}, {target_cy:.1f}). Initiating grasp...")
                    
                    self.execute_grasp(offset_x, offset_y)
                    
                    # 抓取完成后重置状态，等待下一次指令
                    self.target_number = None
                    self.state = "WAITING"
                else:
                    if target_cx is None:
                        rospy.loginfo_throttle(2, f"Searching for target {self.target_number}...")
                    else:
                        rospy.loginfo_throttle(2, "Target found, but need at least 2 adjacent numbers to auto-calibrate first!")
                        
            rate.sleep()
            
        # 退出时释放摄像头资源
        self.cap.release()

if __name__ == "__main__":
    try:
        skill = NumberGraspSkill()
    except rospy.ROSInterruptException:
        pass
