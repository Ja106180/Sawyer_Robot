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

DESCEND_DISTANCE = 0.170   # 抓取时向下伸长的距离 (0.170m = 17.0cm)
LIFT_DISTANCE = 0.10       # 抓取完成后抬高的距离 (0.10m = 10cm)

GRASP_OFFSET_X = 0.0    # 抓取时的 X 方向微调
GRASP_OFFSET_Y = 0.0    # 抓取时的 Y 方向微调

GRID_PHYSICAL_DIST = 0.07          # 相邻数字中心物理距离 7cm (0.07m)

class TransferGraspSkill:
    def __init__(self):
        rospy.init_node('transfer_grasp_skill')
        
        self.limb = intera_interface.Limb("right")
        self.gripper = intera_interface.Gripper("right")
        self.head = intera_interface.Head()
        
        # 状态机变量
        self.state = "INIT"
        self.pan_a_coords = {}          # {1: (cx, cy), 2: (cx, cy), ...} 记录 A 盘数字坐标
        self.debounce_frames = 0        # 防抖计数器
        self.missing_num_candidate = None # 当前疑似消失的数字
        self.locked_num = None          # 最终锁定要抓取/放置的数字
        self.locked_cx = None
        self.locked_cy = None
        self.pan_a_ee_x = None          # A 盘观察时的末端物理 X 坐标
        self.pan_a_ee_y = None          # A 盘观察时的末端物理 Y 坐标
        
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
            
        # 尝试加载单应性标定矩阵
        self.H = None
        try:
            calib_path = os.path.join(rospack.get_path('Position_Calibration'), 'config', 'calibration.yaml')
            import yaml
            with open(calib_path, 'r') as f:
                calib_data = yaml.load(f, Loader=yaml.UnsafeLoader)
                self.H = np.array(calib_data['homography_matrix'])
                rospy.loginfo(f"Successfully loaded Homography Matrix from {calib_path}")
        except Exception as e:
            rospy.logwarn(f"Could not load calibration.yaml: {e}. Will fallback to basic ratio math.")
            
        # 打开 USB 摄像头
        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        if not self.cap.isOpened():
            rospy.logerr(f"Cannot open USB camera with index {CAMERA_INDEX}! Try changing CAMERA_INDEX to 1.")
            return
            
        rospy.loginfo("Transfer Grasp Skill initialized. Starting control loop.")
        self.run()

    def perform_init_sequence(self):
        """返回 A 盘的初始观察位"""
        rospy.loginfo("Moving to A-Pan observation pose...")
        try:
            self.limb.set_joint_position_speed(0.1)
            
            # 保证头部归位
            self.head.set_pan(0.0)
            
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
            
            self.limb.set_joint_position_speed(0.2)
            self.gripper.open()
            
            # 清理摄像头旧缓存
            if hasattr(self, 'cap') and self.cap.isOpened():
                for _ in range(10):
                    self.cap.grab()
                    
            rospy.loginfo("Ready at A-Pan observation pose.")
            return True
        except Exception as e:
            rospy.logerr(f"Initialization failed: {e}")
            return False
            
    def perform_transfer_to_b(self):
        """移动到 B 盘的观察位"""
        rospy.loginfo("Transferring to B-Pan observation pose...")
        try:
            self.limb.set_joint_position_speed(0.1)
            current = self.limb.joint_angles()
            current['right_j0'] = -0.5
            current['right_j6'] = -1.8
            self.limb.move_to_joint_positions(current)
            rospy.sleep(3.0) # 等待3秒，让机械臂完全稳定，避免摄像头画面晃动导致误识别
            
            self.limb.set_joint_position_speed(0.2)
            
            # 清理摄像头旧缓存
            if hasattr(self, 'cap') and self.cap.isOpened():
                for _ in range(10):
                    self.cap.grab()
                    
            rospy.loginfo("Arrived at B-Pan observation pose.")
            return True
        except Exception as e:
            rospy.logerr(f"Transfer failed: {e}")
            return False

    def execute_grasp(self, target_cx, target_cy, w, h, is_place=False):
        action_name = "PLACE" if is_place else "GRASP"
        rospy.loginfo(f"Executing {action_name} at pixel ({target_cx:.1f}, {target_cy:.1f})")
        try:
            current_pose = self.limb.endpoint_pose()
            cx = current_pose['position'].x
            cy = current_pose['position'].y
            cz = current_pose['position'].z
            ori = current_pose['orientation']
            
            if self.H is not None:
                pixel_pt = np.array([target_cx, target_cy, 1.0])
                arm_pt = self.H @ pixel_pt
                
                # 这里算出来的是目标在 A 盘标定系下的绝对物理坐标
                target_x = (arm_pt[0] / arm_pt[2]) + GRASP_OFFSET_X
                target_y = (arm_pt[1] / arm_pt[2]) + GRASP_OFFSET_Y
                
                if not is_place:
                    # 如果是在 A 盘抓取，保存此时机械臂末端的真实绝对位置
                    self.pan_a_ee_x = cx
                    self.pan_a_ee_y = cy
                else:
                    # 如果是在 B 盘放置，H 矩阵算出的 target_x 是它以为还在 A 盘时的坐标！
                    # 所以我们需要算出目标相对于 A 盘末端的【偏移量向量】，然后加到当前的 B 盘末端坐标上！
                    if self.pan_a_ee_x is not None and self.pan_a_ee_y is not None:
                        offset_x = target_x - self.pan_a_ee_x
                        offset_y = target_y - self.pan_a_ee_y
                        target_x = cx + offset_x
                        target_y = cy + offset_y
                    else:
                        rospy.logwarn("Missing Pan A EE coordinates! Target might be inaccurate.")
            else:
                rospy.logwarn("Homography matrix missing! Cannot perform precision grasp.")
                return False
            
            rospy.loginfo(f"Final Target -> X: {target_x:.4f}, Y: {target_y:.4f}")
            
            observe_h = cz
            safe_grasp_h = cz - DESCEND_DISTANCE
            
            # 致命错误修复：抬起的高度必须回到初始的观察高度！
            # 否则因为抓取后没有回位，到了 B 盘时它的起始高度就已经比 A 盘低了，再下降就会撞到桌面！
            lift_h = observe_h
            
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
            
            rospy.loginfo(f"Move 1: Horizontal align...")
            self.limb.set_joint_position_speed(0.2)
            pose1 = create_pose(target_x, target_y, observe_h, ori)
            ik1 = self.limb.ik_request(pose1, "right_hand")
            if not ik1: return False
            self.limb.move_to_joint_positions(ik1)
            
            rospy.loginfo(f"Move 2: Descend...")
            self.limb.set_joint_position_speed(0.15)
            pose2 = create_pose(target_x, target_y, safe_grasp_h, ori)
            ik2 = self.limb.ik_request(pose2, "right_hand")
            if not ik2: return False
            self.limb.move_to_joint_positions(ik2)
            
            if is_place:
                rospy.loginfo("Action: Open gripper (PLACE)")
                self.gripper.open()
            else:
                rospy.loginfo("Action: Close gripper (GRASP)")
                self.gripper.close()
            rospy.sleep(0.3)
            
            rospy.loginfo(f"Move 3: Lift up...")
            self.limb.set_joint_position_speed(0.2)
            pose3 = create_pose(target_x, target_y, lift_h, ori)
            ik3 = self.limb.ik_request(pose3, "right_hand")
            if ik3:
                self.limb.move_to_joint_positions(ik3)
            else:
                self.limb.move_to_joint_positions(ik1)
                
            # 抬高后等待 2 秒，确保物理稳定
            rospy.loginfo("Waiting 2.0s for physical stabilization...")
            rospy.sleep(2.0)
                
            # 不再回归中心，因为马上要执行 Transfer
            return True
            
        except Exception as e:
            rospy.logerr(f"Action failed: {e}")
            return False

    def run(self):
        if not self.perform_init_sequence():
            return
            
        self.state = "OBSERVE_PAN_A"
        rate = rospy.Rate(10) # 10Hz 处理循环
        
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rate.sleep()
                continue
                
            results = self.model(frame, verbose=False)
            annotated_frame = results[0].plot()
            
            # Allow safe exit via window UI
            try:
                if cv2.getWindowProperty("YOLO Vision", cv2.WND_PROP_VISIBLE) < 1:
                    break
            except:
                pass

            # Parse YOLO detections
            detections_dict = {}
            for r in results:
                for box in r.boxes:
                    num = int(box.cls[0]) + 1
                    conf = float(box.conf[0])
                    x1, y1, x2, y2 = box.xyxy[0]
                    cx = float((x1 + x2) / 2)
                    cy = float((y1 + y2) / 2)
                    if num not in detections_dict or conf > detections_dict[num]['conf']:
                        detections_dict[num] = {'num': num, 'cx': cx, 'cy': cy, 'conf': conf}
            
            visible_nums = set(detections_dict.keys())
            
            # ===============================
            # STATE: OBSERVE_PAN_A (检测缺失)
            # ===============================
            if self.state == "OBSERVE_PAN_A":
                # 如果看到完整的 9 个数字，不断刷新记忆库，证明盘面没东西
                if len(visible_nums) == 9:
                    for num, data in detections_dict.items():
                        self.pan_a_coords[num] = (data['cx'], data['cy'])
                    self.debounce_frames = 0
                    self.missing_num_candidate = None
                    cv2.putText(annotated_frame, "PAN A: READY (9 nums)", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # 如果看到刚好 8 个数字，说明可能有东西放上去了 (遮挡了1个)
                elif len(visible_nums) == 8:
                    missing_set = set(range(1, 10)) - visible_nums
                    missing_num = list(missing_set)[0]
                    
                    if missing_num == self.missing_num_candidate:
                        self.debounce_frames += 1
                        cv2.putText(annotated_frame, f"MISSING {missing_num}: {self.debounce_frames}/10", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2)
                    else:
                        self.missing_num_candidate = missing_num
                        self.debounce_frames = 1
                        
                    # 稳定缺失超过 1 秒 (10 frames at 10Hz)
                    if self.debounce_frames >= 10:
                        if missing_num in self.pan_a_coords:
                            self.locked_num = missing_num
                            self.locked_cx, self.locked_cy = self.pan_a_coords[missing_num]
                            rospy.loginfo(f"TARGET LOCKED: {missing_num} missing. Proceeding to grasp.")
                            self.state = "GRASP_MISSING"
                        else:
                            rospy.logwarn("Missing number coords not in memory!")
                            self.debounce_frames = 0
                
                # 如果可见数字 <= 7，说明手在画面里，或者放了太大的东西，重置防抖
                else:
                    self.debounce_frames = 0
                    self.missing_num_candidate = None
                    cv2.putText(annotated_frame, f"OBSTRUCTED ({len(visible_nums)} nums)", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            # ===============================
            # STATE: OBSERVE_PAN_B (寻找目标放置)
            # ===============================
            elif self.state == "OBSERVE_PAN_B":
                cv2.putText(annotated_frame, f"PAN B: SEARCHING {self.locked_num}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                
                if self.locked_num in visible_nums:
                    self.locked_cx = detections_dict[self.locked_num]['cx']
                    self.locked_cy = detections_dict[self.locked_num]['cy']
                    rospy.loginfo(f"FOUND {self.locked_num} ON PAN B. Proceeding to place.")
                    self.state = "PLACE_TARGET"
            
            cv2.imshow("YOLO Vision", annotated_frame)
            cv2.waitKey(1)
            
            # ===============================
            # EXECUTION STATES (无阻塞 UI)
            # ===============================
            if self.state == "GRASP_MISSING":
                h_img, w_img, _ = frame.shape
                success = self.execute_grasp(self.locked_cx, self.locked_cy, w_img, h_img, is_place=False)
                if success:
                    self.state = "TRANSFER_TO_B"
                else:
                    self.perform_init_sequence()
                    self.state = "OBSERVE_PAN_A"
                    
            elif self.state == "TRANSFER_TO_B":
                self.perform_transfer_to_b()
                self.state = "OBSERVE_PAN_B"
                
            elif self.state == "PLACE_TARGET":
                h_img, w_img, _ = frame.shape
                success = self.execute_grasp(self.locked_cx, self.locked_cy, w_img, h_img, is_place=True)
                # 不管成败，都回归 A 盘重置
                self.perform_init_sequence()
                
                # 重置变量
                self.pan_a_coords.clear()
                self.locked_num = None
                self.missing_num_candidate = None
                self.debounce_frames = 0
                self.state = "OBSERVE_PAN_A"

            rate.sleep()
            
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()

# ==========================================
# 信号与入口点
# ==========================================
skill_node = None

def cleanup_handler(sig, frame):
    rospy.loginfo("Received termination signal. Cleaning up...")
    if skill_node is not None and hasattr(skill_node, 'cap') and skill_node.cap.isOpened():
        skill_node.cap.release()
    cv2.destroyAllWindows()
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, cleanup_handler)
    signal.signal(signal.SIGTERM, cleanup_handler)
    try:
        skill_node = TransferGraspSkill()
    except rospy.ROSInterruptException:
        pass 
    finally:
        cleanup_handler(None, None)
