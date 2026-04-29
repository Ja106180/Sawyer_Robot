#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import intera_interface
import subprocess
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import os
import yaml
import rospkg

# Attempt to import mediapipe
try:
    import mediapipe as mp
except ImportError:
    mp = None

class PointGraspSkill:
    def __init__(self):
        rospy.init_node('point_grasp_skill')
        
        # --- Configurable Parameters (User can tune these) ---
        self.OBSERVE_POSE = {
            'right_j0': 1.5,
            'right_j1': -1.0,
            'right_j2': 0.0,
            'right_j3': 1.0,
            'right_j4': 0.0,
            'right_j5': 1.5,
            'right_j6': 0.0
        }
        
        # Load calibration data from Position_Calibration package
        self.homography = None
        self.table_height = 0.746 # Default fallback
        try:
            rospack = rospkg.RosPack()
            config_path = os.path.join(rospack.get_path('Position_Calibration'), 'config', 'calibration.yaml')
            if os.path.exists(config_path):
                with open(config_path, 'r') as f:
                    data = yaml.safe_load(f)
                    self.homography = np.array(data['homography_matrix'])
                    self.table_height = data.get('table_height', 0.746)
                rospy.loginfo("Loaded Homography from: %s", config_path)
            else:
                rospy.logwarn("Calibration file NOT found at %s. Using default mapping.", config_path)
        except Exception as e:
            rospy.logwarn("Error loading calibration: %s", e)

        # Heights (Relative to robot BASE Z)
        self.GRASP_HEIGHT = self.table_height + 0.02  # Descend to 2cm above table
        self.LIFT_HEIGHT = self.table_height + 0.20  # Lift to 20cm above table
        
        self.HOVER_TIME = 2.0       # Seconds to hover for confirmation
        self.LIFT_WAIT_TIME = 2.0   # Seconds to hold in air
        
        # -----------------------------------------------------
        
        self.bridge = CvBridge()
        self.limb = intera_interface.Limb("right")
        self.gripper = intera_interface.Gripper("right")
        
        # MediaPipe Setup
        if mp:
            self.mp_hands = mp.solutions.hands
            self.hands = self.mp_hands.Hands(
                static_image_mode=False,
                max_num_hands=1,
                min_detection_confidence=0.7,
                min_tracking_confidence=0.5
            )

        self.latest_frame = None
        rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        
        self.state = "INIT"
        self.target_pixel = None
        self.hover_start_time = None
        
        rospy.loginfo("Point-to-Grasp Skill Ready (Continuous Loop Mode).")

    def image_callback(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def run_init_script(self):
        """Run the separate initialization script."""
        rospy.loginfo("Executing external initialization script...")
        rospack = rospkg.RosPack()
        init_script = os.path.join(rospack.get_path('openclaw_sawyer'), 'scripts', 'init_point_grasp.py')
        try:
            subprocess.run(["python3", init_script], check=True)
            return True
        except Exception as e:
            rospy.logerr(f"Init script failed: {e}")
            return False

    def pixel_to_base(self, u, v):
        """Map pixel coordinates to robot base coordinates using Homography."""
        if self.homography is not None:
            # Standard pixel to world using Homography
            # world_coords = H * [u, v, 1]^T
            pixel_vec = np.array([u, v, 1.0]).reshape(3, 1)
            world_vec = np.dot(self.homography, pixel_vec)
            world_vec /= world_vec[2] # Normalize by Z (homogeneous)
            return float(world_vec[0]), float(world_vec[1])
        else:
            # Fallback simple linear mapping (Needs manual tuning of scale_x/y)
            # This is a guestimate
            return 0.7, 0.0 

    def execute_grasp(self, x, y):
        rospy.loginfo("Executing Grasp sequence at: X=%.3f, Y=%.3f", x, y)
        
        # Get current orientation to maintain vertical down
        pose = self.limb.endpoint_pose()
        ori = pose['orientation']
        
        try:
            # 1. Pre-grasp
            rospy.loginfo("Step 1: Moving to Pre-grasp...")
            pre_grasp = {'position': (x, y, self.LIFT_HEIGHT), 'orientation': ori}
            self.limb.move_to_joint_positions(self.limb.ik_request(pre_grasp))
            
            # 2. Descend
            self.gripper.open()
            rospy.sleep(0.5)
            rospy.loginfo("Step 2: Descending...")
            descend = {'position': (x, y, self.GRASP_HEIGHT), 'orientation': ori}
            self.limb.move_to_joint_positions(self.limb.ik_request(descend))
            
            # 3. Grasp
            rospy.loginfo("Step 3: Grasping...")
            self.gripper.close()
            rospy.sleep(1.0)
            
            # 4. Lift (To 20cm above table)
            rospy.loginfo("Step 4: Lifting to 20cm...")
            lift = {'position': (x, y, self.LIFT_HEIGHT), 'orientation': ori}
            self.limb.move_to_joint_positions(self.limb.ik_request(lift))
            rospy.sleep(self.LIFT_WAIT_TIME)
            
            # 5. Release
            rospy.loginfo("Step 5: Releasing...")
            self.gripper.open()
            rospy.sleep(1.0)
            
            # 6. Return to observation
            rospy.loginfo("Step 6: Returning to Observe Pose.")
            self.run_init_script()
            
        except Exception as e:
            rospy.logerr(f"Grasp execution failed: {e}")

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.latest_frame is None:
                rate.sleep()
                continue
            
            frame = self.latest_frame.copy()
            h, w, _ = frame.shape

            if self.state == "INIT":
                if self.run_init_script():
                    self.state = "SEARCHING"

            elif self.state == "SEARCHING":
                if not mp:
                    rospy.logerr_once("MediaPipe missing. Detection aborted.")
                    rate.sleep()
                    continue

                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = self.hands.process(rgb_frame)
                
                found_point = False
                if results.multi_hand_landmarks:
                    for hand_lms in results.multi_hand_landmarks:
                        tip = hand_lms.landmark[8]
                        cx, cy = int(tip.x * w), int(tip.y * h)
                        
                        if self.target_pixel:
                            dist = np.sqrt((cx - self.target_pixel[0])**2 + (cy - self.target_pixel[1])**2)
                            if dist < 25: # Tolerance
                                if time.time() - self.hover_start_time > self.HOVER_TIME:
                                    self.state = "GRASPING"
                                    # Use Homography to get base coordinates
                                    tx, ty = self.pixel_to_base(cx, cy)
                                    self.grasp_target = (tx, ty)
                            else:
                                self.target_pixel = (cx, cy)
                                self.hover_start_time = time.time()
                        else:
                            self.target_pixel = (cx, cy)
                            self.hover_start_time = time.time()
                        found_point = True
                        break
                
                if not found_point:
                    self.target_pixel = None
                    self.hover_start_time = None

            elif self.state == "GRASPING":
                self.execute_grasp(self.grasp_target[0], self.grasp_target[1])
                self.state = "SEARCHING" # Continue the loop
                self.target_pixel = None
                self.hover_start_time = None

            rate.sleep()

if __name__ == "__main__":
    try:
        skill = PointGraspSkill()
        skill.run()
    except rospy.ROSInterruptException:
        pass
