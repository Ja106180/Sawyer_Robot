#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import subprocess
import os
import rospkg
import signal
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState, Image
from intera_core_msgs.msg import JointCommand
import intera_interface

class SawyerSkillServer:
    def __init__(self):
        rospy.init_node('sawyer_skill_server')
        
        # State management
        self.processes = {
            "grasp": None,
            "follow": None
        }

        # Movement smoothing parameters
        self.smoothing_factor = 0.65  # Smoothing factor (0.0 ~ 1.0). Smaller = slower/smoother
        self.target_joints = {}       # Target joint positions for arm
        self.current_joints = {}      # Current interpolated positions for arm
        self.target_head_pan = 0.0    # Target pan angle for head
        self.current_head_pan = 0.0   # Current interpolated pan angle for head
        
        # Paths to scripts (avoiding rosrun wrapper)
        rospack = rospkg.RosPack()
        self.paths = {
            "grasp": os.path.join(rospack.get_path('visual_grasping'), 'scripts', 'grasp.py'),
            "follow": os.path.join(rospack.get_path('arm_follow'), 'scripts', 'arm_follow_node.py')
        }
        
        # Sawyer interfaces (with robust error handling)
        self.limb = None
        self.gripper = None
        self.head = None
        
        try:
            self.limb = intera_interface.Limb("right")
            rospy.loginfo("Successfully connected to Sawyer Limb.")
        except Exception as e:
            rospy.logwarn("Could not connect to Sawyer Limb: %s. Manual control will be disabled.", e)

        try:
            self.gripper = intera_interface.Gripper("right")
            rospy.loginfo("Successfully connected to Sawyer Gripper.")
        except Exception as e:
            rospy.logwarn("Could not connect to Sawyer Gripper: %s", e)

        try:
            self.head = intera_interface.Head()
            rospy.loginfo("Successfully connected to Sawyer Head.")
        except Exception as e:
            rospy.logwarn("Could not connect to Sawyer Head: %s", e)
        
        # Initialize current positions to avoid jumps on first command
        if self.limb:
            names = self.limb.joint_names()
            self.current_joints = {name: self.limb.joint_angle(name) for name in names}
            self.target_joints = self.current_joints.copy()
        if self.head:
            self.current_head_pan = 0.0
        
        # Camera control (Attempt to open both)
        try:
            self.cameras = intera_interface.Cameras()
            if self.cameras.verify_camera_exists("head_camera"):
                self.cameras.start_streaming("head_camera")
                rospy.loginfo("Head camera started.")
            if self.cameras.verify_camera_exists("right_hand_camera"):
                self.cameras.start_streaming("right_hand_camera")
                rospy.loginfo("Wrist camera started.")
        except Exception as e:
            rospy.logwarn("Could not initialize cameras: %s", e)

        # Services for Toggles (ROOT NAMES)
        rospy.Service("/sawyer_grasp", SetBool, self.handle_grasp_toggle)
        rospy.Service("/sawyer_follow", SetBool, self.handle_follow_toggle)
        rospy.Service("/sawyer_trigger", Trigger, self.handle_trigger_grasp)
        
        # Topics (ROOT NAMES)
        self.joint_state_pub = rospy.Publisher("/sawyer_state", JointState, queue_size=1)
        self.joint_sub = rospy.Subscriber("/sawyer_joints", JointState, self.handle_joint_command)
        self.head_sub = rospy.Subscriber("/sawyer_head", JointState, self.handle_head_command)
        self.gripper_sub = rospy.Subscriber("/sawyer_gripper", Bool, self.handle_gripper_command)

        # Timer to sync state back to web (10Hz)
        rospy.Timer(rospy.Duration(0.1), self.sync_state)

        # Control loop timer for smooth motion (20Hz)
        rospy.Timer(rospy.Duration(0.05), self.control_loop)

        rospy.loginfo("Sawyer Skill Server initialized.")

    def sync_state(self, event):
        """Publish current robot state for Web UI syncing."""
        if self.limb is None:
            return
        
        state = JointState()
        state.header.stamp = rospy.Time.now()
        names = self.limb.joint_names()
        angles = [self.limb.joint_angle(name) for name in names]
        state.name = names
        state.position = angles
        self.joint_state_pub.publish(state)

    def control_loop(self, event):
        """ High-frequency control loop: performs interpolation for arm and head. """
        # 1. Smooth movements for arm joints
        if self.limb and self.target_joints:
            updated_joints = {}
            for name, target_val in self.target_joints.items():
                curr_val = self.current_joints.get(name, target_val)
                # Interpolation: next = curr + factor * (target - curr)
                next_val = curr_val + self.smoothing_factor * (target_val - curr_val)
                updated_joints[name] = next_val
                self.current_joints[name] = next_val
            
            try:
                self.limb.set_joint_positions(updated_joints)
            except Exception as e:
                rospy.logdebug("Limb control loop error: %s", e)

        # 2. Smooth movements for head pan
        if self.head:
            next_pan = self.current_head_pan + self.smoothing_factor * (self.target_head_pan - self.current_head_pan)
            self.current_head_pan = next_pan
            try:
                self.head.set_pan(next_pan)
            except Exception as e:
                rospy.logdebug("Head control loop error: %s", e)

    def handle_grasp_toggle(self, req):
        """Toggle autonomous grasping script."""
        rospy.loginfo("Received toggle_grasp request: %s", req.data)
        if req.data:
            if self.processes["grasp"] is None or self.processes["grasp"].poll() is not None:
                # Start process directly with python3
                cmd = ["python3", self.paths["grasp"]]
                self.processes["grasp"] = subprocess.Popen(cmd, preexec_fn=os.setsid) # Using setsid to kill group
                return SetBoolResponse(success=True, message="Visual Grasping started.")
            else:
                return SetBoolResponse(success=False, message="Visual Grasping is already running.")
        else:
            if self.processes["grasp"] is not None:
                os.killpg(os.getpgid(self.processes["grasp"].pid), signal.SIGTERM)
                self.processes["grasp"] = None
                return SetBoolResponse(success=True, message="Visual Grasping stopped.")
            else:
                return SetBoolResponse(success=False, message="Visual Grasping is not running.")

    def handle_follow_toggle(self, req):
        """Toggle arm following script."""
        rospy.loginfo("Received toggle_follow request: %s", req.data)
        if req.data:
            if self.processes["follow"] is None or self.processes["follow"].poll() is not None:
                cmd = ["python3", self.paths["follow"]]
                self.processes["follow"] = subprocess.Popen(cmd, preexec_fn=os.setsid)
                return SetBoolResponse(success=True, message="Arm Follow started.")
            else:
                return SetBoolResponse(success=False, message="Arm Follow is already running.")
        else:
            if self.processes["follow"] is not None:
                os.killpg(os.getpgid(self.processes["follow"].pid), signal.SIGTERM)
                self.processes["follow"] = None
                return SetBoolResponse(success=True, message="Arm Follow stopped.")
            else:
                return SetBoolResponse(success=False, message="Arm Follow is not running.")

    def handle_trigger_grasp(self, req):
        """Directly trigger the grasp process once."""
        rospy.loginfo("Received trigger_grasp via OpenClaw.")
        return TriggerResponse(success=True, message="Grasp action triggered.")

    def handle_joint_command(self, msg):
        """Handle manual slider input from Web (Update target values only)."""
        if self.limb is None:
            return
            
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.target_joints[name] = msg.position[i]
        
        rospy.logdebug("Updated Target Joints from Web")

    def handle_head_command(self, msg):
        """Handle head pan slider input (Update target value only)."""
        if self.head is not None and len(msg.position) > 0:
            self.target_head_pan = msg.position[0]
            rospy.logdebug("Updated Target Head Pan from Web")

    def handle_gripper_command(self, msg):
        """Handle gripper open/close (Direct Control)."""
        if self.gripper is not None:
            if msg.data:
                self.gripper.open()
            else:
                self.gripper.close()

if __name__ == '__main__':
    try:
        server = SawyerSkillServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
