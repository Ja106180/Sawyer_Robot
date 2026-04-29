#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import subprocess
import os
import rospkg
import signal
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from std_msgs.msg import Bool, Float32
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
        
        # Paths to scripts
        rospack = rospkg.RosPack()
        self.paths = {
            "grasp": os.path.join(rospack.get_path('visual_grasping'), 'scripts', 'grasp.py'),
            "follow": os.path.join(rospack.get_path('arm_follow'), 'scripts', 'arm_follow_node.py')
        }
        
        # Speed scaling and smoothing
        self.speed_scale = 0.3
        self.smoothing_factor = 0.1  # Initial smoothing
        
        # Sawyer interfaces
        self.limb = None
        self.gripper = None
        self.head = None
        
        try:
            self.limb = intera_interface.Limb("right")
            rospy.loginfo("Successfully connected to Sawyer Limb.")
        except Exception as e:
            rospy.logwarn("Could not connect to Sawyer Limb: %s", e)

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
        
        # Camera control
        try:
            self.cameras = intera_interface.Cameras()
            # Try to open both cameras
            if self.cameras.verify_camera_exists("head_camera"):
                self.cameras.start_streaming("head_camera")
                self.cameras.set_exposure("head_camera", -1) 
                self.cameras.set_gain("head_camera", -1)
                rospy.loginfo("Head camera initialized.")
            
            if self.cameras.verify_camera_exists("right_hand_camera"):
                # Note: Bandwidth might be an issue, but let's try
                self.cameras.start_streaming("right_hand_camera")
                self.cameras.set_exposure("right_hand_camera", -1)
                self.cameras.set_gain("right_hand_camera", -1)
                rospy.loginfo("Wrist camera initialized.")
        except Exception as e:
            rospy.logwarn("Camera initialization error: %s", e)

        # Interpolation state
        self.target_joints = {}
        self.current_joints = {}
        self.target_head_pan = 0.0
        self.current_head_pan = 0.0
        
        # Services
        rospy.Service("/sawyer_grasp", SetBool, self.handle_grasp_toggle)
        rospy.Service("/sawyer_follow", SetBool, self.handle_follow_toggle)
        rospy.Service("/sawyer_trigger", Trigger, self.handle_trigger_grasp)
        rospy.Service("/sawyer_stop", Trigger, self.handle_stop)
        
        # Topics
        self.joint_state_pub = rospy.Publisher("/sawyer_state", JointState, queue_size=1)
        self.joint_sub = rospy.Subscriber("/sawyer_joints", JointState, self.handle_joint_command)
        self.head_sub = rospy.Subscriber("/sawyer_head", JointState, self.handle_head_command)
        self.gripper_sub = rospy.Subscriber("/sawyer_gripper", Bool, self.handle_gripper_command)
        self.speed_sub = rospy.Subscriber("/sawyer_speed", Float32, self.handle_speed_command)

        # Timers
        rospy.Timer(rospy.Duration(0.1), self.sync_state)
        rospy.Timer(ros.Duration(0.02), self.control_loop) # 50Hz control loop

        rospy.loginfo("Sawyer Skill Server with speed control ready.")

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
        
        # Sync current state into interpolation if not already moving
        if not self.target_joints:
            for name, angle in zip(names, angles):
                self.current_joints[name] = angle

    def handle_speed_command(self, msg):
        self.speed_scale = max(0.01, min(1.0, msg.data))
        # Map speed_scale (0.05-1.0) to smoothing_factor (0.02-0.4)
        # Low speed = smaller smoothing factor = slower transition
        self.smoothing_factor = self.speed_scale * 0.3
        rospy.loginfo("Speed updated: %.2f (Smoothing: %.3f)", self.speed_scale, self.smoothing_factor)

    def control_loop(self, event):
        """Interpolation control loop."""
        # Arm control
        if self.limb and self.target_joints:
            cmd_dict = {}
            for name, target_pos in self.target_joints.items():
                curr_pos = self.current_joints.get(name, target_pos)
                next_pos = curr_pos + self.smoothing_factor * (target_pos - curr_pos)
                cmd_dict[name] = next_pos
                self.current_joints[name] = next_pos
            
            if cmd_dict:
                self.limb.set_joint_positions(cmd_dict)

        # Head control
        if self.head:
            next_pan = self.current_head_pan + self.smoothing_factor * (self.target_head_pan - self.current_head_pan)
            self.current_head_pan = next_pan
            self.head.set_pan(next_pan)

    def handle_joint_command(self, msg):
        if self.limb is None: return
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.target_joints[name] = msg.position[i]

    def handle_head_command(self, msg):
        if self.head is not None and len(msg.position) > 0:
            self.target_head_pan = msg.position[0]

    def handle_gripper_command(self, msg):
        if self.gripper is not None:
            if msg.data: self.gripper.open()
            else: self.gripper.close()

    def handle_grasp_toggle(self, req):
        if req.data:
            if self.processes["grasp"] is None or self.processes["grasp"].poll() is not None:
                self.processes["grasp"] = subprocess.Popen(["python3", self.paths["grasp"]], preexec_fn=os.setsid)
                return SetBoolResponse(success=True, message="Grasp started.")
        else:
            if self.processes["grasp"] is not None:
                os.killpg(os.getpgid(self.processes["grasp"].pid), signal.SIGTERM)
                self.processes["grasp"] = None
        return SetBoolResponse(success=True)

    def handle_follow_toggle(self, req):
        if req.data:
            if self.processes["follow"] is None or self.processes["follow"].poll() is not None:
                self.processes["follow"] = subprocess.Popen(["python3", self.paths["follow"]], preexec_fn=os.setsid)
                return SetBoolResponse(success=True, message="Follow started.")
        else:
            if self.processes["follow"] is not None:
                os.killpg(os.getpgid(self.processes["follow"].pid), signal.SIGTERM)
                self.processes["follow"] = None
        return SetBoolResponse(success=True)

    def handle_trigger_grasp(self, req):
        return TriggerResponse(success=True, message="Triggered.")

    def handle_stop(self, req):
        """Emergency Stop: Kill all sub-processes and freeze arm."""
        rospy.logwarn("EMERGENCY STOP TRIGGERED!")
        
        # 1. Kill all background processes
        for name in self.processes:
            if self.processes[name] is not None:
                try:
                    os.killpg(os.getpgid(self.processes[name].pid), signal.SIGTERM)
                    rospy.loginfo(f"Killed process: {name}")
                except Exception as e:
                    rospy.logerr(f"Error killing {name}: {e}")
                self.processes[name] = None
        
        # 2. Clear targets to stop interpolation
        self.target_joints = {}
        self.target_head_pan = self.current_head_pan
        
        # 3. Explicitly send current position to "freeze" the robot
        if self.limb:
            try:
                curr = self.limb.joint_angles()
                self.limb.set_joint_positions(curr)
                # Update our interpolation state too
                for name, angle in curr.items():
                    self.current_joints[name] = angle
            except:
                pass
                
        return TriggerResponse(success=True, message="EMERGENCY STOP EXECUTED. Processes killed, arm frozen.")

if __name__ == '__main__':
    try:
        server = SawyerSkillServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
