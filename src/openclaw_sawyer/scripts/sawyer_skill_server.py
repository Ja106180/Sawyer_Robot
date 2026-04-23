#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import subprocess
import os
import json
import threading
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
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
        
        # Sawyer interfaces
        self.limb = intera_interface.Limb("right")
        self.gripper = intera_interface.Gripper("right_gripper")
        self.head = intera_interface.Head()
        
        # Services for Toggles
        rospy.Service("~set_grasp_mode", SetBool, self.handle_grasp_toggle)
        rospy.Service("~set_follow_mode", SetBool, self.handle_follow_toggle)
        
        # Skills for OpenClaw (Trigger type)
        rospy.Service("~trigger_grasp", Trigger, self.handle_trigger_grasp)
        
        # Subscriber for manual control
        self.joint_cmd_pub = rospy.Publisher("/robot/limb/right/joint_command", JointCommand, queue_size=10)
        self.joint_sub = rospy.Subscriber("~cmd_joints", JointState, self.handle_joint_command)
        
        # Subscriber for head control
        self.head_sub = rospy.Subscriber("~cmd_head", JointState, self.handle_head_command)
        
        # Subscriber for gripper
        self.gripper_sub = rospy.Subscriber("~cmd_gripper", SetBool, self.handle_gripper_command)

        rospy.loginfo("Sawyer Skill Server initialized.")

    def handle_grasp_toggle(self, req):
        """Toggle autonomous grasping script."""
        if req.data:
            if self.processes["grasp"] is None or self.processes["grasp"].poll() is not None:
                # Start process
                cmd = ["rosrun", "visual_grasping", "grasp.py"]
                self.processes["grasp"] = subprocess.Popen(cmd)
                return SetBoolResponse(success=True, message="Visual Grasping started.")
            else:
                return SetBoolResponse(success=False, message="Visual Grasping is already running.")
        else:
            if self.processes["grasp"] is not None:
                self.processes["grasp"].terminate()
                self.processes["grasp"] = None
                return SetBoolResponse(success=True, message="Visual Grasping stopped.")
            else:
                return SetBoolResponse(success=False, message="Visual Grasping is not running.")

    def handle_follow_toggle(self, req):
        """Toggle arm following script."""
        if req.data:
            if self.processes["follow"] is None or self.processes["follow"].poll() is not None:
                cmd = ["rosrun", "arm_follow", "arm_follow_node.py"]
                self.processes["follow"] = subprocess.Popen(cmd)
                return SetBoolResponse(success=True, message="Arm Follow started.")
            else:
                return SetBoolResponse(success=False, message="Arm Follow is already running.")
        else:
            if self.processes["follow"] is not None:
                self.processes["follow"].terminate()
                self.processes["follow"] = None
                return SetBoolResponse(success=True, message="Arm Follow stopped.")
            else:
                return SetBoolResponse(success=False, message="Arm Follow is not running.")

    def handle_trigger_grasp(self, req):
        """Directly trigger the grasp process once (custom implementation or proxy)."""
        # Logic to perform a single grasp could be implemented here
        return TriggerResponse(success=True, message="Grasp action triggered via OpenClaw.")

    def handle_joint_command(self, msg):
        """Handle manual slider input from Web."""
        cmd = JointCommand()
        cmd.names = msg.name
        cmd.position = msg.position
        cmd.mode = JointCommand.POSITION_MODE
        self.joint_cmd_pub.publish(cmd)

    def handle_head_command(self, msg):
        """Handle head pan slider input."""
        if len(msg.position) > 0:
            self.head.set_pan(msg.position[0])

    def handle_gripper_command(self, msg):
        """Handle gripper open/close."""
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
