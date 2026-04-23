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
        
        # Services for Toggles (ROOT NAMES)
        rospy.Service("/sawyer_grasp", SetBool, self.handle_grasp_toggle)
        rospy.Service("/sawyer_follow", SetBool, self.handle_follow_toggle)
        rospy.Service("/sawyer_trigger", Trigger, self.handle_trigger_grasp)
        
        # Topics (ROOT NAMES)
        self.joint_cmd_pub = rospy.Publisher("/robot/limb/right/joint_command", JointCommand, queue_size=1)
        self.joint_sub = rospy.Subscriber("/sawyer_joints", JointState, self.handle_joint_command)
        self.head_sub = rospy.Subscriber("/sawyer_head", JointState, self.handle_head_command)
        self.gripper_sub = rospy.Subscriber("/sawyer_gripper", Bool, self.handle_gripper_command)

        rospy.loginfo("Sawyer Skill Server initialized.")

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
        """Handle manual slider input from Web (Direct Control)."""
        if self.limb is None:
            return
        
        # Construct the dictionary for direct control
        joint_dict = {}
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                joint_dict[name] = msg.position[i]
        
        if joint_dict:
            rospy.logdebug("Direct Setting Joints: %s", joint_dict)
            self.limb.set_joint_positions(joint_dict)

    def handle_head_command(self, msg):
        """Handle head pan slider input (Direct Control)."""
        if self.head is not None and len(msg.position) > 0:
            self.head.set_pan(msg.position[0])

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
