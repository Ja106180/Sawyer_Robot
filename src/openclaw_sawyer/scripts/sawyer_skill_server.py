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
            "follow": None,
            "point": None
        }
        self.skill_active = False  # Pause control when skill process takes over
        
        # Paths to scripts
        rospack = rospkg.RosPack()
        self.paths = {
            "grasp": os.path.join(rospack.get_path('visual_grasping'), 'scripts', 'grasp.py'),
            "follow": os.path.join(rospack.get_path('arm_follow'), 'scripts', 'arm_follow_node.py'),
            "point": os.path.join(rospack.get_path('openclaw_sawyer'), 'scripts', 'point_grasp_skill.py')
        }
        
        # Speed scaling and smoothing
        self.speed_scale = 0.3
        self.smoothing_factor = 0.1  
        
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
        self.cameras = None
        try:
            self.cameras = intera_interface.Cameras()
            if self.cameras.verify_camera_exists("head_camera"):
                self.cameras.start_streaming("head_camera")
            if self.cameras.verify_camera_exists("right_hand_camera"):
                self.cameras.start_streaming("right_hand_camera")
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
        rospy.Service("/sawyer_point", SetBool, self.handle_point_toggle)
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
        rospy.Timer(rospy.Duration(0.02), self.control_loop) 

        rospy.loginfo("Sawyer Skill Server with Point-to-Grasp ready.")

    def sync_state(self, event):
        if self.limb is None: return
        state = JointState()
        state.header.stamp = rospy.Time.now()
        names = self.limb.joint_names()
        angles = [self.limb.joint_angle(name) for name in names]
        state.name = names
        state.position = angles
        self.joint_state_pub.publish(state)
        if not self.target_joints:
            for name, angle in zip(names, angles):
                self.current_joints[name] = angle

    def handle_speed_command(self, msg):
        self.speed_scale = max(0.01, min(1.0, msg.data))
        self.smoothing_factor = self.speed_scale * 0.3

    def control_loop(self, event):
        # Pause when a skill process is running to avoid fighting for control
        if self.skill_active:
            return
        if self.limb and self.target_joints:
            cmd_dict = {}
            for name, target_pos in self.target_joints.items():
                curr_pos = self.current_joints.get(name, target_pos)
                next_pos = curr_pos + self.smoothing_factor * (target_pos - curr_pos)
                cmd_dict[name] = next_pos
                self.current_joints[name] = next_pos
            if cmd_dict:
                self.limb.set_joint_positions(cmd_dict)

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
        return self._toggle_process("grasp", req.data)

    def handle_follow_toggle(self, req):
        return self._toggle_process("follow", req.data)

    def handle_point_toggle(self, req):
        return self._toggle_process("point", req.data)

    def _toggle_process(self, name, active):
        if active:
            if self.processes[name] is None or self.processes[name].poll() is not None:
                # Stop any other running skills first
                for other in self.processes:
                    if other != name and self.processes[other] is not None:
                        try:
                            os.killpg(os.getpgid(self.processes[other].pid), signal.SIGTERM)
                        except: pass
                        self.processes[other] = None
                        rospy.loginfo("Stopped conflicting skill: %s", other)

                # Pause control loop to avoid fighting with skill process
                self.skill_active = True
                self.target_joints = {}

                # Stop cameras so the skill can manage them independently
                try:
                    if self.cameras:
                        if self.cameras.verify_camera_exists("head_camera"):
                            self.cameras.stop_streaming("head_camera")
                        if self.cameras.verify_camera_exists("right_hand_camera"):
                            self.cameras.stop_streaming("right_hand_camera")
                        rospy.loginfo("Cameras released for skill: %s", name)
                except Exception as e:
                    rospy.logwarn("Error stopping cameras: %s", e)

                self.processes[name] = subprocess.Popen(["python3", self.paths[name]], preexec_fn=os.setsid)
                return SetBoolResponse(success=True, message=f"{name} started.")
        else:
            if self.processes[name] is not None:
                try:
                    os.killpg(os.getpgid(self.processes[name].pid), signal.SIGTERM)
                except:
                    pass
                self.processes[name] = None
                self._resume_control()
                return SetBoolResponse(success=True, message=f"{name} stopped.")
        return SetBoolResponse(success=True)

    def _resume_control(self):
        """Resume control after skill process ends."""
        # Don't resume if another skill is still running
        any_active = any(p is not None and p.poll() is None for p in self.processes.values())
        if any_active:
            rospy.loginfo("Other skill still running, staying paused.")
            return

        # Sync current joint positions to avoid jumping
        if self.limb:
            curr = self.limb.joint_angles()
            for jname, angle in curr.items():
                self.current_joints[jname] = angle
            self.target_joints = {}

        # Sync head position
        if self.head:
            try:
                self.current_head_pan = self.head.pan()
                self.target_head_pan = self.current_head_pan
            except Exception as e:
                rospy.logwarn("Error syncing head: %s", e)

        # Restart cameras
        try:
            if self.cameras:
                if self.cameras.verify_camera_exists("head_camera"):
                    self.cameras.start_streaming("head_camera")
                if self.cameras.verify_camera_exists("right_hand_camera"):
                    self.cameras.start_streaming("right_hand_camera")
                rospy.loginfo("Cameras restarted.")
        except Exception as e:
            rospy.logwarn("Error restarting cameras: %s", e)

        self.skill_active = False
        rospy.loginfo("Control loop resumed.")

    def handle_stop(self, req):
        rospy.logwarn("EMERGENCY STOP TRIGGERED!")
        for name in self.processes:
            if self.processes[name] is not None:
                try:
                    os.killpg(os.getpgid(self.processes[name].pid), signal.SIGTERM)
                except: pass
                self.processes[name] = None
        self.target_joints = {}
        self.skill_active = False
        if self.limb:
            curr = self.limb.joint_angles()
            self.limb.set_joint_positions(curr)
            for name, angle in curr.items(): self.current_joints[name] = angle
        if self.head:
            try:
                self.current_head_pan = self.head.pan()
                self.target_head_pan = self.current_head_pan
            except: pass
        # Restart cameras
        try:
            if self.cameras:
                if self.cameras.verify_camera_exists("head_camera"):
                    self.cameras.start_streaming("head_camera")
                if self.cameras.verify_camera_exists("right_hand_camera"):
                    self.cameras.start_streaming("right_hand_camera")
        except Exception as e:
            rospy.logwarn("Error restarting cameras: %s", e)
        return TriggerResponse(success=True, message="Stopped.")

    def handle_trigger_grasp(self, req):
        return TriggerResponse(success=True)

if __name__ == '__main__':
    try:
        server = SawyerSkillServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
