#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import subprocess
import os
import rospkg
import signal
import json
import threading
import csv
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from std_msgs.msg import Bool, Float32, String
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
            "point": None,
            "record": None,
            "playback": None
        }
        self.skill_active = False  # Pause control when skill process takes over
        self.teach_playback_active = False
        self.teach_playback_thread = None
        
        # Commands to launch each skill
        rospack = rospkg.RosPack()
        self.commands = {
            "grasp": ["roslaunch", "visual_grasping", "grasp.launch"],
            "follow": ["roslaunch", "arm_follow", "arm_follow.launch"],
            "point": ["python3", os.path.join(rospack.get_path('openclaw_sawyer'), 'scripts', 'point_grasp_skill.py')]
        }
        
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
        
        # Camera control - cameras OFF by default, controlled via web buttons
        self.cameras = None
        try:
            self.cameras = intera_interface.Cameras()
            # Explicitly stop all cameras (Sawyer may have them on by default)
            for cam in ["head_camera", "right_hand_camera"]:
                try:
                    if self.cameras.verify_camera_exists(cam):
                        self.cameras.stop_streaming(cam)
                except:
                    pass
            rospy.loginfo("Camera interface ready (all cameras stopped).")
        except Exception as e:
            rospy.logwarn("Camera initialization error: %s", e)
        
        # Head lights control - turn ON by default when package runs
        self.lights = None
        self.light_colors = [
            (True, False, False),  # Red
            (False, True, False),  # Green
            (False, False, True),  # Blue
            (True, True, False),   # Yellow
            (True, False, True),   # Magenta/Purple
            (False, True, True),   # Cyan
            (True, True, True)     # White
        ]
        self.current_color_idx = 0
        try:
            self.lights = intera_interface.Lights()
            # Start 1Hz timer to cycle colors sequentially
            rospy.Timer(rospy.Duration(1.0), self.cycle_head_lights)
            rospy.loginfo("Successfully initialized Sawyer Head Lights color cycling.")
        except Exception as e:
            rospy.logwarn("Could not connect to Sawyer Lights: %s", e)
        
        # Virtual spring-damper parameters (tuned for smooth web joystick/slider control)
        # omega = natural frequency (responsiveness), zeta = damping ratio (1.0=critical, >1=overdamped)
        # J0 (base): overdamped to prevent oscillation of the heavy base
        # J1-J6: critically damped for smooth, fast response without overshoot
        self.spring_params = {
            "right_j0": {"omega": 6.0, "zeta": 1.5, "max_delta": 0.5, "snap": 0.02},
            "right_j1": {"omega": 8.0, "zeta": 1.0, "max_delta": 0.6, "snap": 0.02},
            "right_j2": {"omega": 8.0, "zeta": 1.0, "max_delta": 0.6, "snap": 0.02},
            "right_j3": {"omega": 8.0, "zeta": 1.0, "max_delta": 0.6, "snap": 0.02},
            "right_j4": {"omega": 8.0, "zeta": 1.0, "max_delta": 0.6, "snap": 0.02},
            "right_j5": {"omega": 8.0, "zeta": 1.0, "max_delta": 0.6, "snap": 0.02},
            "right_j6": {"omega": 8.0, "zeta": 1.0, "max_delta": 0.6, "snap": 0.02},
        }
        self.joint_velocity = {}   # velocity state for spring-damper
        self.last_command = {}     # last commanded position
        self.last_update_time = rospy.Time.now()

        # Joint limits (from visual.cpp)
        self.joint_limits = {
            "right_j0": (-3.05, 3.05),
            "right_j1": (-1.2, 1.4),
            "right_j2": (-1.5, 1.5),
            "right_j3": (-1.5, 1.5),
            "right_j4": (-2.0, 2.0),
            "right_j5": (-2.0, 1.7),
            "right_j6": (-3.0, 3.0),
        }

        # Interpolation state
        self.target_joints = {}
        self.current_joints = {}
        self.target_head_pan = 0.0
        self.current_head_pan = 0.0
        self.head_controlled = False  # Only command head after web sets it
        self.head_velocity = 0.0     # velocity for head spring-damper
        
        # Services
        rospy.Service("/sawyer_grasp", SetBool, self.handle_grasp_toggle)
        rospy.Service("/sawyer_follow", SetBool, self.handle_follow_toggle)
        rospy.Service("/sawyer_point", SetBool, self.handle_point_toggle)
        rospy.Service("/sawyer_trigger", Trigger, self.handle_trigger_grasp)
        rospy.Service("/sawyer_stop", Trigger, self.handle_stop)
        rospy.Service("/sawyer_head_cam", SetBool, self.handle_head_cam)
        rospy.Service("/sawyer_hand_cam", SetBool, self.handle_hand_cam)
        rospy.Service("/sawyer_reset", Trigger, self.handle_reset)
        
        # OpenClaw alias services
        rospy.Service("/sawyer_skill_server/set_grasp_mode", SetBool, self.handle_grasp_toggle)
        rospy.Service("/sawyer_skill_server/set_follow_mode", SetBool, self.handle_follow_toggle)
        
        # Topics
        self.joint_state_pub = rospy.Publisher("/sawyer_state", JointState, queue_size=1)
        self.joint_sub = rospy.Subscriber("/sawyer_joints", JointState, self.handle_joint_command)
        self.head_sub = rospy.Subscriber("/sawyer_head", JointState, self.handle_head_command)
        self.gripper_sub = rospy.Subscriber("/sawyer_gripper", Bool, self.handle_gripper_command)
        self.speed_sub = rospy.Subscriber("/sawyer_speed", Float32, self.handle_speed_command)

        # OpenClaw alias topics
        self.joint_sub_openclaw = rospy.Subscriber("/sawyer_skill_server/cmd_joints", JointState, self.handle_joint_command)
        self.gripper_sub_openclaw = rospy.Subscriber("/sawyer_skill_server/cmd_gripper", Bool, self.handle_gripper_command)

        # Teach & Replay command subscriber
        self.teach_sub = rospy.Subscriber("/sawyer_teach_cmd", String, self.handle_teach_cmd)

        # Timers - 100Hz control loop (matching visual.cpp)
        rospy.Timer(rospy.Duration(0.1), self.sync_state)
        rospy.Timer(rospy.Duration(0.01), self.control_loop)

        rospy.loginfo("Sawyer Skill Server with spring-damper control ready.")

    def sync_state(self, event):
        if self.limb is None or self.skill_active: return
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
        pass  # Speed is now controlled by spring-damper dynamics, not a simple factor

    def control_loop(self, event):
        # Pause when a skill process is running
        if self.skill_active:
            return

        # --- Joint spring-damper control ---
        if self.limb and self.target_joints:
            now = rospy.Time.now()
            dt = (now - self.last_update_time).to_sec()
            if dt < 0.001:
                dt = 0.001
            elif dt > 0.05:
                dt = 0.05
            self.last_update_time = now

            actual_angles = self.limb.joint_angles()
            cmd = JointCommand()
            cmd.mode = JointCommand.POSITION_MODE
            cmd.header.stamp = now

            joints_to_release = []
            for name in list(self.target_joints.keys()):
                target_pos = self.target_joints[name]
                params = self.spring_params.get(name)
                if params is None:
                    continue
                limits = self.joint_limits.get(name, (-3.14, 3.14))
                target_pos = max(limits[0], min(limits[1], target_pos))

                # Initialize last_command from actual position if not set
                if name not in self.last_command:
                    self.last_command[name] = actual_angles.get(name, target_pos)
                if name not in self.joint_velocity:
                    self.joint_velocity[name] = 0.0

                q_prev = self.last_command[name]
                q_ref = target_pos
                v = self.joint_velocity[name]
                omega = params["omega"]
                zeta = params["zeta"]
                snap = params["snap"]
                max_delta = params["max_delta"]

                # Spring-damper dynamics: a = omega^2 * e - 2*zeta*omega*v
                e = q_ref - q_prev
                a = omega * omega * e - 2.0 * zeta * omega * v
                v += a * dt
                q_new = q_prev + v * dt

                # Snap to target to avoid overshoot
                err_before = q_ref - q_prev
                err_after = q_ref - q_new
                if abs(err_before) < snap and abs(err_after) < snap and err_before * err_after <= 0.0:
                    q_new = q_ref
                    v = 0.0

                # Clamp delta per cycle
                delta = q_new - q_prev
                if abs(delta) > max_delta:
                    delta = max_delta if delta > 0 else -max_delta
                    q_new = q_prev + delta
                    v = delta / dt

                # Clamp to joint limits
                q_new = max(limits[0], min(limits[1], q_new))

                self.last_command[name] = q_new
                self.joint_velocity[name] = v
                self.current_joints[name] = q_new
                cmd.names.append(name)
                cmd.position.append(q_new)

                # Auto-release when converged
                actual_pos = actual_angles.get(name, q_new)
                if abs(actual_pos - target_pos) < 0.01 and abs(v) < 0.01:
                    joints_to_release.append(name)

            for name in joints_to_release:
                del self.target_joints[name]
                if name in self.joint_velocity:
                    self.joint_velocity[name] = 0.0

            if cmd.names:
                self.limb.set_joint_positions(dict(zip(cmd.names, cmd.position)))

        # --- Head spring-damper control ---
        if self.head and self.head_controlled:
            now = rospy.Time.now()
            dt = 0.01  # fixed dt for head
            actual_pan = self.head.pan()
            if abs(actual_pan - self.target_head_pan) < 0.01 and abs(self.head_velocity) < 0.01:
                self.head_controlled = False
                self.current_head_pan = actual_pan
                self.head_velocity = 0.0
            else:
                omega = 10.0
                zeta = 0.8
                e = self.target_head_pan - self.current_head_pan
                a = omega * omega * e - 2.0 * zeta * omega * self.head_velocity
                self.head_velocity += a * dt
                self.current_head_pan += self.head_velocity * dt
                self.head.set_pan(self.current_head_pan)

    def handle_joint_command(self, msg):
        if self.limb is None or self.skill_active: return
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.target_joints[name] = msg.position[i]

    def handle_head_command(self, msg):
        if self.head is None or self.skill_active: return
        if len(msg.position) > 0:
            self.target_head_pan = msg.position[0]
            self.head_controlled = True

    def handle_gripper_command(self, msg):
        if self.gripper is None or self.skill_active: return
        if msg.data: self.gripper.open()
        else: self.gripper.close()

    def handle_head_cam(self, req):
        """Toggle head camera streaming."""
        if self.skill_active:
            return SetBoolResponse(success=False, message="Skill is running, camera locked.")
        if self.cameras is None:
            return SetBoolResponse(success=False, message="Camera interface not available.")
        try:
            if req.data:
                self.cameras.start_streaming("head_camera")
                return SetBoolResponse(success=True, message="Head camera ON.")
            else:
                self.cameras.stop_streaming("head_camera")
                return SetBoolResponse(success=True, message="Head camera OFF.")
        except Exception as e:
            return SetBoolResponse(success=False, message=str(e))

    def handle_hand_cam(self, req):
        """Toggle right hand camera streaming."""
        if self.skill_active:
            return SetBoolResponse(success=False, message="Skill is running, camera locked.")
        if self.cameras is None:
            return SetBoolResponse(success=False, message="Camera interface not available.")
        try:
            if req.data:
                self.cameras.start_streaming("right_hand_camera")
                return SetBoolResponse(success=True, message="Hand camera ON.")
            else:
                self.cameras.stop_streaming("right_hand_camera")
                return SetBoolResponse(success=True, message="Hand camera OFF.")
        except Exception as e:
            return SetBoolResponse(success=False, message=str(e))

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

                # Exit Limb control mode to prevent SDK command timeout interference
                if self.limb:
                    try:
                        self.limb.exit_control_mode()
                    except Exception as e:
                        rospy.logwarn("Error exiting control mode: %s", e)

                # Stop cameras so the skill can manage them independently
                if self.cameras:
                    for cam in ["head_camera", "right_hand_camera"]:
                        try:
                            if self.cameras.verify_camera_exists(cam):
                                self.cameras.stop_streaming(cam)
                                rospy.loginfo("Stopped camera: %s", cam)
                        except Exception as e:
                            rospy.logwarn("Could not stop %s: %s", cam, e)

                self.processes[name] = subprocess.Popen(self.commands[name], preexec_fn=os.setsid)
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
                self.last_command[jname] = angle  # Reset spring-damper state
                self.joint_velocity[jname] = 0.0
            self.target_joints = {}
            self.last_update_time = rospy.Time.now()

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
        self.head_controlled = False
        self.head_velocity = 0.0
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
        self.head_controlled = False
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

    def handle_teach_cmd(self, msg):
        try:
            data = json.loads(msg.data)
            cmd = data.get("cmd")
            rospack = rospkg.RosPack()
            actions_dir = os.path.join(rospack.get_path('openclaw_sawyer'), 'actions')
            
            if not os.path.exists(actions_dir):
                os.makedirs(actions_dir)
            
            if cmd == "record_start":
                # Stop any active skill
                for p in self.processes:
                    if self.processes[p] is not None:
                        try: os.killpg(os.getpgid(self.processes[p].pid), signal.SIGTERM)
                        except: pass
                        self.processes[p] = None
                self.skill_active = True
                
                temp_file = os.path.join(actions_dir, "temp_recording.csv")
                self.processes["record"] = subprocess.Popen(["rosrun", "intera_examples", "joint_recorder.py", "-f", temp_file], preexec_fn=os.setsid)
                rospy.loginfo("Started recording to temp file.")
                
            elif cmd == "record_save":
                if self.processes.get("record") is not None:
                    try:
                        os.killpg(os.getpgid(self.processes["record"].pid), signal.SIGINT)
                        self.processes["record"].wait(timeout=5)
                    except: pass
                    self.processes["record"] = None
                    
                name = data.get("name", "action")
                temp_file = os.path.join(actions_dir, "temp_recording.csv")
                save_file = os.path.join(actions_dir, f"{name}.csv")
                if os.path.exists(temp_file):
                    os.rename(temp_file, save_file)
                
                self._resume_control()
                rospy.loginfo(f"Saved recording to {save_file}.")
                
            elif cmd == "record_cancel":
                if self.processes.get("record") is not None:
                    try: os.killpg(os.getpgid(self.processes["record"].pid), signal.SIGTERM)
                    except: pass
                    self.processes["record"] = None
                
                temp_file = os.path.join(actions_dir, "temp_recording.csv")
                if os.path.exists(temp_file):
                    os.remove(temp_file)
                
                self._resume_control()
                rospy.loginfo("Cancelled recording.")
                
            elif cmd == "action_delete":
                name = data.get("name", "")
                if name:
                    del_file = os.path.join(actions_dir, f"{name}.csv")
                    if os.path.exists(del_file):
                        os.remove(del_file)
                        rospy.loginfo(f"Deleted action file: {del_file}")
                
            elif cmd == "play_start":
                actions = data.get("actions", [])
                loop = data.get("loop", False)
                if not actions: return
                    
                for p in self.processes:
                    if self.processes[p] is not None:
                        try: os.killpg(os.getpgid(self.processes[p].pid), signal.SIGTERM)
                        except: pass
                        self.processes[p] = None
                        
                self.skill_active = True
                self.teach_playback_active = False
                if self.teach_playback_thread:
                    self.teach_playback_thread.join(timeout=2)
                    
                self.teach_playback_active = True
                self.teach_playback_thread = threading.Thread(target=self._playback_loop, args=(actions, loop, actions_dir))
                self.teach_playback_thread.daemon = True
                self.teach_playback_thread.start()
                rospy.loginfo(f"Started playback loop with actions: {actions}")
                
            elif cmd == "play_stop":
                self.teach_playback_active = False
                if self.processes.get("playback") is not None:
                    try: os.killpg(os.getpgid(self.processes["playback"].pid), signal.SIGTERM)
                    except: pass
                    self.processes["playback"] = None
                rospy.loginfo("Stopped playback.")
                
        except Exception as e:
            rospy.logerr(f"Error handling teach cmd: {e}")

    def _playback_loop(self, actions, loop, actions_dir):
        while self.teach_playback_active:
            for action in actions:
                if not self.teach_playback_active:
                    break
                file_path = os.path.join(actions_dir, f"{action}.csv")
                if not os.path.exists(file_path):
                    rospy.logwarn(f"Playback file not found: {file_path}")
                    continue
                
                rospy.loginfo(f"Playing back in-process: {action}")
                try:
                    with open(file_path, 'r') as f:
                        reader = csv.reader(f)
                        headers = next(reader)
                        joint_names = headers[1:]
                        start_time = None
                        
                        for row in reader:
                            if not self.teach_playback_active:
                                break
                            
                            t = float(row[0])
                            positions = [float(x) for x in row[1:]]
                            
                            if start_time is None:
                                start_pose = {}
                                for i, name in enumerate(joint_names):
                                    if name.startswith('right_j'):
                                        start_pose[name] = positions[i]
                                
                                # Check if already near start pose
                                current_angles = self.limb.joint_angles()
                                max_err = 0.0
                                for jname, jtarget in start_pose.items():
                                    err = abs(current_angles.get(jname, jtarget) - jtarget)
                                    if err > max_err: max_err = err
                                
                                if max_err > 0.08:
                                    rospy.loginfo(f"Moving to start pose (max err: {max_err:.3f})...")
                                    self.limb.move_to_joint_positions(start_pose, timeout=15.0)
                                    rospy.sleep(0.5)
                                
                                start_time = rospy.get_time()
                                first_t_csv = t
                                continue
                                
                            target_time = start_time + (t - first_t_csv)
                            
                            while rospy.get_time() < target_time and self.teach_playback_active:
                                rospy.sleep(0.002)
                                
                            if not self.teach_playback_active:
                                break
                                
                            limb_cmd = {}
                            for i, name in enumerate(joint_names):
                                if name.startswith('right_j'):
                                    limb_cmd[name] = positions[i]
                                elif 'gripper' in name.lower() and self.gripper:
                                    if positions[i] > 50.0 or positions[i] > 0.5: # SDK handles differently
                                        self.gripper.open()
                                    else:
                                        self.gripper.close()
                                        
                            if limb_cmd:
                                self.limb.set_joint_positions(limb_cmd)
                except Exception as e:
                    rospy.logerr(f"In-process playback error: {e}")
                    
            if not loop:
                break
        
        self.teach_playback_active = False
        self._resume_control()
        rospy.loginfo("Playback loop finished.")

    def handle_trigger_grasp(self, req):
        return TriggerResponse(success=True)

    def handle_reset(self, req):
        rospy.loginfo("RESET TRIGGERED: Displaying face on head screen, resetting head pan & joints slowly.")
        
        # 1. Start the image display script in the background
        try:
            rospy.loginfo("Running face image display on head screen...")
            subprocess.Popen(["rosrun", "intera_examples", "head_display_image.py", "-f", "/home/mycar/Image/face.png"])
        except Exception as e:
            rospy.logwarn("Failed to launch head display script: %s", e)

        # 2. Reset head pan to -1.5
        if self.head:
            try:
                rospy.loginfo("Resetting head pan to -1.5...")
                self.head.set_pan(-1.5)
            except Exception as e:
                rospy.logwarn("Failed to reset head pan: %s", e)

        # 3. Pause the active control loop and clear targets
        self.skill_active = True
        self.target_joints = {}

        # 4. Move the joints slowly to the target pose
        if self.limb:
            try:
                # Set speed to a very slow value for a continuous, smooth and slow movement
                self.limb.set_joint_position_speed(0.1)
                
                target_pose = {
                    "right_j0": 0.0,
                    "right_j1": 0.0,
                    "right_j2": 0.0,
                    "right_j3": -1.6,
                    "right_j4": 0.0,
                    "right_j5": 1.5,
                    "right_j6": 0.0
                }
                
                rospy.loginfo("Moving joints slowly to [0.0, 0.0, 0.0, -1.6, 0.0, 1.5, 0.0]...")
                self.limb.move_to_joint_positions(target_pose)
                
                # Restore default speed
                self.limb.set_joint_position_speed(0.3)
                rospy.loginfo("Reset joint movement complete.")
            except Exception as e:
                rospy.logerr("Error moving to reset pose: %s", e)

        # 5. Resume normal control and sync state
        self._resume_control()
        
        return TriggerResponse(success=True, message="Reset completed successfully.")

    def cycle_head_lights(self, event):
        if self.lights is None:
            return
        try:
            # Get the current color states (R, G, B)
            r, g, b = self.light_colors[self.current_color_idx]
            
            # Apply to head lights
            self.lights.set_light_state("head_red_light", r)
            self.lights.set_light_state("head_green_light", g)
            self.lights.set_light_state("head_blue_light", b)
            
            # Advance index to the next color in the cycle
            self.current_color_idx = (self.current_color_idx + 1) % len(self.light_colors)
        except Exception as e:
            pass



if __name__ == '__main__':
    try:
        server = SawyerSkillServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
