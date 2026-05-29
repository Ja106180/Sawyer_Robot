#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gripper_diagnose.py

A diagnostic tool to inspect the detailed status, parameters, and raw ROS topics
for the Sawyer Electric Gripper.
Usage:
    rosrun arm_follow gripper_diagnose.py
"""

import rospy
import sys
import json
import intera_interface
from std_msgs.msg import String
# Note: Raw I/O messages are usually published as JSON or custom messages
# We will subscribe to config/state topics to dump them.

def dump_msg(topic_name, msg_type):
    try:
        msg = rospy.wait_for_message(topic_name, msg_type, timeout=2.0)
        rospy.loginfo("--- Raw data from %s ---", topic_name)
        print(msg)
    except rospy.ROSException:
        rospy.logwarn("Timeout waiting for message on %s", topic_name)

def main():
    rospy.init_node("gripper_diagnose", anonymous=True)
    
    rospy.loginfo("==================================================")
    rospy.loginfo("         SAWYER GRIPPER HARDWARE DIAGNOSTICS      ")
    rospy.loginfo("==================================================")
    
    # 1. Initialize gripper interface without calibration
    rospy.loginfo("[1] Connecting to 'right_gripper' interface...")
    try:
        gripper = intera_interface.Gripper("right_gripper", calibrate=False)
        rospy.loginfo("Successfully connected to gripper interface.")
    except Exception as e:
        rospy.logerr("Failed to connect to gripper interface: %s", e)
        rospy.loginfo("Please verify that the robot is enabled and intera.sh is sourced correctly.")
        sys.exit(1)
        
    # 2. Print high-level status
    rospy.loginfo("[2] Reading high-level status:")
    try:
        print(f"  - Gripper Name:       {gripper.name}")
        print(f"  - Is Calibrated?      {gripper.is_calibrated()}")
        print(f"  - Current Position:   {gripper.get_position()} m")
        print(f"  - Current Force:      {gripper.get_force()} N")
    except Exception as e:
        rospy.logerr("Error reading status: %s", e)
        
    # 3. Dump all internal properties and dict values of the Gripper object
    rospy.loginfo("[3] Dumping internal SDK state dictionary:")
    try:
        for k, v in gripper.__dict__.items():
            # Skip big binary objects or functions to keep it clean
            if k.startswith('_') and not k.startswith('_state') and not k.startswith('_config'):
                continue
            print(f"  {k}: {v}")
    except Exception as e:
         rospy.logerr("Error dumping dictionary: %s", e)

    # 4. Subscribe to the raw I/O end effector config/state topics to see raw hardware logs
    rospy.loginfo("[4] Attempting to listen to raw I/O topics...")
    
    # Let's inspect the active topics first to find the exact namespace
    try:
        topics = [t[0] for t in rospy.get_published_topics()]
    except Exception as e:
        rospy.logwarn("Failed to get topic list: %s", e)
        topics = []
    
    io_topics = [t for t in topics if "end_effector" in t]
    rospy.loginfo("Found the following end_effector topics in ROS:")
    for t in io_topics:
        print(f"  - {t}")
        
    # We will try to dump a message from the config and state topics
    config_topic = next((t for t in io_topics if "config" in t), "/io/end_effector/config")
    state_topic = next((t for t in io_topics if "state" in t), "/io/end_effector/state")
    
    rospy.loginfo("[5] Fetching single message from config topic...")
    # Using rospy.msg.AnyMsg to print raw data
    from rospy.msg import AnyMsg
    dump_msg(config_topic, AnyMsg)
    
    rospy.loginfo("[6] Fetching single message from state topic...")
    dump_msg(state_topic, AnyMsg)
    
    rospy.loginfo("==================================================")
    rospy.loginfo("Diagnostics complete. Please copy the output above!")
    rospy.loginfo("==================================================")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
