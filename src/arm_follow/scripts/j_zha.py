#!/usr/bin/env python3
"""
j_zha.py

Utility to control Sawyer right gripper (end effector) open/close.
Usage:
    rosrun arm_follow j_zha.py --open
    rosrun arm_follow j_zha.py --close
    rosrun arm_follow j_zha.py --position 0.5
    rosrun arm_follow j_zha.py --reboot
    rosrun arm_follow j_zha.py --calibrate
"""

import argparse
import sys
import time

import rospy
import intera_interface


def parse_args():
    parser = argparse.ArgumentParser(description="Control Sawyer right gripper open/close.")
    action_group = parser.add_mutually_exclusive_group(required=True)
    action_group.add_argument("--open", action="store_true", help="Open the gripper fully.")
    action_group.add_argument("--close", action="store_true", help="Close the gripper fully.")
    action_group.add_argument(
        "--position", type=float, help="Set gripper position (0.0=closed, 1.0=open). Range: [0.0, 1.0]"
    )
    action_group.add_argument("--reboot", action="store_true", help="Reboot/power cycle the gripper to clear errors.")
    action_group.add_argument("--calibrate", action="store_true", help="Calibrate the gripper (sets min/max travel).")
    
    parser.add_argument(
        "--speed", type=float, default=0.3, help="Gripper move speed (0-1). Default: 0.3"
    )
    parser.add_argument(
        "--timeout", type=float, default=5.0, help="Move timeout in seconds. Default: 5.0"
    )
    return parser.parse_args(rospy.myargv()[1:])


def main():
    rospy.init_node("j_zha", anonymous=False)
    args = parse_args()

    # Initialize the gripper interface without auto-calibrating immediately.
    # This prevents the constructor from crashing or blocking on error states
    # when the user wants to run a reboot command.
    try:
        gripper = intera_interface.Gripper("right_gripper", calibrate=False)
    except Exception as e:
        rospy.logerr("Failed to initialize gripper interface: %s", e)
        sys.exit(1)

    # Handle reboot request
    if args.reboot:
        rospy.loginfo("Rebooting/Power cycling the gripper to clear error states...")
        try:
            gripper.reboot()
            rospy.loginfo("Reboot command sent successfully. Please wait about 3-5 seconds for the gripper to restart.")
        except Exception as e:
            rospy.logerr("Failed to reboot gripper: %s", e)
            sys.exit(1)
        return

    # Handle calibrate request
    if args.calibrate:
        rospy.loginfo("Starting manual calibration of the right gripper...")
        rospy.loginfo("IMPORTANT: Please make sure the gripper fingers are completely free, not holding or touching any objects.")
        try:
            success = gripper.calibrate()
            if success:
                rospy.loginfo("Gripper calibration completed successfully.")
            else:
                rospy.logerr("Gripper calibration failed! Please verify that the fingers are free and the cable connection is secure.")
                sys.exit(1)
        except Exception as e:
            rospy.logerr("Failed to calibrate gripper: %s", e)
            sys.exit(1)
        return

    # For standard movement commands, verify the gripper is ready and calibrated.
    if not gripper.is_ready():
        rospy.logwarn("Gripper is not ready/calibrated. Attempting auto-calibration...")
        rospy.loginfo("IMPORTANT: Please ensure the gripper fingers have free space to open and close fully.")
        try:
            if not gripper.calibrate():
                rospy.logerr("Auto-calibration failed! The gripper cannot be controlled.")
                rospy.logerr("Try running: rosrun arm_follow j_zha.py --reboot")
                sys.exit(1)
            rospy.loginfo("Auto-calibration succeeded.")
        except Exception as e:
            rospy.logerr("Error during auto-calibration: %s", e)
            sys.exit(1)

    # Perform the requested action
    if args.open:
        rospy.loginfo("Opening right gripper...")
        gripper.open()
        rospy.loginfo("Right gripper opened successfully.")
    elif args.close:
        rospy.loginfo("Closing right gripper...")
        gripper.close()
        rospy.loginfo("Right gripper closed successfully.")
    elif args.position is not None:
        position = max(0.0, min(1.0, args.position))  # Clamp to [0.0, 1.0]
        rospy.loginfo("Setting right gripper position to %.2f (0.0=closed, 1.0=open)...", position)
        gripper.set_position(position)
        rospy.loginfo("Right gripper position set successfully.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass


