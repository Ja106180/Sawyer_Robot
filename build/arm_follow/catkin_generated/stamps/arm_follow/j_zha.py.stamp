#!/usr/bin/env python3
"""
j_zha.py

Utility to control Sawyer right gripper (end effector) open/close.
Usage:
    rosrun arm_follow j_zha.py --open
    rosrun arm_follow j_zha.py --close
    rosrun arm_follow j_zha.py --position 0.5
"""

import argparse
import sys

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

    try:
        gripper = intera_interface.Gripper("right")
    except Exception as e:
        rospy.logerr("Failed to initialize gripper interface: %s", e)
        sys.exit(1)

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

