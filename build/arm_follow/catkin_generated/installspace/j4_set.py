#!/usr/bin/env python3
"""
j4_set.py

Utility to rotate Sawyer right_j4 (forearm roll) to a target angle.
Usage:
    rosrun arm_follow j4_set.py --angle 0.5
"""

import argparse
import sys

import numpy as np
import rospy
import intera_interface

J4_MIN = -2.0
J4_MAX = 2.0


def parse_args():
    parser = argparse.ArgumentParser(description="Set Sawyer right_j4 angle.")
    parser.add_argument("--angle", type=float, required=True, help="Target angle in radians.")
    parser.add_argument("--speed", type=float, default=0.3, help="Joint move speed (0-1). Default: 0.3")
    parser.add_argument(
        "--timeout", type=float, default=10.0, help="Move timeout in seconds. Default: 10.0"
    )
    parser.add_argument(
        "--threshold", type=float, default=0.01, help="Move completion threshold in radians. Default: 0.01"
    )
    return parser.parse_args(rospy.myargv()[1:])


def main():
    rospy.init_node("j4_set", anonymous=False)
    args = parse_args()

    target = float(np.clip(args.angle, J4_MIN, J4_MAX))

    limb = intera_interface.Limb("right")
    limb.set_joint_position_speed(args.speed)

    current = limb.joint_angles()
    if "right_j4" not in current:
        rospy.logerr("Joint right_j4 not available.")
        sys.exit(1)

    goal = current.copy()
    goal["right_j4"] = target

    rospy.loginfo("Moving right_j4 to %.3f rad (clipped from %.3f)", target, args.angle)
    limb.move_to_joint_positions(goal, timeout=args.timeout, threshold=args.threshold)
    rospy.loginfo("right_j4 set successfully.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

