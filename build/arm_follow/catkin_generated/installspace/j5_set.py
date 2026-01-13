#!/usr/bin/env python3
"""
j5_set.py

Utility to rotate Sawyer right_j5 (wrist pitch) to a target angle.
Usage:
    rosrun arm_follow j5_set.py --angle 0.4
"""

import argparse
import sys

import numpy as np
import rospy
import intera_interface

J5_MIN = -2.0
J5_MAX = 1.7


def parse_args():
    parser = argparse.ArgumentParser(description="Set Sawyer right_j5 wrist pitch angle.")
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
    rospy.init_node("j5_set", anonymous=False)
    args = parse_args()

    target = float(np.clip(args.angle, J5_MIN, J5_MAX))

    limb = intera_interface.Limb("right")
    limb.set_joint_position_speed(args.speed)

    current = limb.joint_angles()
    if "right_j5" not in current:
        rospy.logerr("Joint right_j5 not available.")
        sys.exit(1)

    goal = current.copy()
    goal["right_j5"] = target

    rospy.loginfo("Moving right_j5 to %.3f rad (clipped from %.3f)", target, args.angle)
    limb.move_to_joint_positions(goal, timeout=args.timeout, threshold=args.threshold)
    rospy.loginfo("right_j5 set successfully.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

