#!/usr/bin/env python3
"""
head_set.py

Quick utility to rotate Sawyer's head pan joint to a requested angle.
Usage:
    rosrun arm_follow head_set.py --angle 0.0
Optional arguments allow tweaking speed and timeout.
"""

import argparse
import sys

import rospy
import intera_interface
import numpy as np


HEAD_MIN = -1.57  # radians
HEAD_MAX = 1.57


def parse_args():
    parser = argparse.ArgumentParser(description="Set Sawyer head pan angle.")
    parser.add_argument(
        "--angle",
        type=float,
        required=True,
        help="Target pan angle in radians (-1.57 ~ 1.57).",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0.5,
        help="Motion speed (0-1). Default: 0.5",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="Move timeout in seconds. Default: 5.0",
    )
    return parser.parse_args(rospy.myargv()[1:])


def main():
    rospy.init_node("head_set", anonymous=False)
    args = parse_args()

    target = float(np.clip(args.angle, HEAD_MIN, HEAD_MAX))
    head = intera_interface.Head()

    rospy.loginfo(
        "Setting head pan to %.3f rad (clipped from %.3f)",
        target,
        args.angle,
    )
    success = head.set_pan(target, speed=args.speed, timeout=args.timeout)
    if not success:
        rospy.logerr("Failed to move head to requested angle.")
        sys.exit(1)

    rospy.loginfo("Head pan set successfully.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

