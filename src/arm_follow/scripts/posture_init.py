#!/usr/bin/env python3
"""
posture_init.py

Set Sawyer to a predefined posture before arm-follow tracking starts.
Moves:
  - Base (right_j0)
  - Shoulder roll (right_j2)
  - Forearm roll (right_j4)
  - Wrist pitch (right_j5)
  - Wrist roll (right_j6)
  - Head pan

Once finished, sets /arm_follow/posture_ready = true so the C++ node
knows it can start controlling j1/j3.
"""

import argparse
import sys
from typing import Dict

import numpy as np
import rospy
import intera_interface


DEFAULTS = {
    "j0": 0.0,
    "j1": 0.0,
    "j2": 0.0,
    "j3": 0.0,
    "j4": 0.0,
    "j5": 0.0,
    "j6": 0.0,
    "head": -1.5,
}

LIMITS = {
    "j0": (-3.05, 3.05),
    "j1": (-1.2, 1.4),
    "j2": (-1.5, 1.5),
    "j3": (-1.5, 1.5),
    "j4": (-2.0, 2.0),
    "j5": (-2.0, 1.7),
    "j6": (-3.0, 3.0),
    "head": (-1.57, 1.57),
}


def parse_args():
    parser = argparse.ArgumentParser(description="Pre-position Sawyer joints before arm follow.")
    parser.add_argument("--j0", type=float, default=DEFAULTS["j0"], help="Target angle for right_j0 (rad).")
    parser.add_argument("--j1", type=float, default=DEFAULTS["j1"], help="Target angle for right_j1 (rad).")
    parser.add_argument("--j2", type=float, default=DEFAULTS["j2"], help="Target angle for right_j2 (rad).")
    parser.add_argument("--j3", type=float, default=DEFAULTS["j3"], help="Target angle for right_j3 (rad).")
    parser.add_argument("--j4", type=float, default=DEFAULTS["j4"], help="Target angle for right_j4 (rad).")
    parser.add_argument("--j5", type=float, default=DEFAULTS["j5"], help="Target angle for right_j5 (rad).")
    parser.add_argument("--j6", type=float, default=DEFAULTS["j6"], help="Target angle for right_j6 (rad).")
    parser.add_argument("--head", type=float, default=DEFAULTS["head"], help="Target head pan angle (rad).")
    parser.add_argument("--speed", type=float, default=0.3, help="Joint speed (0-1). Default: 0.3")
    parser.add_argument("--timeout", type=float, default=15.0, help="Move timeout seconds. Default: 15.")
    parser.add_argument("--threshold", type=float, default=0.01, help="Joint threshold radians. Default: 0.01")
    return parser.parse_args(rospy.myargv()[1:])


def clip(value: float, name: str) -> float:
    low, high = LIMITS[name]
    clipped = float(np.clip(value, low, high))
    if clipped != value:
        rospy.logwarn("%s clipped from %.3f to %.3f", name, value, clipped)
    return clipped


def move_limb(limb: intera_interface.Limb, targets: Dict[str, float], speed: float, timeout: float, threshold: float):
    limb.set_joint_position_speed(speed)
    current = limb.joint_angles()
    goal = current.copy()
    for joint, angle in targets.items():
        goal[joint] = angle

    limb.move_to_joint_positions(goal, timeout=timeout, threshold=threshold)


def main():
    rospy.init_node("posture_init", anonymous=False)
    rospy.set_param("/arm_follow/posture_ready", False)

    args = parse_args()

    limb = intera_interface.Limb("right")
    head = intera_interface.Head()

    targets = {
        "right_j0": clip(args.j0, "j0"),
        "right_j1": clip(args.j1, "j1"),
        "right_j2": clip(args.j2, "j2"),
        "right_j3": clip(args.j3, "j3"),
        "right_j4": clip(args.j4, "j4"),
        "right_j5": clip(args.j5, "j5"),
        "right_j6": clip(args.j6, "j6"),
    }

    rospy.loginfo("Setting base/arm joints to preset: %s", targets)
    move_limb(limb, targets, args.speed, args.timeout, args.threshold)

    head_angle = clip(args.head, "head")
    rospy.loginfo("Setting head pan to %.3f rad", head_angle)
    if not head.set_pan(head_angle, speed=min(0.5, args.speed), timeout=args.timeout):
        rospy.logwarn("Failed to set head pan angle.")

    rospy.set_param("/arm_follow/posture_ready", True)
    rospy.loginfo("Posture initialization complete.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

