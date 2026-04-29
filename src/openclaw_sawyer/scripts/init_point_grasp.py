#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import intera_interface
import sys

def main():
    rospy.init_node('init_point_grasp_pose')
    
    # 这里的参数可以根据您的实际桌面位置调整
    # 目标：手腕垂直向下，高度适中
    target_pose = {
        'right_j0': 1.5,
        'right_j1': -1.0,
        'right_j2': 0.0,
        'right_j3': 1.0,
        'right_j4': 0.0,
        'right_j5': 1.5,
        'right_j6': 0.0
    }
    
    try:
        limb = intera_interface.Limb("right")
        rospy.loginfo("正在初始化机械臂到观测位置...")
        limb.move_to_joint_positions(target_pose)
        rospy.loginfo("初始化完成。")
        return True
    except Exception as e:
        rospy.logerr(f"初始化失败: {e}")
        return False

if __name__ == '__main__':
    if main():
        sys.exit(0)
    else:
        sys.exit(1)
