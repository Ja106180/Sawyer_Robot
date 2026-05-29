#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import sys
import os
import csv
import rospy
import intera_interface
from intera_interface import CHECK_VERSION

def clean_line(line, names):
    def try_float(x):
        try: return float(x)
        except ValueError: return None
        
    line = [try_float(x) for x in line]
    combined = list(zip(names[1:], line[1:]))
    cleaned = [x for x in combined if x[1] is not None]
    command = dict(cleaned)
    right_command = {key: val for key, val in command.items() if key.startswith('right_')}
    return command, right_command, line

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--files', nargs='+', required=True, help='List of csv files to play')
    parser.add_argument('-l', '--loop', action='store_true', help='Loop the sequence infinitely')
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("seamless_playback", anonymous=True)
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    if not rs.state().enabled:
        rs.enable()

    limb = intera_interface.Limb('right')
    has_gripper = False
    try:
        gripper = intera_interface.Gripper('right_gripper')
        has_gripper = True
    except:
        pass

    rate = rospy.Rate(100)
    
    # Preload all files into memory
    sequences = []
    for f in args.files:
        if not os.path.exists(f):
            rospy.logwarn(f"File not found: {f}")
            continue
        with open(f, 'r') as csvfile:
            reader = csv.reader(csvfile)
            try:
                headers = next(reader)
                rows = list(reader)
                if rows:
                    sequences.append((f, headers, rows))
            except StopIteration:
                pass

    if not sequences:
        rospy.logerr("No valid files to play.")
        return

    def play_sequence():
        for filename, headers, rows in sequences:
            if rospy.is_shutdown(): return False
            rospy.loginfo(f"Playing back: {os.path.basename(filename)}")
            
            _cmd, cmd_start, _raw = clean_line(rows[0], headers)
            
            # Smart transition: Check if we are already at start position
            current_angles = limb.joint_angles()
            max_err = 0.0
            for jn, jt in cmd_start.items():
                err = abs(current_angles.get(jn, jt) - jt)
                if err > max_err: max_err = err
                
            if max_err > 0.08:
                rospy.loginfo(f"Moving to start position (err={max_err:.3f})...")
                limb.move_to_joint_positions(cmd_start)
                rospy.sleep(0.2)
            else:
                rospy.loginfo("Already at start pose, seamless transition.")

            start_time = rospy.get_time()
            for row in rows:
                if rospy.is_shutdown(): return False
                
                cmd, limb_cmd, values = clean_line(row, headers)
                target_time = values[0]
                
                while (rospy.get_time() - start_time) < target_time:
                    if rospy.is_shutdown(): return False
                    if limb_cmd:
                        limb.set_joint_positions(limb_cmd)
                    if has_gripper and gripper.name in cmd:
                        try:
                            gripper.set_position(cmd[gripper.name])
                        except: pass
                    rate.sleep()
        return True

    try:
        if args.loop:
            while not rospy.is_shutdown():
                if not play_sequence():
                    break
        else:
            play_sequence()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Playback finished.")

if __name__ == '__main__':
    main()
