#!/usr/bin/env python3
"""
arm_follow_node.py

机械臂跟随节点：
1. 订阅YOLO Pose识别的关键点数据
2. 找到面积最大的人
3. 判断右手是否在50-150度范围内（从图像底部0度到顶部180度）
4. 平滑跟踪，避免突变和抖动
5. 控制Sawyer右臂的前4个关节（J0, J1, J2, J3）
"""

import rospy
import numpy as np
from arm_follow.msg import PersonsKeypoints, PersonKeypoints
from intera_core_msgs.msg import JointCommand
from std_msgs.msg import Header
import intera_interface


class ArmFollowNode:
    def __init__(self):
        rospy.init_node("arm_follow_node", anonymous=False)
        
        # 参数配置
        self.follow_min_angle = rospy.get_param("~follow_min_angle", 50.0)  # 跟随最小角度
        self.follow_max_angle = rospy.get_param("~follow_max_angle", 150.0)  # 跟随最大角度
        self.mutation_threshold = rospy.get_param("~mutation_threshold", 90.0)  # 突变阈值（度）
        self.smoothing_factor = rospy.get_param("~smoothing_factor", 0.7)  # 平滑系数（0-1，越大越平滑）
        self.confidence_threshold = rospy.get_param("~confidence_threshold", 0.5)  # 关键点置信度阈值
        
        # 关节角度映射参数（简单映射）
        # 图像角度范围 [50, 150] 映射到关节角度范围
        self.joint_angle_min = rospy.get_param("~joint_angle_min", -0.5)  # 关节最小角度（弧度）
        self.joint_angle_max = rospy.get_param("~joint_angle_max", 0.5)   # 关节最大角度（弧度）
        
        # 头部控制参数（手动控制模式）
        self.head_pan_min = rospy.get_param("~head_pan_min", -1.57)  # 头部最小角度（-90度，弧度）
        self.head_pan_max = rospy.get_param("~head_pan_max", 1.57)   # 头部最大角度（+90度，弧度）
        # 固定的头部角度（弧度），如果设置则移动头部到该角度并保持不动
        # 如果不设置，头部保持当前状态，程序不控制头部
        self.fixed_head_angle = rospy.get_param("~fixed_head_angle", None)
        
        # 状态变量
        self.current_right_hand_angle = None  # 当前右手角度
        self.smoothed_angle = None  # 平滑后的角度
        self.last_angle = None  # 上一时刻的角度
        self.last_joint_angles = None  # 上一次的关节角度（字典）
        self.is_following = False  # 是否正在跟随
        self.stable_count = 0  # 稳定计数
        self.stable_threshold = 5  # 需要稳定5帧才开始跟随
        
        # 头部控制状态变量
        self.head_interface = None
        
        # 初始化头部控制接口（仅当设置了固定角度时）
        if self.fixed_head_angle is not None:
            try:
                self.head_interface = intera_interface.Head()
                # 移动到固定角度并保持
                fixed_angle = float(self.fixed_head_angle)
                fixed_angle = np.clip(fixed_angle, self.head_pan_min, self.head_pan_max)
                self.head_interface.set_pan(fixed_angle, speed=0.5, timeout=5.0)
                rospy.loginfo("Head set to fixed angle: %.2f rad (%.1f degrees) and will stay fixed", 
                             fixed_angle, np.degrees(fixed_angle))
            except Exception as e:
                rospy.logerr("Failed to initialize head interface: %s", e)
        else:
            rospy.loginfo("Head control disabled - head will stay at current position (manual control)")
        
        # 机械臂接口：记录当前关节角度作为基准，避免主动带动底座和末端
        self.joint_names = ["right_j0", "right_j1", "right_j2", "right_j3"]
        try:
            self.limb = intera_interface.Limb("right")
            current_angles = self.limb.joint_angles()
        except Exception as exc:
            rospy.logerr("Failed to initialize limb interface: %s", exc)
            raise

        self.neutral_joint_angles = {
            name: current_angles.get(name, 0.0) for name in self.joint_names
        }
        self.last_joint_angles = self.neutral_joint_angles.copy()

        # 发布者
        self.joint_cmd_pub = rospy.Publisher("/robot/limb/right/joint_command", JointCommand, queue_size=10)
        
        # 订阅者
        self.keypoints_sub = rospy.Subscriber("/yolo_pose/keypoints", PersonsKeypoints, self.keypoints_callback)
        
        rospy.loginfo("Arm follow node initialized")
        rospy.loginfo(f"Follow range: {self.follow_min_angle} - {self.follow_max_angle} degrees")
        rospy.loginfo(f"Mutation threshold: {self.mutation_threshold} degrees")
        rospy.loginfo(f"Smoothing factor: {self.smoothing_factor}")
    
    def calculate_hand_angle(self, wrist_y, image_height):
        """
        计算右手在图像中的角度（从底部0度到顶部180度）
        
        Args:
            wrist_y: 右手腕的y坐标（图像坐标，顶部为0）
            image_height: 图像高度
            
        Returns:
            角度（0-180度），如果无效返回None
        """
        if wrist_y < 0 or wrist_y > image_height:
            return None
        
        # 从下到上：0度（底部）到180度（顶部）
        # wrist_y=0 对应顶部（180度），wrist_y=image_height 对应底部（0度）
        normalized_y = 1.0 - (wrist_y / image_height)  # 归一化到[0, 1]，底部为0，顶部为1
        angle = normalized_y * 180.0  # 转换为0-180度
        
        return angle
    
    def is_hand_extended(self, hand_angle):
        """
        判断右手是否在跟随范围内（50-150度）
        
        Args:
            hand_angle: 右手角度
            
        Returns:
            True if in range, False otherwise
        """
        if hand_angle is None:
            return False
        
        return self.follow_min_angle <= hand_angle <= self.follow_max_angle
    
    def find_largest_person(self, persons):
        """
        找到面积最大的人
        
        Args:
            persons: PersonKeypoints列表
            
        Returns:
            面积最大的PersonKeypoints，如果没有则返回None
        """
        if not persons or len(persons) == 0:
            return None
        
        largest_person = max(persons, key=lambda p: p.bbox_area)
        return largest_person
    
    def smooth_angle(self, new_angle):
        """
        平滑角度，避免抖动
        
        Args:
            new_angle: 新的角度值
            
        Returns:
            平滑后的角度
        """
        if new_angle is None:
            return self.smoothed_angle
        
        if self.smoothed_angle is None:
            self.smoothed_angle = new_angle
            return new_angle
        
        # 指数移动平均
        self.smoothed_angle = (self.smoothing_factor * self.smoothed_angle + 
                              (1 - self.smoothing_factor) * new_angle)
        
        return self.smoothed_angle
    
    def check_mutation(self, new_angle):
        """
        检查是否发生突变（角度变化超过阈值）
        
        Args:
            new_angle: 新的角度值
            
        Returns:
            True if mutation detected, False otherwise
        """
        if new_angle is None or self.last_angle is None:
            return False
        
        angle_diff = abs(new_angle - self.last_angle)
        return angle_diff > self.mutation_threshold
    
    def map_angle_to_joint(self, hand_angle):
        """
        将手部角度（50-150度）映射到关节角度
        
        Args:
            hand_angle: 手部角度（度）
            
        Returns:
            关节角度（字典）
        """
        # 将角度范围[50, 150]归一化到[0, 1]
        normalized = (hand_angle - self.follow_min_angle) / (self.follow_max_angle - self.follow_min_angle)
        normalized = np.clip(normalized, 0.0, 1.0)
        
        # 映射到关节角度范围
        joint_angle = self.joint_angle_min + normalized * (self.joint_angle_max - self.joint_angle_min)
        
        target_angles = self.neutral_joint_angles.copy()
        target_angles["right_j1"] = joint_angle
        
        # J2 做适度补偿，保持自然姿态
        j2_neutral = self.neutral_joint_angles.get("right_j2", 0.0)
        shoulder_offset = joint_angle - self.neutral_joint_angles["right_j1"]
        target_angles["right_j2"] = j2_neutral - shoulder_offset * 0.3
        
        # J0、J3 保持初始角度，避免底座和手腕乱动
        target_angles["right_j0"] = self.neutral_joint_angles["right_j0"]
        target_angles["right_j3"] = self.neutral_joint_angles["right_j3"]
        
        return target_angles
    
    def send_joint_command(self, joint_angles):
        """
        发送关节角度命令到机械臂
        
        Args:
            joint_angles: 关节角度字典
        """
        if joint_angles is None:
            return
        
        cmd = JointCommand()
        cmd.header = Header()
        cmd.header.stamp = rospy.Time.now()
        
        cmd.names = self.joint_names
        cmd.position = [joint_angles[name] for name in self.joint_names]
        cmd.mode = JointCommand.POSITION_MODE
        
        self.joint_cmd_pub.publish(cmd)
    
    def keypoints_callback(self, msg):
        """
        处理接收到的关键点数据
        
        Args:
            msg: PersonsKeypoints消息
        """
        # 找到面积最大的人
        largest_person = self.find_largest_person(msg.persons)
        
        if largest_person is None:
            rospy.logdebug("No person detected")
            # 目标丢失，保持当前位置
            if self.is_following and self.last_joint_angles:
                self.send_joint_command(self.last_joint_angles)
                self.is_following = False
            return
        
        # 头部控制已移除：头部保持手动设置的位置或当前状态，不会自动跟随
        
        # 检查右手关键点是否有效
        if largest_person.right_wrist_conf < self.confidence_threshold:
            rospy.logdebug("Right wrist confidence too low: %.2f", largest_person.right_wrist_conf)
            # 检测中断，保持当前位置
            if self.is_following and self.last_joint_angles:
                self.send_joint_command(self.last_joint_angles)
                self.is_following = False
            return
        
        # 计算右手角度
        hand_angle = self.calculate_hand_angle(
            largest_person.right_wrist_y,
            largest_person.image_height
        )
        
        if hand_angle is None:
            rospy.logdebug("Invalid hand angle")
            return
        
        # 检查是否伸出右手（在跟随范围内）
        if not self.is_hand_extended(hand_angle):
            rospy.logdebug("Hand not in follow range: %.1f degrees", hand_angle)
            # 不在跟随范围，保持当前位置
            if self.is_following and self.last_joint_angles:
                self.send_joint_command(self.last_joint_angles)
                self.is_following = False
            return
        
        # 检查是否发生突变
        if self.check_mutation(hand_angle):
            rospy.logwarn("Mutation detected: angle changed by %.1f degrees", 
                         abs(hand_angle - self.last_angle) if self.last_angle else 0)
            # 保持突变时的位置
            if self.is_following and self.last_joint_angles:
                self.send_joint_command(self.last_joint_angles)
            self.last_angle = hand_angle
            self.stable_count = 0
            return
        
        # 平滑角度
        smoothed_angle = self.smooth_angle(hand_angle)
        
        # 稳定计数
        if abs(smoothed_angle - hand_angle) < 5.0:  # 平滑后角度接近原始角度
            self.stable_count += 1
        else:
            self.stable_count = 0
        
        # 如果稳定了，开始跟随
        if self.stable_count >= self.stable_threshold:
            if not self.is_following:
                rospy.loginfo("Starting to follow hand at angle: %.1f degrees", smoothed_angle)
                self.is_following = True
            
            # 映射角度到关节角度
            joint_angles = self.map_angle_to_joint(smoothed_angle)
            
            # 发送关节命令
            self.send_joint_command(joint_angles)
            self.last_joint_angles = joint_angles.copy()
        else:
            # 未稳定，保持当前位置
            if self.is_following and self.last_joint_angles:
                self.send_joint_command(self.last_joint_angles)
        
        self.last_angle = hand_angle
        
        # 日志输出（降低频率）
        if rospy.get_time() % 2.0 < 0.1:  # 大约每2秒输出一次
            rospy.loginfo("Hand angle: %.1f deg, Smoothed: %.1f deg, Following: %s", 
                         hand_angle, smoothed_angle, self.is_following)
    
    def run(self):
        """运行节点"""
        rospy.loginfo("Arm follow node started")
        rospy.spin()


if __name__ == "__main__":
    try:
        node = ArmFollowNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

