#!/usr/bin/env python3
"""
head_photo_node.py

头部图片显示节点：
1. 订阅 /vision_control/swipe_direction 话题
2. 根据滑动方向切换显示 photo 文件夹中的图片
3. 在机械臂头部屏幕显示图片
4. 订阅 /vision_control/vertical_move 话题
5. 使用虚拟弹簧控制机械臂j1关节（上下控制）
"""

import os
import cv2
import rospy
import glob
import numpy as np
import time
import intera_interface
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from intera_core_msgs.msg import JointCommand
from std_msgs.msg import Header

# 注意：直接发布到 /robot/head_display 话题即可，不需要 intera_interface


class HeadPhotoNode:
    def __init__(self):
        rospy.init_node("head_photo_node", anonymous=False)
        
        # 获取photo文件夹路径
        package_path = rospy.get_param("~photo_path", None)
        if package_path is None:
            # 默认路径：相对于脚本文件的photo文件夹
            script_dir = os.path.dirname(os.path.abspath(__file__))
            package_path = os.path.join(os.path.dirname(script_dir), "photo")
        
        self.photo_dir = package_path
        rospy.loginfo("Photo directory: %s", self.photo_dir)
        
        # 加载所有图片文件
        self.image_files = self._load_image_files()
        if not self.image_files:
            rospy.logerr("No image files found in %s", self.photo_dir)
            rospy.signal_shutdown("No image files found")
            return
        
        rospy.loginfo("Loaded %d image files", len(self.image_files))
        
        # 当前图片索引（默认第一张，索引0）
        self.current_index = 0
        
        # CvBridge
        self.bridge = CvBridge()
        
        # 图片发布者（直接发布到/robot/head_display话题）
        self._image_pub = rospy.Publisher('/robot/head_display', Image, latch=True, queue_size=10)
        
        # 订阅滑动方向话题
        self.swipe_sub = rospy.Subscriber("/vision_control/swipe_direction", String, 
                                         self.swipe_direction_callback, queue_size=1)
        
        # 订阅上下移动话题
        self.vertical_move_sub = rospy.Subscriber("/vision_control/vertical_move", Float32,
                                                  self.vertical_move_callback, queue_size=1)
        
        # 订阅面积变化话题（放大/缩小）
        self.area_move_sub = rospy.Subscriber("/vision_control/area_move", Float32,
                                             self.area_move_callback, queue_size=1)
        
        # 初始化机械臂j1关节控制
        self.init_j1_control()
        
        # 初始化机械臂j3关节控制（放大/缩小）
        self.init_j3_control()
        
        # 显示第一张图片
        self.display_current_image()
        
        rospy.loginfo("Head photo node started")
        rospy.loginfo("Current image: %s (index %d/%d)", 
                     os.path.basename(self.image_files[self.current_index]),
                     self.current_index + 1, len(self.image_files))
    
    def _load_image_files(self):
        """
        加载photo文件夹中的所有图片文件
        
        Returns:
            排序后的图片文件路径列表
        """
        if not os.path.exists(self.photo_dir):
            rospy.logwarn("Photo directory does not exist: %s", self.photo_dir)
            return []
        
        # 支持的图片格式
        image_extensions = ['*.jpg', '*.jpeg', '*.png', '*.bmp', '*.gif', '*.tiff', '*.tif']
        
        image_files = []
        for ext in image_extensions:
            # 搜索所有匹配的文件
            pattern = os.path.join(self.photo_dir, ext)
            image_files.extend(glob.glob(pattern))
            # 也搜索大写扩展名
            pattern = os.path.join(self.photo_dir, ext.upper())
            image_files.extend(glob.glob(pattern))
        
        # 去重并排序（自然排序，确保0.png在1.png之前）
        def natural_sort_key(filename):
            """自然排序键：如果是数字文件名，按数字排序；否则按字符串排序"""
            basename = os.path.splitext(os.path.basename(filename))[0]
            try:
                # 尝试转换为数字
                return (0, int(basename), filename)  # 数字文件名
            except ValueError:
                return (1, basename, filename)  # 非数字文件名
        
        image_files = sorted(list(set(image_files)), key=natural_sort_key)
        
        return image_files
    
    def _setup_image(self, image_path):
        """
        加载图片并转换为ROS Image消息
        
        Args:
            image_path: 图片文件路径
            
        Returns:
            sensor_msgs/Image 或 None
        """
        if not os.access(image_path, os.R_OK):
            rospy.logerr("Cannot read file at '%s'", image_path)
            return None
        
        try:
            img = cv2.imread(image_path)
            if img is None:
                rospy.logerr("Failed to load image: %s", image_path)
                return None
            
            # 转换为ROS Image消息
            image_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            return image_msg
        except Exception as e:
            rospy.logerr("Error loading image %s: %s", image_path, e)
            return None
    
    def display_current_image(self):
        """
        显示当前索引的图片
        """
        if not self.image_files:
            return
        
        image_path = self.image_files[self.current_index]
        image_msg = self._setup_image(image_path)
        
        if image_msg is None:
            rospy.logwarn("Failed to load image: %s", image_path)
            return
        
        # 发布图片到机械臂头部屏幕
        self._image_pub.publish(image_msg)
        
        rospy.loginfo("Display image: %s (index %d/%d)", 
                     os.path.basename(image_path),
                     self.current_index + 1, len(self.image_files))
    
    def swipe_direction_callback(self, msg):
        """
        滑动方向回调函数
        
        Args:
            msg: String消息，内容为 'left', 'right', 或 'None'
        """
        direction = msg.data
        
        if direction == 'left':
            # 往左切换：上一张图片（索引减1）
            self.current_index = (self.current_index - 1) % len(self.image_files)
            self.display_current_image()
            rospy.loginfo("Swipe left: switch to previous image (index %d/%d)", 
                         self.current_index + 1, len(self.image_files))
        
        elif direction == 'right':
            # 往右切换：下一张图片（索引加1）
            self.current_index = (self.current_index + 1) % len(self.image_files)
            self.display_current_image()
            rospy.loginfo("Swipe right: switch to next image (index %d/%d)", 
                         self.current_index + 1, len(self.image_files))
        
        # 'None' 时不切换，保持当前图片
        # 不需要处理
    
    def init_j1_control(self):
        """
        初始化j1关节控制（设置j1为1，归一化值）
        """
        try:
            self.limb = intera_interface.Limb("right")
            
            # j1关节范围（归一化值：0.0到1.0）
            # 映射到实际角度范围：j1实际范围是(-1.7, 0.5)弧度
            # 0.0 -> -1.7, 1.0 -> 0.5
            self.j1_normalized_min = 0.0
            self.j1_normalized_max = 1.0
            self.j1_actual_min = -1.7  # 实际最小角度（弧度）
            self.j1_actual_max = 0.5   # 实际最大角度（弧度）
            
            # 当前归一化值（初始化为1.0）
            self.current_j1_normalized = 1.0
            
            # 虚拟弹簧控制参数（参照arm_follow的C++实现）
            self.omega = 3.5  # 自然频率
            self.zeta = 0.95  # 阻尼系数
            self.dt = 0.01  # 控制循环周期（秒）
            self.j1_velocity = 0.0  # j1速度
            self.j1_snap_threshold = 0.05  # 快照阈值
            self.j1_max_delta = 0.6  # 每周期最大变化量（弧度/秒）
            
            # 发布者
            self.joint_cmd_pub = rospy.Publisher("/robot/limb/right/joint_command", JointCommand, queue_size=10)
            
            # 初始化j1为1（归一化值，映射到实际角度）
            target_normalized = 1.0
            target_actual = self.normalized_to_actual_j1(target_normalized)
            
            # 获取当前j1角度
            current_angles = self.limb.joint_angles()
            current_j1 = current_angles.get("right_j1", 0.0)
            
            # 移动到目标位置（初始化）
            rospy.loginfo("Initializing j1 to normalized value 1.0 (actual: %.3f rad, current: %.3f rad)", 
                         target_actual, current_j1)
            
            # 使用move_to_joint_positions初始化
            goal = current_angles.copy()
            goal["right_j1"] = target_actual
            self.limb.move_to_joint_positions(goal, timeout=5.0, threshold=0.01)
            
            # 初始化虚拟弹簧控制状态
            self.current_j1_actual = target_actual
            self.target_j1_actual = target_actual
            self.last_j1_command = target_actual
            
            # 启动控制循环
            self.control_timer = rospy.Timer(rospy.Duration(self.dt), self.j1_control_loop)
            
            rospy.loginfo("J1 control initialized: normalized=1.0, actual=%.3f rad", target_actual)
            
        except Exception as e:
            rospy.logerr("Failed to initialize j1 control: %s", e)
            self.limb = None
            self.joint_cmd_pub = None
    
    def normalized_to_actual_j1(self, normalized):
        """
        将归一化值（0.0-1.0）转换为实际j1角度（弧度）
        
        Args:
            normalized: 归一化值（0.0-1.0）
            
        Returns:
            实际角度（弧度）
        """
        normalized = np.clip(normalized, self.j1_normalized_min, self.j1_normalized_max)
        actual = self.j1_actual_min + normalized * (self.j1_actual_max - self.j1_actual_min)
        return actual
    
    def actual_to_normalized_j1(self, actual):
        """
        将实际j1角度（弧度）转换为归一化值（0.0-1.0）
        
        Args:
            actual: 实际角度（弧度）
            
        Returns:
            归一化值（0.0-1.0）
        """
        actual = np.clip(actual, self.j1_actual_min, self.j1_actual_max)
        normalized = (actual - self.j1_actual_min) / (self.j1_actual_max - self.j1_actual_min)
        return normalized
    
    def vertical_move_callback(self, msg):
        """
        上下移动回调函数
        
        Args:
            msg: Float32消息，正数=向上，负数=向下，0=没有移动
        """
        if self.limb is None or not hasattr(self, 'current_j1_normalized'):
            return
        
        move_value = msg.data
        
        if abs(move_value) < 0.001:
            # 没有移动，保持当前目标
            return
        
        # 数据范围约-1.0到1.0（已经缩小100倍）
        # vertical_displacement是相对于基准点的位移，缩小100倍后作为偏移量
        # 基准归一化值是1.0（初始位置）
        # 目标归一化值 = 基准1.0 - move_value（反向偏移）
        # 向上移动（正数）：j1增加，从1.0往上增加（但不超过1.0，所以被限制）
        # 向下移动（负数）：j1减少，从1.0往下减少（但不小于0.0）
        # move_value是缩小100倍的值（范围约0.01-1.0），直接作为偏移量（取反）
        
        base_normalized = 1.0  # 基准归一化值（初始位置）
        new_normalized = base_normalized - move_value  # 目标归一化值 = 基准 - 偏移（方向反转）
        
        # 限制在0.0到1.0范围内
        new_normalized = np.clip(new_normalized, self.j1_normalized_min, self.j1_normalized_max)
        
        # 更新目标值
        self.target_j1_actual = self.normalized_to_actual_j1(new_normalized)
        self.current_j1_normalized = new_normalized
        
        rospy.logdebug("Vertical move: value=%.3f, normalized=%.3f (base=1.0+offset), actual=%.3f", 
                      move_value, new_normalized, self.target_j1_actual)
    
    def j1_control_loop(self, event):
        """
        j1关节虚拟弹簧控制循环
        
        Args:
            event: Timer事件
        """
        if self.limb is None or not hasattr(self, 'target_j1_actual'):
            return
        
        try:
            # 获取当前j1角度
            current_angles = self.limb.joint_angles()
            q_prev = current_angles.get("right_j1", self.last_j1_command)
            
            # 目标角度
            q_ref = self.target_j1_actual
            
            # 虚拟弹簧-阻尼动力学（参照arm_follow的C++实现）
            e = q_ref - q_prev
            a = self.omega * self.omega * e - 2.0 * self.zeta * self.omega * self.j1_velocity
            self.j1_velocity += a * self.dt
            q_new = q_prev + self.j1_velocity * self.dt
            
            # 避免超调：当非常接近目标时，直接设为目标
            err_before = q_ref - q_prev
            err_after = q_ref - q_new
            if abs(err_before) < self.j1_snap_threshold and abs(err_after) < self.j1_snap_threshold and \
               err_before * err_after <= 0.0:
                q_new = q_ref
                self.j1_velocity = 0.0
            
            # 限制每周期最大变化量
            delta = q_new - q_prev
            max_delta_per_cycle = self.j1_max_delta * self.dt
            if abs(delta) > max_delta_per_cycle:
                delta = np.copysign(max_delta_per_cycle, delta)
                q_new = q_prev + delta
                self.j1_velocity = delta / self.dt
            
            # 限制在关节范围内
            q_new = np.clip(q_new, self.j1_actual_min, self.j1_actual_max)
            
            # 更新当前归一化值
            self.current_j1_normalized = self.actual_to_normalized_j1(q_new)
            
            # 发布关节命令（同时发布j1和j3，如果j3已初始化）
            cmd = JointCommand()
            cmd.header = Header()
            cmd.header.stamp = rospy.Time.now()
            
            if hasattr(self, 'last_j3_command'):
                # j3已初始化，同时发布j1和j3
                cmd.names = ["right_j1", "right_j3"]
                cmd.position = [q_new, self.last_j3_command]
            else:
                # j3未初始化，只发布j1
                cmd.names = ["right_j1"]
                cmd.position = [q_new]
            
            cmd.mode = JointCommand.POSITION_MODE
            self.joint_cmd_pub.publish(cmd)
            self.last_j1_command = q_new
            
        except Exception as e:
            rospy.logerr("Error in j1 control loop: %s", e)
    
    def init_j3_control(self):
        """
        初始化j3关节控制（设置j3为-1.5，实际角度）
        """
        try:
            if self.limb is None:
                rospy.logerr("Limb interface not available for j3 control")
                return
            
            # j3关节范围（实际角度：-1.5到-2.5弧度）
            # 归一化值：0.0到1.0
            # 0.0 -> -2.5（最小值），1.0 -> -1.5（最大值）
            self.j3_normalized_min = 0.0
            self.j3_normalized_max = 1.0
            self.j3_actual_min = -2.5  # 实际最小角度（弧度）
            self.j3_actual_max = -1.5  # 实际最大角度（弧度）
            
            # 当前归一化值（初始化为1.0，对应-1.5弧度）
            self.current_j3_normalized = 1.0
            
            # 虚拟弹簧控制参数（参照arm_follow的C++实现和j1控制）
            self.j3_omega = 4.5  # 自然频率（参照C++代码，j3用4.5）
            self.j3_zeta = 0.9   # 阻尼系数（参照C++代码，j3用0.9）
            self.j3_dt = 0.01    # 控制循环周期（秒）
            self.j3_velocity = 0.0  # j3速度
            self.j3_snap_threshold = 0.05  # 快照阈值
            self.j3_max_delta = 0.6  # 每周期最大变化量（弧度/秒）
            
            # 初始化j3为-1.5（实际角度，归一化值1.0）
            target_actual = -1.5  # 实际角度
            target_normalized = self.actual_to_normalized_j3(target_actual)
            
            # 获取当前j3角度
            current_angles = self.limb.joint_angles()
            current_j3 = current_angles.get("right_j3", 0.0)
            
            # 移动到目标位置（初始化）
            rospy.loginfo("Initializing j3 to -1.5 rad (normalized: %.3f, current: %.3f rad)", 
                         target_normalized, current_j3)
            
            # 使用move_to_joint_positions初始化
            goal = current_angles.copy()
            goal["right_j3"] = target_actual
            self.limb.move_to_joint_positions(goal, timeout=5.0, threshold=0.01)
            
            # 初始化虚拟弹簧控制状态
            self.current_j3_actual = target_actual
            self.target_j3_actual = target_actual
            self.last_j3_command = target_actual
            
            # 启动控制循环
            self.j3_control_timer = rospy.Timer(rospy.Duration(self.j3_dt), self.j3_control_loop)
            
            rospy.loginfo("J3 control initialized: normalized=1.0, actual=-1.5 rad")
            
        except Exception as e:
            rospy.logerr("Failed to initialize j3 control: %s", e)
            self.joint_cmd_pub = None
    
    def normalized_to_actual_j3(self, normalized):
        """
        将归一化值（0.0-1.0）转换为实际j3角度（弧度）
        
        Args:
            normalized: 归一化值（0.0-1.0）
            
        Returns:
            实际角度（弧度）
        """
        normalized = np.clip(normalized, self.j3_normalized_min, self.j3_normalized_max)
        actual = self.j3_actual_min + normalized * (self.j3_actual_max - self.j3_actual_min)
        return actual
    
    def actual_to_normalized_j3(self, actual):
        """
        将实际j3角度（弧度）转换为归一化值（0.0-1.0）
        
        Args:
            actual: 实际角度（弧度）
            
        Returns:
            归一化值（0.0-1.0）
        """
        actual = np.clip(actual, self.j3_actual_min, self.j3_actual_max)
        normalized = (actual - self.j3_actual_min) / (self.j3_actual_max - self.j3_actual_min)
        return normalized
    
    def area_move_callback(self, msg):
        """
        面积变化回调函数（放大/缩小）
        
        Args:
            msg: Float32消息，正数=放大，负数=缩小，0=没有移动
        """
        if self.limb is None or not hasattr(self, 'current_j3_normalized'):
            return
        
        move_value = msg.data
        
        if abs(move_value) < 0.001:
            # 没有移动，保持当前目标
            return
        
        # 数据范围约-1.0到1.0（已经缩小50倍）
        # area_displacement是相对于基准点的位移，缩小50倍后作为偏移量
        # 基准归一化值是1.0（初始位置，对应-1.5弧度）
        # 目标归一化值 = 基准1.0 - move_value（反向偏移）
        # 放大（正数）：j3往前（往-1.5方向，归一化值减少）
        # 缩小（负数）：j3往后（往-2.5方向，归一化值增加）
        # move_value是缩小50倍的值（范围约0.01-1.0），直接作为偏移量（取反）
        
        base_normalized = 1.0  # 基准归一化值（初始位置，对应-1.5弧度）
        new_normalized = base_normalized - move_value  # 目标归一化值 = 基准 - 偏移（方向反转）
        
        # 限制在0.0到1.0范围内
        new_normalized = np.clip(new_normalized, self.j3_normalized_min, self.j3_normalized_max)
        
        # 更新目标值
        self.target_j3_actual = self.normalized_to_actual_j3(new_normalized)
        self.current_j3_normalized = new_normalized
        
        rospy.logdebug("Area move: value=%.3f, normalized=%.3f (base=1.0-offset), actual=%.3f", 
                      move_value, new_normalized, self.target_j3_actual)
    
    def j3_control_loop(self, event):
        """
        j3关节虚拟弹簧控制循环
        
        Args:
            event: Timer事件
        """
        if self.limb is None or not hasattr(self, 'target_j3_actual') or self.joint_cmd_pub is None:
            return
        
        try:
            # 获取当前j3角度
            current_angles = self.limb.joint_angles()
            q_prev = current_angles.get("right_j3", self.last_j3_command)
            
            # 目标角度
            q_ref = self.target_j3_actual
            
            # 虚拟弹簧-阻尼动力学（参照arm_follow的C++实现）
            e = q_ref - q_prev
            a = self.j3_omega * self.j3_omega * e - 2.0 * self.j3_zeta * self.j3_omega * self.j3_velocity
            self.j3_velocity += a * self.j3_dt
            q_new = q_prev + self.j3_velocity * self.j3_dt
            
            # 避免超调：当非常接近目标时，直接设为目标
            err_before = q_ref - q_prev
            err_after = q_ref - q_new
            if abs(err_before) < self.j3_snap_threshold and abs(err_after) < self.j3_snap_threshold and \
               err_before * err_after <= 0.0:
                q_new = q_ref
                self.j3_velocity = 0.0
            
            # 限制每周期最大变化量
            delta = q_new - q_prev
            max_delta_per_cycle = self.j3_max_delta * self.j3_dt
            if abs(delta) > max_delta_per_cycle:
                delta = np.copysign(max_delta_per_cycle, delta)
                q_new = q_prev + delta
                self.j3_velocity = delta / self.j3_dt
            
            # 限制在关节范围内
            q_new = np.clip(q_new, self.j3_actual_min, self.j3_actual_max)
            
            # 更新当前归一化值
            self.current_j3_normalized = self.actual_to_normalized_j3(q_new)
            
            # 发布关节命令（同时发布j1和j3）
            cmd = JointCommand()
            cmd.header = Header()
            cmd.header.stamp = rospy.Time.now()
            cmd.names = ["right_j1", "right_j3"]
            cmd.position = [self.last_j1_command, q_new]
            cmd.mode = JointCommand.POSITION_MODE
            
            self.joint_cmd_pub.publish(cmd)
            self.last_j3_command = q_new
            
        except Exception as e:
            rospy.logerr("Error in j3 control loop: %s", e)
    
    def run(self):
        """主循环（保持节点运行）"""
        rospy.spin()


if __name__ == "__main__":
    try:
        node = HeadPhotoNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Node exception: %s", e)

