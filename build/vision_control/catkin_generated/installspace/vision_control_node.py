#!/usr/bin/env python3
"""
vision_control_node.py

视觉控制节点：
1. 使用USB摄像头读取图像
2. MediaPipe人脸检测（显示检测框）
3. MediaPipe手势识别（滑动和放大/缩小）
4. 可视化界面（右上角UI：4个箭头+2个圆）
"""

import cv2
import rospy
import numpy as np
import mediapipe as mp
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Header, String, Float32
from collections import deque
import time
import threading

class VisionControlNode:
    def __init__(self):
        rospy.init_node("vision_control_node", anonymous=False)
        
        # 参数配置
        self.image_topic = rospy.get_param("~image_topic", "/camera/image_raw")  # 图像话题
        self.frame_width = rospy.get_param("~frame_width", 640)
        self.frame_height = rospy.get_param("~frame_height", 480)
        self.fps = rospy.get_param("~fps", 30.0)
        
        # 手势识别参数
        self.swipe_threshold = rospy.get_param("~swipe_threshold", 30.0)  # 滑动速度阈值（像素/帧）
        self.swipe_frames = rospy.get_param("~swipe_frames", 5)  # 连续帧数确认滑动
        self.zoom_threshold = rospy.get_param("~zoom_threshold", 0.15)  # 放大/缩小变化阈值
        self.zoom_speed_threshold = rospy.get_param("~zoom_speed_threshold", 0.05)  # 变化速度阈值
        
        # UI状态持续时间（秒）
        self.ui_highlight_duration = rospy.get_param("~ui_highlight_duration", 1.0)
        
        # CvBridge
        self.bridge = CvBridge()
        
        # 订阅图像话题
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)
        
        # 发布者（可选，如果需要发布处理后的图像）
        self.image_pub = rospy.Publisher("/vision_control/image", Image, queue_size=1)
        
        # 发布左右滑动方向
        self.swipe_direction_pub = rospy.Publisher("/vision_control/swipe_direction", String, queue_size=1)
        self.current_swipe_direction = None  # 当前滑动方向，用于跟踪状态变化
        
        # 发布上下移动位移（正数=向上，负数=向下，0=没有移动）
        self.vertical_move_pub = rospy.Publisher("/vision_control/vertical_move", Float32, queue_size=1)
        
        # 发布面积变化位移（正数=放大，负数=缩小，0=没有移动）
        self.area_move_pub = rospy.Publisher("/vision_control/area_move", Float32, queue_size=1)
        
        # MediaPipe初始化
        self.mp_face_detection = mp.solutions.face_detection
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        
        self.face_detection = self.mp_face_detection.FaceDetection(
            model_selection=0,  # 0=short-range, 1=full-range
            min_detection_confidence=0.75  # 提高阈值减少误检
        )
        
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,  # 允许检测两只手以便区分左右手
            min_detection_confidence=0.7,  # 提高阈值减少误检
            min_tracking_confidence=0.7  # 提高跟踪阈值
        )
        
        # 状态机：'idle'（待机）或 'control'（控制模式）
        self.control_state = 'idle'
        
        # 子模式：'horizontal'（左右模式）或 'vertical'（上下模式）
        self.sub_mode = 'horizontal'  # 默认左右模式
        
        # 激活/退出检测参数
        self.activation_hold_time = 1.0  # 激活手势保持时间（秒）
        self.deactivation_hold_time = 1.0  # 退出手势保持时间（秒）
        self.activation_start_time = None  # 开始检测激活手势的时间
        self.deactivation_start_time = None  # 开始检测退出手势的时间
        
        # 手势"1"检测参数（切换模式）
        self.one_finger_hold_time = 1.0  # 手势"1"保持时间（秒）
        self.one_finger_start_time = None  # 开始检测手势"1"的时间
        self.one_finger_confirmed = False  # 是否已经完成第一步（1个手指连续1秒）
        
        # 手势跟踪状态
        self.hand_history = deque(maxlen=10)  # 保存最近10帧的手部位置
        self.last_hand_center = None
        self.last_finger_spread = None
        self.current_finger_count = None  # 当前手指数量（用于显示）
        
        # 滑动检测状态（左右模式）
        self.base_hand_x = None  # 基准手掌中心X坐标（像素）
        self.last_swipe_time = None  # 上次滑动时间
        self.swipe_cooldown_duration = 1.0  # 滑动冷却时间（秒）
        self.swipe_threshold_pixel = 40  # 滑动阈值（像素）
        self.prep_threshold_pixel = 20  # 蓄力阈值（像素）
        
        # 上下移动检测状态（上下模式）
        # vertical_displacement: 显示用的位移（向上为正，向下为负），初始化为0
        self.vertical_displacement = 0
        # 基准点（0点）：用于计算位移值
        self.base_vertical_y = None  # 基准手掌中心Y坐标（像素）
        # 采样相关：每0.05秒采样一次手掌中心Y坐标，用于判断方向
        self.last_vertical_sample_time = None  # 上一次采样的时间
        self.last_vertical_y = None  # 上一次采样时手掌中心的Y坐标（像素）
        # 当前上下移动方向：1 = 向上，-1 = 向下，0 = 未确定（只用于判断方向）
        self.vertical_direction = 0
        
        # 手掌面积检测状态（放大/缩小）
        # area_displacement: 显示用的面积变化（增大为正，减小为负），初始化为0
        self.area_displacement = 0
        # 基准点（0点）：用于计算面积变化值
        self.base_area = None  # 基准手掌面积
        # 采样相关：每0.05秒采样一次手掌面积，用于判断方向
        self.last_area_sample_time = None  # 上一次采样的时间
        self.last_area = None  # 上一次采样时的手掌面积
        # 当前面积变化方向：1 = 增大，-1 = 减小，0 = 未确定（只用于判断方向）
        self.area_direction = 0
        
        # UI状态管理（4个箭头 + 2个圆）
        self.ui_states = {
            'arrow_left': {'active': False, 'time': 0},
            'arrow_right': {'active': False, 'time': 0},
            'arrow_up': {'active': False, 'time': 0},
            'arrow_down': {'active': False, 'time': 0},
            'circle_large': {'active': False, 'time': 0},  # 大圆（放大）
            'circle_small': {'active': False, 'time': 0}   # 小圆（缩小）
        }
        
        # 图像处理状态变量
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.image_received = False
        
        rospy.loginfo("Vision control node started")
        rospy.loginfo("Subscribing to image topic: %s", self.image_topic)
        
        # 设置循环频率
        self.rate = rospy.Rate(self.fps)
    
    def calculate_finger_spread(self, landmarks):
        """
        计算手指张开程度
        
        Args:
            landmarks: MediaPipe手部关键点
            
        Returns:
            手指张开程度（0-1之间，越大越张开）
        """
        # 使用指尖到手掌中心的距离
        # 手掌中心：手腕（0）和手掌底部（9）的中点
        wrist = landmarks.landmark[0]
        middle_mcp = landmarks.landmark[9]  # 中指根部
        
        palm_center_x = (wrist.x + middle_mcp.x) / 2
        palm_center_y = (wrist.y + middle_mcp.y) / 2
        
        # 计算5个指尖到手掌中心的距离
        finger_tips = [4, 8, 12, 16, 20]  # 拇指、食指、中指、无名指、小指
        distances = []
        
        for tip_idx in finger_tips:
            tip = landmarks.landmark[tip_idx]
            dist = np.sqrt((tip.x - palm_center_x)**2 + (tip.y - palm_center_y)**2)
            distances.append(dist)
        
        # 归一化到0-1范围（基于经验值）
        avg_distance = np.mean(distances)
        # 经验值：完全闭合约0.05，完全张开约0.15
        normalized = np.clip((avg_distance - 0.05) / 0.10, 0.0, 1.0)
        
        return normalized
    
    def is_hand_open(self, finger_spread):
        """
        判断手是否张开（用于激活）
        
        Args:
            finger_spread: 手指张开程度
            
        Returns:
            True if hand is open, False otherwise
        """
        return finger_spread > 0.6  # 张开程度大于60%认为是张开
    
    def is_hand_closed(self, finger_spread):
        """
        判断手是否握拳（用于退出）
        
        Args:
            finger_spread: 手指张开程度
            
        Returns:
            True if hand is closed, False otherwise
        """
        return finger_spread < 0.3  # 张开程度小于30%认为是握拳
    
    def count_fingers(self, landmarks):
        """
        计算手指数量（简化版：使用经典方法，更稳定可靠）
        
        Args:
            landmarks: MediaPipe手部关键点
            
        Returns:
            手指数量
        """
        # 手指关键点索引
        finger_tips = [4, 8, 12, 16, 20]  # 拇指、食指、中指、无名指、小指
        finger_pips = [3, 6, 10, 14, 18]  # PIP点（近端指间关节）
        
        finger_count = 0
        
        # 拇指特殊处理（拇指是横向运动的，使用x坐标比较）
        thumb_tip = landmarks.landmark[finger_tips[0]]
        thumb_ip = landmarks.landmark[finger_pips[0]]
        # 对于右手，拇指伸直时tip的x坐标应该大于ip的x坐标
        # 添加一个小的阈值来避免抖动
        if thumb_tip.x > thumb_ip.x + 0.01:
            finger_count += 1
        
        # 其他四个手指：使用y坐标比较（指尖在关节上方表示伸直）
        for i in range(1, 5):
            tip = landmarks.landmark[finger_tips[i]]
            pip = landmarks.landmark[finger_pips[i]]
            
            # 指尖y坐标小于关节y坐标表示伸直（图像中更靠上）
            # 使用较小的阈值来避免抖动，但不要太严格
            if tip.y < pip.y - 0.015:
                finger_count += 1
        
        return finger_count
    
    def is_one_finger(self, landmarks):
        """
        判断是否是手势"1"（一个手指）
        
        Args:
            landmarks: MediaPipe手部关键点
            
        Returns:
            True if one finger, False otherwise
        """
        finger_count = self.count_fingers(landmarks)
        return finger_count == 1
    
    def detect_swipe_left_right(self, current_hand_x_pixel):
        """
        检测左右滑动手势（新算法）
        
        Args:
            current_hand_x_pixel: 当前手掌中心X坐标（像素）
            
        Returns:
            滑动方向 ('left', 'right', None)
        """
        if current_hand_x_pixel is None or self.base_hand_x is None:
            return None
        
        # 检查是否在冷却时间内
        current_time = time.time()
        if self.last_swipe_time is not None:
            elapsed = current_time - self.last_swipe_time
            if elapsed < self.swipe_cooldown_duration:
                # 冷却时间内，不检测滑动
                return None
            elif elapsed >= self.swipe_cooldown_duration:
                # 冷却时间结束，更新基准点
                self.base_hand_x = current_hand_x_pixel
                self.last_swipe_time = None
                rospy.loginfo("Swipe cooldown ended, update base position")
        
        # 计算相对于基准点的位移
        displacement = current_hand_x_pixel - self.base_hand_x
        
        # 如果位移在蓄力范围内（±20像素），不算滑动
        if abs(displacement) <= self.prep_threshold_pixel:
            return None
        
        # 如果位移超过滑动阈值（±40像素），判断为滑动
        if abs(displacement) > self.swipe_threshold_pixel:
            direction = 'right' if displacement > 0 else 'left'
            # 记录滑动时间，开始冷却
            self.last_swipe_time = current_time
            rospy.loginfo("Swipe detected: %s, displacement: %.1f pixels", direction, displacement)
            return direction
        
        return None
    
    def update_vertical_displacement(self, current_hand_y_pixel):
        """
        每0.05秒更新一次上下位移
        
        逻辑：
        1. 初始化时：设置基准点（0点），位移为0
        2. 位移值 = 基准点 - 当前位置（向上为正，向下为负）
        3. 每0.05秒采样一次，用历史点相减判断方向（只判断方向，不管值）
        4. 方向反转：把当前位置设为新的基准点（0点），重新计算位移
        
        Args:
            current_hand_y_pixel: 当前手掌中心Y坐标（像素）
        """
        if current_hand_y_pixel is None:
            return
        
        current_time = time.time()
        
        # 初始化基准点（0点）
        if self.base_vertical_y is None:
            self.base_vertical_y = current_hand_y_pixel
            self.vertical_displacement = 0
            self.last_vertical_y = current_hand_y_pixel
            self.last_vertical_sample_time = current_time
            self.vertical_direction = 0
            rospy.loginfo("Initialize vertical base point: Y=%.1f pixels, displacement=0", current_hand_y_pixel)
            # 初始化时发布0（没有移动）
            msg = Float32()
            msg.data = 0.0
            self.vertical_move_pub.publish(msg)
            return
        
        # 计算位移值（根据基准点）
        # 图像坐标：向上y减小，向下y增大
        # 向上为正：base_y - current_y > 0
        # 向下为负：base_y - current_y < 0
        self.vertical_displacement = self.base_vertical_y - current_hand_y_pixel
        
        # 每0.05秒采样一次，判断方向
        if self.last_vertical_y is None or self.last_vertical_sample_time is None:
            self.last_vertical_y = current_hand_y_pixel
            self.last_vertical_sample_time = current_time
            return
        
        # 采样间隔不足0.05秒，不判断方向，但继续发布当前位移值
        if current_time - self.last_vertical_sample_time < 0.05:
            # 继续发布当前位移值
            move_value = self.vertical_displacement / 100.0
            if abs(move_value) > 1.0:
                move_value = 0.0
            msg = Float32()
            msg.data = move_value
            self.vertical_move_pub.publish(msg)
            return
        
        # 计算这0.05秒内的位移（只用于判断方向，不管值）
        dy = current_hand_y_pixel - self.last_vertical_y
        
        # 太小的移动（<5像素）视为抖动，忽略方向判断，但发布0（没有移动）
        if abs(dy) < 5.0:
            self.last_vertical_y = current_hand_y_pixel
            self.last_vertical_sample_time = current_time
            # 发布0（没有移动）
            msg = Float32()
            msg.data = 0.0
            self.vertical_move_pub.publish(msg)
            return
        
        # 判断方向（只判断方向，不管值）
        # dy < 0 表示向上移动（y坐标减小）
        # dy > 0 表示向下移动（y坐标增大）
        move_dir = 1 if dy < 0 else -1  # 1=向上，-1=向下
        
        if self.vertical_direction == 0:
            # 第一次确定方向
            self.vertical_direction = move_dir
            rospy.loginfo("Vertical direction determined: %s", "up" if move_dir == 1 else "down")
        elif move_dir != self.vertical_direction:
            # 方向反转：把当前位置设为新的基准点（0点）
            self.base_vertical_y = current_hand_y_pixel
            self.vertical_displacement = 0
            self.vertical_direction = move_dir
            # 方向反转时，发布0（没有移动）
            msg = Float32()
            msg.data = 0.0
            self.vertical_move_pub.publish(msg)
            rospy.loginfo("Vertical direction reversed: %s, reset base point to current position (displacement=0)", 
                         "up" if move_dir == 1 else "down")
        
        # 更新采样点
        self.last_vertical_y = current_hand_y_pixel
        self.last_vertical_sample_time = current_time
        
        # 发布上下移动位移值（正数=向上，负数=向下，0=没有移动）
        # 数据范围约1-100，需要缩小100倍用于控制（0.01增量）
        move_value = self.vertical_displacement / 100.0
        # 限制在合理范围内（-1.0到1.0），超出范围视为无效，发布0
        if abs(move_value) > 1.0:
            move_value = 0.0
        
        msg = Float32()
        msg.data = move_value
        self.vertical_move_pub.publish(msg)
        if abs(move_value) > 0.01:  # 只有有有效移动时才记录
            rospy.logdebug("Publish vertical move: %.3f (displacement: %.1f pixels)", move_value, self.vertical_displacement)
    
    def calculate_palm_area(self, landmarks, frame_width, frame_height):
        """
        计算手掌面积（使用关键点围成的多边形面积）
        
        Args:
            landmarks: MediaPipe手部关键点
            frame_width: 图像宽度（像素）
            frame_height: 图像高度（像素）
            
        Returns:
            手掌面积（像素²）
        """
        # 使用手掌周围的关键点来估算面积
        # 选择手掌轮廓的关键点：手腕(0), 拇指MCP(2), 食指MCP(5), 中指MCP(9), 无名指MCP(13), 小指MCP(17)
        palm_points = [0, 2, 5, 9, 13, 17]
        
        # 转换为像素坐标
        points = []
        for idx in palm_points:
            landmark = landmarks.landmark[idx]
            x = int(landmark.x * frame_width)
            y = int(landmark.y * frame_height)
            points.append([x, y])
        
        # 计算多边形面积（使用Shoelace公式）
        if len(points) < 3:
            return 0
        
        area = 0
        for i in range(len(points)):
            j = (i + 1) % len(points)
            area += points[i][0] * points[j][1]
            area -= points[j][0] * points[i][1]
         
        return abs(area) / 2.0
    
    def update_area_displacement(self, current_area):
        """
        每0.05秒更新一次面积变化（线性化处理：使用开平方根转换为等效距离）
        
        逻辑：
        1. 初始化时：设置基准点（0点），位移为0
        2. 位移值 = sqrt(当前面积) - sqrt(基准面积)（增大为正，减小为负）
        3. 每0.05秒采样一次，用历史点相减判断方向（只判断方向，不管值）
        4. 方向反转：把当前位置设为新的基准点（0点），重新计算位移
        
        Args:
            current_area: 当前手掌面积（像素²）
        """
        if current_area is None or current_area <= 0:
            return
        
        current_time = time.time()
        
        # 初始化基准点（0点）
        if self.base_area is None:
            self.base_area = current_area
            self.area_displacement = 0
            self.last_area = current_area
            self.last_area_sample_time = current_time
            self.area_direction = 0
            rospy.loginfo("Initialize area base point: area=%.1f pixels², sqrt(area)=%.1f, displacement=0", 
                         current_area, np.sqrt(current_area))
            # 初始化时发布0（没有移动）
            msg = Float32()
            msg.data = 0.0
            self.area_move_pub.publish(msg)
            return
        
        # 计算位移值（根据基准点，使用开平方根线性化）
        # 使用等效距离：distance = sqrt(area)
        # 距离增大为正：sqrt(current_area) - sqrt(base_area) > 0
        # 距离减小为负：sqrt(current_area) - sqrt(base_area) < 0
        current_distance = np.sqrt(current_area)
        base_distance = np.sqrt(self.base_area)
        self.area_displacement = current_distance - base_distance
        
        # 每0.05秒采样一次，判断方向
        if self.last_area is None or self.last_area_sample_time is None:
            self.last_area = current_area
            self.last_area_sample_time = current_time
            return
        
        # 采样间隔不足0.05秒，不判断方向，但继续发布当前位移值
        if current_time - self.last_area_sample_time < 0.05:
            # 继续发布当前位移值
            move_value = self.area_displacement / 50.0  # 缩小50倍
            if abs(move_value) > 1.0:
                move_value = 0.0
            msg = Float32()
            msg.data = move_value
            self.area_move_pub.publish(msg)
            return
        
        # 计算这0.05秒内的面积变化（只用于判断方向，不管值）
        # 方向判断仍然使用原始面积值，因为只需要知道增大还是减小
        darea = current_area - self.last_area
        
        # 太小的变化（<100像素²）视为抖动，忽略方向判断，但发布0（没有移动）
        if abs(darea) < 100.0:
            self.last_area = current_area
            self.last_area_sample_time = current_time
            # 发布0（没有移动）
            msg = Float32()
            msg.data = 0.0
            self.area_move_pub.publish(msg)
            return
        
        # 判断方向（只判断方向，不管值）
        # darea > 0 表示面积增大（靠近摄像头）
        # darea < 0 表示面积减小（远离摄像头）
        move_dir = 1 if darea > 0 else -1  # 1=增大，-1=减小
        
        if self.area_direction == 0:
            # 第一次确定方向
            self.area_direction = move_dir
            rospy.loginfo("Area direction determined: %s", "increase" if move_dir == 1 else "decrease")
        elif move_dir != self.area_direction:
            # 方向反转：把当前位置设为新的基准点（0点）
            self.base_area = current_area
            self.area_displacement = 0
            self.area_direction = move_dir
            # 方向反转时，发布0（没有移动）
            msg = Float32()
            msg.data = 0.0
            self.area_move_pub.publish(msg)
            rospy.loginfo("Area direction reversed: %s, reset base point to current area (displacement=0)", 
                         "increase" if move_dir == 1 else "decrease")
        
        # 更新采样点
        self.last_area = current_area
        self.last_area_sample_time = current_time
        
        # 发布面积变化位移值（正数=放大，负数=缩小，0=没有移动）
        # 数据范围约1-50（线性化后的等效距离），需要缩小50倍用于控制
        move_value = self.area_displacement / 50.0
        # 限制在合理范围内（-1.0到1.0），超出范围视为无效，发布0
        if abs(move_value) > 1.0:
            move_value = 0.0
        
        msg = Float32()
        msg.data = move_value
        self.area_move_pub.publish(msg)
        if abs(move_value) > 0.01:  # 只有有有效移动时才记录
            rospy.logdebug("Publish area move: %.3f (displacement: %.1f)", move_value, self.area_displacement)
    
    def detect_zoom(self, current_spread):
        """
        检测放大/缩小手势
        
        Args:
            current_spread: 当前手指张开程度
            
        Returns:
            'zoom_in' (放大/张开), 'zoom_out' (缩小/闭合), None
        """
        if current_spread is None:
            return None
        
        if self.last_finger_spread is None:
            self.last_finger_spread = current_spread
            return None
        
        # 计算变化量和变化速度
        change = current_spread - self.last_finger_spread
        change_speed = abs(change)
        
        # 检测突然变化
        if change_speed > self.zoom_speed_threshold:
            if change > 0 and current_spread > self.last_finger_spread + self.zoom_threshold:
                # 突然张开 -> 放大
                return 'zoom_in'
            elif change < 0 and current_spread < self.last_finger_spread - self.zoom_threshold:
                # 突然闭合 -> 缩小
                return 'zoom_out'
        
        self.last_finger_spread = current_spread
        return None
    
    def activate_ui_element(self, element_name):
        """
        激活UI元素（变绿）
        
        Args:
            element_name: UI元素名称
        """
        if element_name in self.ui_states:
            self.ui_states[element_name]['active'] = True
            self.ui_states[element_name]['time'] = time.time()
            rospy.loginfo("Activate UI element: %s", element_name)
    
    def update_ui_states(self):
        """
        更新UI状态（检查是否需要从绿色变回白色）
        """
        current_time = time.time()
        for element_name, state in self.ui_states.items():
            if state['active']:
                elapsed = current_time - state['time']
                if elapsed >= self.ui_highlight_duration:
                    state['active'] = False
                    rospy.logdebug("UI element %s back to white", element_name)
    
    def draw_ui(self, image):
        """
        在图像右上角绘制UI（4个箭头 + 2个圆）
        
        Args:
            image: OpenCV图像
            
        Returns:
            绘制后的图像
        """
        h, w = image.shape[:2]
        
        # UI区域大小
        ui_size = 180
        ui_margin = 20
        ui_x = w - ui_size - ui_margin
        ui_y = ui_margin
        
        # 绘制半透明背景
        overlay = image.copy()
        # 根据控制状态改变边框颜色
        border_color = (0, 255, 0) if self.control_state == 'control' else (255, 255, 255)
        cv2.rectangle(overlay, (ui_x - 10, ui_y - 10), 
                     (ui_x + ui_size + 10, ui_y + ui_size + 10), 
                     (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.3, image, 0.7, 0, image)
        # 绘制边框
        cv2.rectangle(image, (ui_x - 10, ui_y - 10), 
                     (ui_x + ui_size + 10, ui_y + ui_size + 10), 
                     border_color, 2)
        
        # 箭头大小和位置
        arrow_length = 35
        arrow_thickness = 3
        arrow_head_size = 12
        center_x = ui_x + 60  # 箭头区域中心（偏左）
        center_y = ui_y + ui_size // 2
        offset = 35
        
        # 定义颜色
        white = (255, 255, 255)
        green = (0, 255, 0)
        
        # 绘制4个标准箭头（使用cv2.arrowedLine）
        # 左箭头
        color = green if self.ui_states['arrow_left']['active'] else white
        pt1 = (center_x - offset, center_y)
        pt2 = (center_x - offset - arrow_length, center_y)
        cv2.arrowedLine(image, pt1, pt2, color, arrow_thickness, 
                       tipLength=0.4, line_type=cv2.LINE_AA)
        
        # 右箭头
        color = green if self.ui_states['arrow_right']['active'] else white
        pt1 = (center_x + offset, center_y)
        pt2 = (center_x + offset + arrow_length, center_y)
        cv2.arrowedLine(image, pt1, pt2, color, arrow_thickness, 
                       tipLength=0.4, line_type=cv2.LINE_AA)
        
        # 上箭头（上下模式：显示正数，有数值时变绿）
        if self.control_state == 'control' and self.sub_mode == 'vertical':
            # 上下模式：根据vertical_displacement决定颜色
            color = green if self.vertical_displacement > 0 else white
            # 显示正数值
            if self.vertical_displacement > 0:
                value_text = "+{}".format(int(self.vertical_displacement))
                text_size = cv2.getTextSize(value_text, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)[0]
                text_x = center_x - text_size[0] // 2
                text_y = center_y - offset - arrow_length - 5
                cv2.putText(image, value_text, (text_x, text_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        else:
            color = green if self.ui_states['arrow_up']['active'] else white
        pt1 = (center_x, center_y - offset)
        pt2 = (center_x, center_y - offset - arrow_length)
        cv2.arrowedLine(image, pt1, pt2, color, arrow_thickness, 
                       tipLength=0.4, line_type=cv2.LINE_AA)
        
        # 下箭头（上下模式：显示负数，有数值时变绿）
        if self.control_state == 'control' and self.sub_mode == 'vertical':
            # 上下模式：根据vertical_displacement决定颜色
            color = green if self.vertical_displacement < 0 else white
            # 显示负数值
            if self.vertical_displacement < 0:
                value_text = "{}".format(int(self.vertical_displacement))  # 负数自带负号
                text_size = cv2.getTextSize(value_text, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)[0]
                text_x = center_x - text_size[0] // 2
                text_y = center_y + offset + arrow_length + 20
                cv2.putText(image, value_text, (text_x, text_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        else:
            color = green if self.ui_states['arrow_down']['active'] else white
        pt1 = (center_x, center_y + offset)
        pt2 = (center_x, center_y + offset + arrow_length)
        cv2.arrowedLine(image, pt1, pt2, color, arrow_thickness, 
                       tipLength=0.4, line_type=cv2.LINE_AA)
        
        # 绘制两个圆（放在箭头右侧，往右移更多）
        circle_x = ui_x + ui_size - 30  # 右侧位置（往右移）
        
        # 大圆（放大）- 上下模式：显示正数，有数值时变绿
        large_radius = 18
        if self.control_state == 'control' and self.sub_mode == 'vertical':
            # 上下模式：根据area_displacement决定颜色
            color = green if self.area_displacement > 0 else white
            # 显示正数值
            if self.area_displacement > 0:
                value_text = "+{}".format(int(self.area_displacement))
                text_size = cv2.getTextSize(value_text, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)[0]
                text_x = circle_x - text_size[0] // 2
                text_y = center_y - 15 - large_radius - 5
                cv2.putText(image, value_text, (text_x, text_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        else:
            color = green if self.ui_states['circle_large']['active'] else white
        cv2.circle(image, (circle_x, center_y - 15), large_radius, color, 2)
        
        # 小圆（缩小）- 上下模式：显示负数，有数值时变绿
        small_radius = 12
        if self.control_state == 'control' and self.sub_mode == 'vertical':
            # 上下模式：根据area_displacement决定颜色
            color = green if self.area_displacement < 0 else white
            # 显示负数值
            if self.area_displacement < 0:
                value_text = "{}".format(int(self.area_displacement))  # 负数自带负号
                text_size = cv2.getTextSize(value_text, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)[0]
                text_x = circle_x - text_size[0] // 2
                text_y = center_y + 15 + small_radius + 20
                cv2.putText(image, value_text, (text_x, text_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        else:
            color = green if self.ui_states['circle_small']['active'] else white
        cv2.circle(image, (circle_x, center_y + 15), small_radius, color, 2)
        
        # 显示"one"和"two"标识
        # 模式1（左右模式/horizontal）："one"绿色，"two"白色
        # 模式2（上下模式/vertical）："two"绿色，"one"白色
        if self.control_state == 'control':
            if self.sub_mode == 'horizontal':
                # 模式1：one绿色，two白色
                one_color = green
                two_color = white
            else:  # vertical
                # 模式2：two绿色，one白色
                one_color = white
                two_color = green
        else:
            # 待机模式：都显示白色
            one_color = white
            two_color = white
        
        # 在UI区域左上角显示"one"和"two"
        one_text = "one"
        two_text = "two"
        font_scale = 0.6
        font_thickness = 2
        
        # "one"文本
        one_text_size = cv2.getTextSize(one_text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)[0]
        one_text_x = ui_x + 10
        one_text_y = ui_y + 25
        cv2.putText(image, one_text, (one_text_x, one_text_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, font_scale, one_color, font_thickness)
        
        # "two"文本（在"one"下方）
        two_text_size = cv2.getTextSize(two_text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)[0]
        two_text_x = ui_x + 10
        two_text_y = ui_y + 50
        cv2.putText(image, two_text, (two_text_x, two_text_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, font_scale, two_color, font_thickness)
        
        # 显示手指数量（在"two"下方）
        if self.current_finger_count is not None:
            finger_count_text = "Fingers: {}".format(self.current_finger_count)
            finger_count_font_scale = 0.5
            finger_count_font_thickness = 2
            finger_count_text_size = cv2.getTextSize(finger_count_text, cv2.FONT_HERSHEY_SIMPLEX, 
                                                    finger_count_font_scale, finger_count_font_thickness)[0]
            finger_count_x = ui_x + 10
            finger_count_y = ui_y + 75
            # 手指数量显示为黄色，更醒目
            finger_count_color = (0, 255, 255)  # 黄色 (BGR格式)
            cv2.putText(image, finger_count_text, (finger_count_x, finger_count_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, finger_count_font_scale, finger_count_color, finger_count_font_thickness)
        else:
            # 没有检测到手，显示"No hand"
            no_hand_text = "No hand"
            no_hand_font_scale = 0.5
            no_hand_font_thickness = 2
            no_hand_x = ui_x + 10
            no_hand_y = ui_y + 75
            no_hand_color = (128, 128, 128)  # 灰色
            cv2.putText(image, no_hand_text, (no_hand_x, no_hand_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, no_hand_font_scale, no_hand_color, no_hand_font_thickness)
        
        # 显示控制状态文字（英文）
        status_text = "Control Mode" if self.control_state == 'control' else "Idle Mode"
        status_color = green if self.control_state == 'control' else white
        text_size = cv2.getTextSize(status_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
        text_x = ui_x + (ui_size - text_size[0]) // 2
        text_y = ui_y + ui_size + 25
        cv2.putText(image, status_text, (text_x, text_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 1)
        
        return image
    
    def image_callback(self, msg):
        """
        图像回调函数
        
        Args:
            msg: ROS Image 消息
        """
        try:
            # 转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            if cv_image is None or cv_image.size == 0:
                return
            
            # 首次收到图像时记录日志
            if not self.image_received:
                rospy.loginfo("First image received, size: %dx%d", cv_image.shape[1], cv_image.shape[0])
                self.image_received = True
            
            # 处理图像
            processed_frame = self.process_frame(cv_image)
            
            # 保存到成员变量（线程安全）
            with self.frame_lock:
                self.latest_frame = processed_frame
            
        except Exception as e:
            rospy.logerr("Error in image callback: %s", e)
    
    def process_frame(self, frame):
        """
        处理一帧图像
        
        Args:
            frame: OpenCV图像（BGR格式）
            
        Returns:
            处理后的图像
        """
        # 转换为RGB（MediaPipe需要）
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb_frame.flags.writeable = False
        
        # 人脸检测
        face_results = self.face_detection.process(rgb_frame)
        
        # 手部检测
        hand_results = self.hands.process(rgb_frame)
        
        # 转换回BGR用于显示
        rgb_frame.flags.writeable = True
        frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
        
        # 绘制人脸检测框（只显示置信度最高且面积合理的人脸）
        if face_results.detections:
            # 找到置信度最高的人脸
            best_detection = None
            best_confidence = 0.0
            
            for detection in face_results.detections:
                confidence = detection.score[0]
                if confidence > best_confidence:
                    best_confidence = confidence
                    best_detection = detection
            
            # 只绘制置信度最高的人脸，且置信度要大于0.75
            if best_detection and best_confidence >= 0.75:
                bbox = best_detection.location_data.relative_bounding_box
                h, w = frame.shape[:2]
                
                x = int(bbox.xmin * w)
                y = int(bbox.ymin * h)
                width = int(bbox.width * w)
                height = int(bbox.height * h)
                
                # 面积过滤：人脸面积应该合理（不能太小也不能太大）
                face_area = width * height
                min_area = (w * h) * 0.01  # 至少占图像1%
                max_area = (w * h) * 0.5   # 最多占图像50%
                
                if min_area <= face_area <= max_area:
                    # 绘制矩形框
                    cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 255, 0), 2)
                    
                    # 绘制置信度
                    cv2.putText(frame, f'Face: {best_confidence:.2f}', 
                               (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 
                               0.5, (0, 255, 0), 2)
        
        # 处理手部检测（只检测右手）
        current_hand_center = None
        current_finger_spread = None
        current_hand_landmarks = None  # 保存hand_landmarks用于退出检测
        right_hand_detected = False
        
        if hand_results.multi_hand_landmarks and hand_results.multi_handedness:
            # 找到右手
            # 注意：MediaPipe的左右手判断基于手在图像中的位置，不是实际的手
            # 如果手在图像右侧，MediaPipe可能判断为'Left'（从手自己的视角）
            # 如果手在图像左侧，MediaPipe可能判断为'Right'
            # 根据用户反馈，现在检测到的是左手，说明MediaPipe判断反了
            # 所以我们应该检测label为'Left'的手（实际是右手）
            for idx, hand_landmarks in enumerate(hand_results.multi_hand_landmarks):
                if idx < len(hand_results.multi_handedness):
                    handedness = hand_results.multi_handedness[idx]
                    hand_label = handedness.classification[0].label
                    hand_score = handedness.classification[0].score
                    
                    # MediaPipe判断反了，所以检测'Left'（实际是右手）
                    if hand_label == 'Left' and hand_score > 0.5:
                        right_hand_detected = True
                        current_hand_landmarks = hand_landmarks  # 保存landmarks
                        
                        # 绘制手部关键点
                        self.mp_drawing.draw_landmarks(
                            frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                        
                        # 计算手部中心（使用手腕位置）
                        wrist = hand_landmarks.landmark[0]
                        h, w = frame.shape[:2]
                        current_hand_center = (wrist.x, wrist.y)
                        
                        # 计算手指张开程度
                        current_finger_spread = self.calculate_finger_spread(hand_landmarks)
                        
                        # 计算手指数量（用于显示）
                        self.current_finger_count = self.count_fingers(hand_landmarks)
                        break
        else:
            # 没有检测到手，重置手指数量
            self.current_finger_count = None
        
        # 状态机：处理激活/退出
        current_time = time.time()
        
        if right_hand_detected and current_finger_spread is not None:
            if self.control_state == 'idle':
                # 待机状态：检测激活手势（手掌张开保持2秒）
                if self.is_hand_open(current_finger_spread):
                    if self.activation_start_time is None:
                        self.activation_start_time = current_time
                    elif current_time - self.activation_start_time >= self.activation_hold_time:
                        # 激活控制模式
                        self.control_state = 'control'
                        self.sub_mode = 'horizontal'  # 默认左右模式
                        self.activation_start_time = None
                        # 基准点将在下一帧初始化（在控制模式检测中）
                        self.base_hand_x = None
                        self.vertical_displacement = 0
                        self.base_vertical_y = None
                        self.last_vertical_sample_time = None
                        self.last_vertical_y = None
                        self.vertical_direction = 0
                        self.area_displacement = 0
                        self.base_area = None
                        self.last_area_sample_time = None
                        self.last_area = None
                        self.area_direction = 0
                        self.one_finger_start_time = None
                        self.one_finger_confirmed = False
                        rospy.loginfo("Enter control mode (horizontal mode)")
                else:
                    # 手势不符合，重置计时
                    self.activation_start_time = None
            else:
                # 控制模式：检测退出手势（手指数量为0）
                if current_hand_landmarks is not None:
                    finger_count = self.count_fingers(current_hand_landmarks)
                    if finger_count == 0:
                        if self.deactivation_start_time is None:
                            self.deactivation_start_time = current_time
                        elif current_time - self.deactivation_start_time >= self.deactivation_hold_time:
                            # 退出控制模式
                            self.control_state = 'idle'
                            self.sub_mode = 'horizontal'  # 重置为左右模式
                            self.deactivation_start_time = None
                            self.hand_history.clear()  # 清空历史
                            self.base_hand_x = None  # 清空基准点
                            self.last_swipe_time = None  # 清空滑动时间
                            self.vertical_displacement = 0
                            self.base_vertical_y = None
                            self.last_vertical_sample_time = None
                            self.last_vertical_y = None
                            self.vertical_direction = 0
                            self.area_displacement = 0
                            self.base_area = None
                            self.last_area_sample_time = None
                            self.last_area = None
                            self.area_direction = 0
                            self.one_finger_start_time = None
                            self.one_finger_confirmed = False
                            # 重置滑动方向状态
                            if self.current_swipe_direction is not None:
                                self.current_swipe_direction = None
                                msg = String()
                                msg.data = 'None'
                                self.swipe_direction_pub.publish(msg)
                            rospy.loginfo("Exit control mode (finger count = 0)")
                    else:
                        # 手指数量不是0，重置计时
                        self.deactivation_start_time = None
                else:
                    # 没有检测到手，重置计时
                    self.deactivation_start_time = None
        else:
            # 没有检测到右手，重置计时
            if self.control_state == 'idle':
                self.activation_start_time = None
            else:
                self.deactivation_start_time = None
                # 没有检测到右手时，如果是在控制模式且是上下模式，发布0（清零）并重置位移值
                if self.control_state == 'control' and self.sub_mode == 'vertical':
                    # 重置位移值，以便下次检测到手时从零开始
                    self.vertical_displacement = 0
                    self.base_vertical_y = None
                    self.last_vertical_sample_time = None
                    self.last_vertical_y = None
                    self.vertical_direction = 0
                    self.area_displacement = 0
                    self.base_area = None
                    self.last_area_sample_time = None
                    self.last_area = None
                    self.area_direction = 0
                    
                    msg = Float32()
                    msg.data = 0.0
                    self.vertical_move_pub.publish(msg)
                    msg_area = Float32()
                    msg_area.data = 0.0
                    self.area_move_pub.publish(msg_area)
        
        # 只在控制模式下响应控制手势
        if self.control_state == 'control' and right_hand_detected:
            if current_hand_center is not None and hand_results.multi_hand_landmarks:
                h, w = frame.shape[:2]
                
                # 获取当前手部关键点（右手）
                hand_landmarks = None
                for idx, hl in enumerate(hand_results.multi_hand_landmarks):
                    if idx < len(hand_results.multi_handedness):
                        handedness = hand_results.multi_handedness[idx]
                        if handedness.classification[0].label == 'Left' and handedness.classification[0].score > 0.5:
                            hand_landmarks = hl
                            break
                
                if self.sub_mode == 'horizontal':
                    # 左右模式：检测手势"1"切换到上下模式
                    # 要求：1. 连续1秒手指数量都是1  2. 然后检测到手掌张开
                    if hand_landmarks is not None:
                        finger_count = self.count_fingers(hand_landmarks)
                        if finger_count == 1:
                            if not self.one_finger_confirmed:
                                # 第一步：检测1个手指连续1秒
                                if self.one_finger_start_time is None:
                                    self.one_finger_start_time = current_time
                                    rospy.logdebug("One finger detected, start timing...")
                                elif current_time - self.one_finger_start_time >= self.one_finger_hold_time:
                                    # 连续1秒都是1个手指，完成第一步
                                    self.one_finger_confirmed = True
                                    self.one_finger_start_time = None
                                    rospy.loginfo("Step 1 completed: one finger held for 1 second, waiting for palm open...")
                            else:
                                # 第二步：已经完成第一步，检查手掌是否张开
                                if self.is_hand_open(current_finger_spread):
                                    # 手掌张开，切换到上下模式
                                    self.sub_mode = 'vertical'
                                    self.one_finger_confirmed = False
                                    self.one_finger_start_time = None
                                    self.vertical_displacement = 0
                                    self.base_vertical_y = None
                                    self.last_vertical_sample_time = None
                                    self.last_vertical_y = None
                                    self.vertical_direction = 0
                                    # 初始化面积基准点（在进入模式2时，手掌张开，记录此时面积作为0点）
                                    self.area_displacement = 0
                                    self.base_area = None
                                    self.last_area_sample_time = None
                                    self.last_area = None
                                    self.area_direction = 0
                                    # 重置滑动方向状态
                                    if self.current_swipe_direction is not None:
                                        self.current_swipe_direction = None
                                        msg = String()
                                        msg.data = 'None'
                                        self.swipe_direction_pub.publish(msg)
                                    rospy.loginfo("Switch to vertical mode (one finger 1s + palm open)")
                        else:
                            # 手指数量不是1，重置所有状态
                            if self.one_finger_confirmed:
                                rospy.logdebug("One finger lost after confirmation, reset (finger count: %d)", finger_count)
                            elif self.one_finger_start_time is not None:
                                rospy.logdebug("One finger lost, reset timing (finger count: %d)", finger_count)
                            self.one_finger_start_time = None
                            self.one_finger_confirmed = False
                    else:
                        # 没有检测到手，重置所有状态
                        self.one_finger_start_time = None
                        self.one_finger_confirmed = False
                    
                    # 左右模式：检测左右滑动
                    current_hand_x_pixel = current_hand_center[0] * w
                    
                    # 如果基准点还没初始化，现在初始化
                    if self.base_hand_x is None:
                        self.base_hand_x = current_hand_x_pixel
                        rospy.loginfo("Initialize base hand X: %.1f pixels", self.base_hand_x)
                    
                    # 检测左右滑动
                    swipe_direction = self.detect_swipe_left_right(current_hand_x_pixel)
                    
                    # 发布滑动方向话题
                    if swipe_direction:
                        # 镜像摄像头下左右相反，左滑亮右箭头，右滑亮左箭头
                        if swipe_direction == 'left':
                            self.activate_ui_element('arrow_right')
                            # 发布"left"信息
                            if self.current_swipe_direction != 'left':
                                self.current_swipe_direction = 'left'
                                msg = String()
                                msg.data = 'left'
                                self.swipe_direction_pub.publish(msg)
                                rospy.loginfo("Publish swipe direction: left")
                        elif swipe_direction == 'right':
                            self.activate_ui_element('arrow_left')
                            # 发布"right"信息
                            if self.current_swipe_direction != 'right':
                                self.current_swipe_direction = 'right'
                                msg = String()
                                msg.data = 'right'
                                self.swipe_direction_pub.publish(msg)
                                rospy.loginfo("Publish swipe direction: right")
                    else:
                        # 没有滑动，发布"None"
                        if self.current_swipe_direction != None:
                            self.current_swipe_direction = None
                            msg = String()
                            msg.data = 'None'
                            self.swipe_direction_pub.publish(msg)
                            rospy.logdebug("Publish swipe direction: None")
                
                elif self.sub_mode == 'vertical':
                    # 上下模式：检测手势"1"切换回左右模式
                    # 要求：1. 连续1秒手指数量都是1  2. 然后检测到手掌张开
                    if hand_landmarks is not None:
                        finger_count = self.count_fingers(hand_landmarks)
                        if finger_count == 1:
                            if not self.one_finger_confirmed:
                                # 第一步：检测1个手指连续1秒
                                if self.one_finger_start_time is None:
                                    self.one_finger_start_time = current_time
                                    rospy.logdebug("One finger detected in vertical mode, start timing...")
                                elif current_time - self.one_finger_start_time >= self.one_finger_hold_time:
                                    # 连续1秒都是1个手指，完成第一步
                                    self.one_finger_confirmed = True
                                    self.one_finger_start_time = None
                                    rospy.loginfo("Step 1 completed in vertical mode: one finger held for 1 second, waiting for palm open...")
                            else:
                                # 第二步：已经完成第一步，检查手掌是否张开
                                if self.is_hand_open(current_finger_spread):
                                    # 手掌张开，切换回左右模式
                                    self.sub_mode = 'horizontal'
                                    self.one_finger_confirmed = False
                                    self.one_finger_start_time = None
                                    self.base_hand_x = None  # 重置左右模式的基准点
                                    self.last_swipe_time = None
                                    # 重置面积状态
                                    self.area_displacement = 0
                                    self.base_area = None
                                    self.last_area_sample_time = None
                                    self.last_area = None
                                    self.area_direction = 0
                                    rospy.loginfo("Switch back to horizontal mode (one finger 1s + palm open)")
                        else:
                            # 手指数量不是1，重置所有状态
                            if self.one_finger_confirmed:
                                rospy.logdebug("One finger lost after confirmation in vertical mode, reset (finger count: %d)", finger_count)
                            elif self.one_finger_start_time is not None:
                                rospy.logdebug("One finger lost in vertical mode, reset timing (finger count: %d)", finger_count)
                            self.one_finger_start_time = None
                            self.one_finger_confirmed = False
                            
                            # 上下模式：检测手掌张开，开始控制（只有在不是切换模式时才执行）
                            if self.is_hand_open(current_finger_spread):
                                # 获取当前手掌中心Y坐标（像素）
                                current_hand_y_pixel = current_hand_center[1] * h
                                # 使用采样方式更新上下位移（每0.05秒采样一次）
                                self.update_vertical_displacement(current_hand_y_pixel)
                                
                                # 同时更新面积变化（放大/缩小）
                                current_palm_area = self.calculate_palm_area(hand_landmarks, w, h)
                                self.update_area_displacement(current_palm_area)
                            else:
                                # 手掌没有张开，发布0（没有移动）
                                msg = Float32()
                                msg.data = 0.0
                                self.vertical_move_pub.publish(msg)
                                # 面积变化也发布0
                                msg_area = Float32()
                                msg_area.data = 0.0
                                self.area_move_pub.publish(msg_area)
                    else:
                        # 没有检测到手，重置所有状态并发布0（清零）
                        self.one_finger_start_time = None
                        self.one_finger_confirmed = False
                        # 重置位移值，以便下次检测到手时从零开始
                        self.vertical_displacement = 0
                        self.base_vertical_y = None
                        self.last_vertical_sample_time = None
                        self.last_vertical_y = None
                        self.vertical_direction = 0
                        self.area_displacement = 0
                        self.base_area = None
                        self.last_area_sample_time = None
                        self.last_area = None
                        self.area_direction = 0
                        
                        msg = Float32()
                        msg.data = 0.0
                        self.vertical_move_pub.publish(msg)
                        # 面积变化也发布0
                        msg_area = Float32()
                        msg_area.data = 0.0
                        self.area_move_pub.publish(msg_area)
            
            # 暂时不检测放大/缩小
            # zoom_action = self.detect_zoom(current_finger_spread)
            # if zoom_action:
            #     if zoom_action == 'zoom_in':
            #         self.activate_ui_element('circle_large')
            #     elif zoom_action == 'zoom_out':
            #         self.activate_ui_element('circle_small')
        else:
            # 不在控制模式，清空历史
            if self.control_state == 'idle':
                self.hand_history.clear()
        
        # 更新UI状态
        self.update_ui_states()
        
        # 绘制UI
        frame = self.draw_ui(frame)
        
        # 在屏幕正上方绘制10像素参考线
        h, w = frame.shape[:2]
        reference_line_y = 30  # 距离顶部30像素
        reference_line_x_start = w // 2 - 5  # 从中心向左5像素开始
        reference_line_x_end = w // 2 + 5  # 到中心向右5像素结束（总共10像素）
        
        # 绘制参考线（蓝色，粗一点方便看清）
        cv2.line(frame, 
                (reference_line_x_start, reference_line_y), 
                (reference_line_x_end, reference_line_y), 
                (255, 0, 0), 3)  # 蓝色 (BGR格式)
        
        # 在参考线下方标注"10 pixels"
        text = "10 pixels"
        text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)[0]
        text_x = reference_line_x_start - text_size[0] // 2
        text_y = reference_line_y + 20
        cv2.putText(frame, text, (text_x, text_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)  # 蓝色文字
        
        return frame
    
    def run(self):
        """主循环：显示窗口和处理用户输入"""
        rospy.loginfo("Vision control node running...")
        rospy.loginfo("Waiting for images from topic: %s", self.image_topic)
        
        window_name = "Vision Control"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        
        while not rospy.is_shutdown():
            try:
                # 从回调函数获取最新图像
                with self.frame_lock:
                    if self.latest_frame is not None:
                        frame_to_show = self.latest_frame.copy()
                    else:
                        frame_to_show = None
                
                if frame_to_show is not None:
                    # 显示图像
                    cv2.imshow(window_name, frame_to_show)
                    
                    # 发布图像（可选）
                    try:
                        image_msg = self.bridge.cv2_to_imgmsg(frame_to_show, "bgr8")
                        image_msg.header = Header()
                        image_msg.header.stamp = rospy.Time.now()
                        image_msg.header.frame_id = "camera_frame"
                        self.image_pub.publish(image_msg)
                    except Exception as e:
                        rospy.logdebug("Publish image error: %s", e)
                
                # 检查按键（'q'退出）
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    rospy.loginfo("Received exit signal")
                    break
                
            except Exception as e:
                rospy.logerr("Error in run loop: %s", e)
            
            self.rate.sleep()
        
        # 清理
        cv2.destroyAllWindows()
        rospy.loginfo("Vision control node closed")


if __name__ == "__main__":
    try:
        node = VisionControlNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Node exception: %s", e)

