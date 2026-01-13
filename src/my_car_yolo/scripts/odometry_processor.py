#!/usr/bin/env python3
"""
odometry_processor.py - 里程计数据处理器

功能：
1. 订阅ESP32发布的原始编码器数据 (/my_car_yolo/wheel_encoders)
2. 脉冲计数去抖：连续检测跳变才计数，避免电磁干扰
3. 运动学模型：脉冲转角度、角度转位移/速度
4. 航迹推演：更新小车x/y坐标和航向角(θ)
5. 数据预处理：滤波与校准
6. 数据融合：结合IMU抑制漂移
7. 发布处理后的里程计数据 (/odom)
"""

import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Quaternion, TransformStamped
from tf.transformations import quaternion_from_euler
import tf2_ros
from collections import deque


class ComplementaryFilter:
    """互补滤波器：融合编码器和IMU"""

    def __init__(self, alpha=0.98):
        self.alpha = alpha  # IMU权重
        self.angle = 0.0    # 融合后的角度

    def update(self, encoder_angle, imu_angle, dt):
        """更新融合角度"""
        # 编码器角度积分 (低频，高精度)
        # IMU角度 (高频，低漂移)
        self.angle = self.alpha * (self.angle + imu_angle * dt) + \
                    (1.0 - self.alpha) * encoder_angle
        return self.angle


class OdometryProcessor:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('odometry_processor', anonymous=False)

        # 获取参数（全局变量，方便修改）
        self.ticks_per_rev = rospy.get_param('~ticks_per_rev', 260.0)       # 每转脉冲数
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.048)         # 轮子半径 (m)
        self.wheel_base = rospy.get_param('~wheel_base', 0.1475)            # 轮距 (m)
        self.update_rate = rospy.get_param('~update_rate', 20.0)            # 更新频率 (Hz)

        # 去抖参数
        self.debounce_count = rospy.get_param('~debounce_count', 2)         # 连续检测次数
        self.max_velocity = rospy.get_param('~max_velocity', 1.0)           # 最大速度 (m/s)

        # 滤波参数
        self.velocity_filter_alpha = rospy.get_param('~velocity_filter_alpha', 0.3)
        self.use_complementary = rospy.get_param('~use_complementary', True)  # 使用互补滤波

        # 运动学参数
        self.ticks_per_meter = self.ticks_per_rev / (2.0 * math.pi * self.wheel_radius)

        # 状态变量
        self.x = 0.0          # 小车X坐标 (m)
        self.y = 0.0          # 小车Y坐标 (m)
        self.theta = 0.0      # 小车航向角 (rad)

        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.last_time = None

        # 速度滤波
        self.left_velocity_filtered = 0.0
        self.right_velocity_filtered = 0.0

        # 去抖计数器
        self.left_debounce_counter = 0
        self.right_debounce_counter = 0
        self.last_left_direction = 0
        self.last_right_direction = 0

        # 互补滤波器
        self.complementary_filter = ComplementaryFilter(alpha=0.95)

        # IMU数据缓存
        self.imu_yaw = 0.0
        self.imu_yaw_rate = 0.0

        # 发布者和订阅者
        self.encoders_sub = rospy.Subscriber('/my_car_yolo/wheel_encoders', Int32MultiArray, self.encoders_callback)
        self.imu_sub = rospy.Subscriber('/my_car_yolo/imu_processed', Imu, self.imu_callback)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

        # TF广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # 初始化完成
        self.initialized = False

        # 状态跟踪
        self.last_encoder_time = None
        self.encoder_connection_status = "等待ESP32连接"

        # 定时器：定期检查连接状态
        self.status_timer = rospy.Timer(rospy.Duration(2.0), self.status_callback)

        rospy.loginfo("里程计处理器已启动")
        rospy.loginfo("轮子半径: %.3f m, 轮距: %.3f m", self.wheel_radius, self.wheel_base)
        rospy.loginfo("每转脉冲数: %.0f, 每米脉冲数: %.1f", self.ticks_per_rev, self.ticks_per_meter)
        rospy.loginfo("使用互补滤波: %s", "是" if self.use_complementary else "否")
        rospy.loginfo("等待ESP32连接编码器数据...")

    def status_callback(self, event):
        """定期检查连接状态"""
        current_time = rospy.Time.now().to_sec()

        if self.last_encoder_time is None:
            # 还没有收到过编码器数据
            if self.encoder_connection_status != "等待ESP32连接":
                self.encoder_connection_status = "等待ESP32连接"
                # 不重复打印日志
        else:
            time_since_last_data = current_time - self.last_encoder_time
            if time_since_last_data > 5.0:
                # 超过5秒没有数据
                if self.encoder_connection_status != "ESP32连接断开":
                    self.encoder_connection_status = "ESP32连接断开"
                    rospy.logwarn("ESP32编码器连接断开，等待重新连接...")
            elif self.encoder_connection_status != "ESP32已连接":
                self.encoder_connection_status = "ESP32已连接"
                rospy.loginfo("ESP32已连接，开始接收编码器数据")

                if not self.initialized:
                    self.initialized = True
                    self.last_left_ticks = 0
                    self.last_right_ticks = 0
                    self.last_time = current_time
                    rospy.loginfo("里程计初始化完成")

    def imu_callback(self, msg):
        """IMU数据回调 - 处理sensor_msgs/Imu消息"""
        # 从四元数提取航向角
        from tf.transformations import quaternion_matrix
        import numpy as np

        quaternion = [msg.orientation.x, msg.orientation.y,
                     msg.orientation.z, msg.orientation.w]

        # 计算旋转矩阵
        rot_matrix = quaternion_matrix(quaternion)

        # 提取航向角 (绕Z轴旋转)
        self.imu_yaw = math.atan2(rot_matrix[1, 0], rot_matrix[0, 0])

        # 获取角速度 (绕Z轴)
        self.imu_yaw_rate = msg.angular_velocity.z

    def debounce_pulse(self, current_ticks, last_ticks, debounce_counter, last_direction):
        """脉冲去抖处理"""
        direction = 1 if current_ticks > last_ticks else -1 if current_ticks < last_ticks else 0

        if direction == 0:
            # 无变化，重置计数器
            return 0, debounce_counter, direction

        if direction == last_direction:
            # 相同方向，增加计数器
            debounce_counter += 1
            if debounce_counter >= self.debounce_count:
                # 达到阈值，返回有效脉冲
                return direction, 0, direction  # 重置计数器
            else:
                return 0, debounce_counter, direction
        else:
            # 方向改变，重置计数器
            return 0, 1, direction

    def encoders_callback(self, msg):
        """编码器数据回调 - 处理Int32MultiArray消息"""
        # 更新数据接收时间
        self.last_encoder_time = rospy.Time.now().to_sec()

        # ESP32没有header，使用当前时间
        current_time = self.last_encoder_time

        # 解析Int32MultiArray: [left_pulses, right_pulses, left_velocity*1000, right_velocity*1000]
        if len(msg.data) < 4:
            rospy.logwarn("编码器数据格式错误，期望4个元素，收到%d个", len(msg.data))
            return

        left_pulses = msg.data[0]
        right_pulses = msg.data[1]
        left_velocity_from_esp32 = msg.data[2] / 1000.0  # 转换为m/s
        right_velocity_from_esp32 = msg.data[3] / 1000.0  # 转换为m/s

        if not self.initialized:
            # 初始化上一次数据
            self.last_left_ticks = left_pulses
            self.last_right_ticks = right_pulses
            self.last_time = current_time
            self.initialized = True
            return

        if self.last_time is None:
            self.last_time = current_time
            return

        dt = current_time - self.last_time
        if dt <= 0 or dt > 0.1:  # 防止时间异常
            self.last_time = current_time
            return

        # 去抖处理
        left_delta, self.left_debounce_counter, self.last_left_direction = \
            self.debounce_pulse(left_pulses, self.last_left_ticks,
                              self.left_debounce_counter, self.last_left_direction)

        right_delta, self.right_debounce_counter, self.last_right_direction = \
            self.debounce_pulse(right_pulses, self.last_right_ticks,
                               self.right_debounce_counter, self.last_right_direction)

        # 计算轮子转角 (弧度)
        left_angle_delta = (left_delta * 2.0 * math.pi) / self.ticks_per_rev
        right_angle_delta = (right_delta * 2.0 * math.pi) / self.ticks_per_rev

        # 计算轮子速度 (m/s)
        left_velocity_raw = (left_angle_delta * self.wheel_radius) / dt
        right_velocity_raw = (right_angle_delta * self.wheel_radius) / dt

        # 速度限制和滤波
        left_velocity_raw = np.clip(left_velocity_raw, -self.max_velocity, self.max_velocity)
        right_velocity_raw = np.clip(right_velocity_raw, -self.max_velocity, self.max_velocity)

        # 低通滤波
        self.left_velocity_filtered = (1.0 - self.velocity_filter_alpha) * self.left_velocity_filtered + \
                                    self.velocity_filter_alpha * left_velocity_raw
        self.right_velocity_filtered = (1.0 - self.velocity_filter_alpha) * self.right_velocity_filtered + \
                                     self.velocity_filter_alpha * right_velocity_raw

        # 差分驱动运动学
        linear_velocity = (self.right_velocity_filtered + self.left_velocity_filtered) / 2.0
        angular_velocity = (self.right_velocity_filtered - self.left_velocity_filtered) / self.wheel_base

        # 航向角更新
        if self.use_complementary and self.imu_yaw != 0.0:
            # 使用互补滤波融合编码器和IMU
            encoder_theta = self.theta + angular_velocity * dt
            self.theta = self.complementary_filter.update(encoder_theta, self.imu_yaw_rate, dt)
        else:
            # 仅使用编码器
            self.theta += angular_velocity * dt

        # 归一化角度到 -pi 到 pi
        while self.theta > math.pi:
            self.theta -= 2.0 * math.pi
        while self.theta < -math.pi:
            self.theta += 2.0 * math.pi

        # 位置更新
        self.x += linear_velocity * dt * math.cos(self.theta)
        self.y += linear_velocity * dt * math.sin(self.theta)

        # 创建里程计消息
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()  # ESP32没有header，使用当前时间
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # 位姿
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # 姿态 (四元数)
        quat = quaternion_from_euler(0.0, 0.0, self.theta)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # 位姿协方差 (简化)
        odom_msg.pose.covariance[0] = 0.1  # x
        odom_msg.pose.covariance[7] = 0.1  # y
        odom_msg.pose.covariance[35] = 0.1 # theta

        # 速度
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity

        # 速度协方差 (简化)
        odom_msg.twist.covariance[0] = 0.1   # linear_x
        odom_msg.twist.covariance[35] = 0.1  # angular_z

        # 发布里程计
        self.odom_pub.publish(odom_msg)

        # 发布TF变换
        self.publish_tf(odom_msg)

        # 更新上一次数据
        self.last_left_ticks = left_pulses
        self.last_right_ticks = right_pulses
        self.last_time = current_time

        # 调试输出
        if int(current_time * 10) % 50 == 0:  # 每0.5秒输出一次
            rospy.loginfo_throttle(0.5,
                "位置: (%.3f, %.3f), 角度: %.1f°, 速度: %.3f m/s, %.1f°/s",
                self.x, self.y, math.degrees(self.theta),
                linear_velocity, math.degrees(angular_velocity))

    def publish_tf(self, odom_msg):
        """发布TF变换"""
        transform = TransformStamped()
        transform.header = odom_msg.header
        transform.child_frame_id = odom_msg.child_frame_id

        transform.transform.translation.x = odom_msg.pose.pose.position.x
        transform.transform.translation.y = odom_msg.pose.pose.position.y
        transform.transform.translation.z = odom_msg.pose.pose.position.z

        transform.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform)


def main():
    try:
        processor = OdometryProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("里程计处理器已停止")


if __name__ == '__main__':
    main()
