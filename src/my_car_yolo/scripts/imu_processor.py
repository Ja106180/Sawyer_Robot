#!/usr/bin/env python3
"""
imu_processor.py - IMU数据处理器

功能：
1. 订阅ESP32发布的原始IMU数据 (/my_car_yolo/imu_raw)
2. 角度转换：原始数据转航向角 (0-360°)
3. 360°跳变处理：使用角度归一化避免跳变
4. 数据预处理：零偏校准、低通滤波
5. 数据融合：卡尔曼滤波融合加速度计和陀螺仪
6. 发布处理后的IMU数据 (/my_car_yolo/imu_processed)
"""

import rospy
import math
import numpy as np
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from collections import deque


class KalmanFilter:
    """简化的卡尔曼滤波器，用于角度融合"""

    def __init__(self):
        # 状态向量: [角度, 角速度]
        self.x = np.array([[0.0], [0.0]])  # 状态估计

        # 状态协方差矩阵
        self.P = np.array([[1.0, 0.0], [0.0, 1.0]])

        # 过程噪声协方差
        self.Q = np.array([[0.01, 0.0], [0.0, 0.1]])

        # 测量噪声协方差 (角度测量噪声)
        self.R = np.array([[0.1]])

        # 状态转移矩阵 (dt会在每次更新时设置)
        self.F = np.eye(2)

        # 控制矩阵 (无控制输入)
        self.B = np.zeros((2, 1))

        # 测量矩阵 (只测量角度)
        self.H = np.array([[1.0, 0.0]])

        self.last_time = None

    def predict(self, dt, gyro_measurement):
        """预测步骤"""
        # 更新状态转移矩阵
        self.F = np.array([[1.0, -dt], [0.0, 1.0]])

        # 预测状态
        self.x = self.F @ self.x + np.array([[dt * gyro_measurement], [gyro_measurement]])

        # 预测协方差
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, angle_measurement):
        """更新步骤"""
        # 计算卡尔曼增益
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # 更新状态
        y = np.array([[angle_measurement]]) - self.H @ self.x
        self.x = self.x + K @ y

        # 更新协方差
        I = np.eye(2)
        self.P = (I - K @ self.H) @ self.P

    def get_angle(self):
        """获取当前角度估计"""
        return self.x[0, 0]

    def get_rate(self):
        """获取当前角速度估计"""
        return self.x[1, 0]


class IMUProcessor:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('imu_processor', anonymous=False)

        # 获取参数（全局变量，方便修改）
        self.sample_rate = rospy.get_param('~sample_rate', 50.0)           # IMU采样率 (Hz)
        self.gyro_bias_alpha = rospy.get_param('~gyro_bias_alpha', 0.001)   # 零偏自适应系数
        self.gyro_static_thresh = rospy.get_param('~gyro_static_thresh', 0.05)  # 静止阈值 (rad/s)
        self.low_pass_alpha = rospy.get_param('~low_pass_alpha', 0.3)       # 低通滤波系数
        self.use_kalman = rospy.get_param('~use_kalman', True)              # 是否使用卡尔曼滤波
        # 航向角增益（用于校正一圈误差），默认1.0，可通过标定后在参数服务器中调整
        self.yaw_gain = rospy.get_param('~yaw_gain', 1.0)
        # 航向角积分阈值：当角速度绝对值小于该值时，认为是静止，不再积分，防止慢速漂移
        # 注意：实际IMU的零偏常常在 0.01 rad/s 量级，这里把默认阈值设得稍大一些，
        # 先保证静止时“绝对不积分”，后面再根据需要调小。
        self.yaw_static_thresh = rospy.get_param('~yaw_static_thresh', 0.05)  # rad/s

        # 陀螺仪/加速度计参数
        # 注意：ESP32 端已经把原始寄存器值转换为 SI 单位：
        #  - angular_velocity 已经是 rad/s
        #  - linear_acceleration 已经是 m/s^2
        # 因此这里不再进行二次缩放，直接使用消息中的数值。

        # 状态变量
        self.gyro_bias_x = 0.0   # X轴零偏
        self.gyro_bias_y = 0.0   # Y轴零偏
        self.gyro_bias_z = 0.0   # Z轴零偏

        self.gyro_filtered_x = 0.0  # 滤波后的角速度
        self.gyro_filtered_y = 0.0
        self.gyro_filtered_z = 0.0

        self.yaw = 0.0           # 当前航向角 (弧度)
        self.yaw_deg = 0.0       # 当前航向角 (度)
        self.last_time = None

        # 卡尔曼滤波器 (用于航向角融合)
        self.kalman = KalmanFilter()

        # 历史数据缓冲 (用于零偏校准)
        self.gyro_buffer = deque(maxlen=100)  # 存储最近100个陀螺仪数据

        # 发布者和订阅者
        self.imu_sub = rospy.Subscriber('/my_car_yolo/imu_raw', Imu, self.imu_callback)
        self.processed_pub = rospy.Publisher('/my_car_yolo/imu_processed', Imu, queue_size=10)
        # 方便调试：单独发布航向角（Z轴角度，单位：度）
        self.yaw_deg_pub = rospy.Publisher('/my_car_yolo/yaw_deg', Float32, queue_size=10)

        # 初始化完成
        self.initialized = False
        self.init_samples = 0
        self.required_init_samples = int(self.sample_rate * 2)  # 2秒初始化

        # 状态跟踪
        self.last_data_time = None
        self.connection_status = "等待ESP32连接"

        # 定时器：定期检查连接状态
        self.status_timer = rospy.Timer(rospy.Duration(2.0), self.status_callback)

        rospy.loginfo("IMU处理器已启动")
        rospy.loginfo("采样率: %.1f Hz", self.sample_rate)
        rospy.loginfo("使用卡尔曼滤波: %s", "是" if self.use_kalman else "否")
        rospy.loginfo("等待ESP32连接IMU数据...")

    def status_callback(self, event):
        """定期检查连接状态"""
        current_time = rospy.Time.now().to_sec()

        if self.last_data_time is None:
            # 还没有收到过数据
            if self.connection_status != "等待ESP32连接":
                self.connection_status = "等待ESP32连接"
                rospy.loginfo("等待ESP32连接IMU数据...")
        else:
            time_since_last_data = current_time - self.last_data_time
            if time_since_last_data > 5.0:
                # 超过5秒没有数据
                if self.connection_status != "ESP32连接断开":
                    self.connection_status = "ESP32连接断开"
                    rospy.logwarn("ESP32连接断开，等待重新连接...")
            elif self.connection_status != "ESP32已连接":
                self.connection_status = "ESP32已连接"
                rospy.loginfo("ESP32已连接，开始接收IMU数据")

                if not self.initialized:
                    rospy.loginfo("开始IMU零偏校准...")

    def normalize_angle(self, angle):
        """角度归一化，避免360°跳变问题"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def imu_callback(self, msg):
        """IMU数据回调函数"""
        # 更新数据接收时间
        self.last_data_time = rospy.Time.now().to_sec()

        current_time = msg.header.stamp.to_sec()
        if self.last_time is None:
            self.last_time = current_time
            return

        dt = current_time - self.last_time
        if dt <= 0 or dt > 0.1:  # 防止时间跳跃
            self.last_time = current_time
            return

        # 提取原始数据（ESP32 端已转换为 SI 单位：rad/s、m/s^2）
        gyro_raw_x = msg.angular_velocity.x
        gyro_raw_y = msg.angular_velocity.y
        gyro_raw_z = msg.angular_velocity.z

        accel_x = msg.linear_acceleration.x
        accel_y = msg.linear_acceleration.y
        accel_z = msg.linear_acceleration.z

        # 零偏校准 (使用前2秒数据)
        if not self.initialized:
            self.gyro_buffer.append((gyro_raw_x, gyro_raw_y, gyro_raw_z))
            self.init_samples += 1

            if self.init_samples >= self.required_init_samples:
                # 计算平均零偏
                gyro_samples = np.array(self.gyro_buffer)
                self.gyro_bias_x = np.mean(gyro_samples[:, 0])
                self.gyro_bias_y = np.mean(gyro_samples[:, 1])
                self.gyro_bias_z = np.mean(gyro_samples[:, 2])

                # 自适应零偏校准
                if abs(self.gyro_bias_z) < self.gyro_static_thresh:
                    self.gyro_bias_z = 0.0  # 静止时清零

                self.initialized = True
                rospy.loginfo("零偏校准完成: bias_z = %.6f rad/s", self.gyro_bias_z)
            # 注意：这里不再提前返回，让程序继续处理数据

        # 零偏补偿 + 增益校正（主要用于Z轴航向角）
        gyro_corrected_x = (gyro_raw_x - self.gyro_bias_x)
        gyro_corrected_y = (gyro_raw_y - self.gyro_bias_y)
        gyro_corrected_z = (gyro_raw_z - self.gyro_bias_z) * self.yaw_gain

        # 自适应零偏校准 (运行时缓慢调整)
        if abs(gyro_corrected_z) < self.gyro_static_thresh:
            self.gyro_bias_z = (1.0 - self.gyro_bias_alpha) * self.gyro_bias_z + \
                              self.gyro_bias_alpha * gyro_raw_z

        # 低通滤波
        self.gyro_filtered_x = (1.0 - self.low_pass_alpha) * self.gyro_filtered_x + \
                              self.low_pass_alpha * gyro_corrected_x
        self.gyro_filtered_y = (1.0 - self.low_pass_alpha) * self.gyro_filtered_y + \
                              self.low_pass_alpha * gyro_corrected_y
        self.gyro_filtered_z = (1.0 - self.low_pass_alpha) * self.gyro_filtered_z + \
                              self.low_pass_alpha * gyro_corrected_z

        # 角度积分 (只使用Z轴陀螺仪计算航向角)
        # 为了防止静止时微小零偏导致角度缓慢漂移，这里加一个静止阈值：
        # 当 |gyro_corrected_z| 很小（认为没有真正转动）时，不再积分。
        if abs(gyro_corrected_z) < self.yaw_static_thresh:
            omega_z_for_integrate = 0.0
        else:
            omega_z_for_integrate = self.gyro_filtered_z

        self.yaw += omega_z_for_integrate * dt
        self.yaw = self.normalize_angle(self.yaw)  # 避免跳变
        self.yaw_deg = math.degrees(self.yaw)

        # 说明：对于航向角（绕 Z 轴），单凭加速度计无法直接得到绝对角度，
        # 原来的卡尔曼融合会把“倾角”错误地当成航向角覆盖掉，导致你看到
        # 只有左右倾斜才变化明显、绕Z轴旋转几乎不变。
        # 这里保留卡尔曼类以便以后扩展，但不再用加速度计去覆盖 yaw，
        # 航向角仅由陀螺仪积分 + 零偏校正得到。

        # 创建并发布处理后的IMU消息
        processed_msg = Imu()
        processed_msg.header = msg.header
        processed_msg.header.frame_id = "imu_link"

        # 方向 (使用四元数表示航向角)
        from tf.transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, self.yaw)  # roll=0, pitch=0, yaw=处理后的航向角
        processed_msg.orientation.x = q[0]
        processed_msg.orientation.y = q[1]
        processed_msg.orientation.z = q[2]
        processed_msg.orientation.w = q[3]

        # 角速度 (滤波后的)
        processed_msg.angular_velocity.x = self.gyro_filtered_x
        processed_msg.angular_velocity.y = self.gyro_filtered_y
        processed_msg.angular_velocity.z = self.gyro_filtered_z

        # 线加速度 (原始数据，IMU已做了一些处理)
        processed_msg.linear_acceleration.x = accel_x
        processed_msg.linear_acceleration.y = accel_y
        processed_msg.linear_acceleration.z = accel_z

        # 设置协方差 (简化处理)
        processed_msg.orientation_covariance[0] = 0.01  # 航向角误差
        processed_msg.orientation_covariance[4] = 0.01
        processed_msg.orientation_covariance[8] = 0.01

        processed_msg.angular_velocity_covariance[0] = 0.01
        processed_msg.angular_velocity_covariance[4] = 0.01
        processed_msg.angular_velocity_covariance[8] = 0.01

        processed_msg.linear_acceleration_covariance[0] = 0.1
        processed_msg.linear_acceleration_covariance[4] = 0.1
        processed_msg.linear_acceleration_covariance[8] = 0.1

        self.processed_pub.publish(processed_msg)

        # 发布简化的Z轴角度（单位：度），方便调试查看
        yaw_msg = Float32()
        yaw_msg.data = self.yaw_deg
        self.yaw_deg_pub.publish(yaw_msg)

        # 调试输出 (降低频率)
        if int(current_time * 10) % 10 == 0:  # 每0.1秒输出一次
            rospy.loginfo_throttle(0.1, "航向角: %.1f°, 角速度: %.2f°/s, 零偏: %.3f°/s",
                                  self.yaw_deg,
                                  math.degrees(self.gyro_filtered_z),
                                  math.degrees(self.gyro_bias_z))

        self.last_time = current_time


def main():
    try:
        processor = IMUProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("IMU处理器已停止")


if __name__ == '__main__':
    main()
