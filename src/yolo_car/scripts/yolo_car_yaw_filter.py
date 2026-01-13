#!/usr/bin/env python3
# yolo_car_yaw_filter.py
# 订阅原始 /yolo_car/cmd_vel 与 /yolo_car/imu_raw（仅用 gyro.z 积分 yaw），
# 对直行时的航向漂移做简单抑制，限制原地打圈，再发布到 /yolo_car/cmd_vel。

import math
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu


class YoloCarYawFilter:
    def __init__(self):
        rospy.init_node("yolo_car_yaw_filter", anonymous=False)

        # 参数
        self.k_yaw = float(rospy.get_param("~k_yaw", 0.6))               # 航向误差增益
        self.yaw_deadband = float(rospy.get_param("~yaw_deadband", 0.05))  # 航向误差死区(rad)
        self.max_ang = float(rospy.get_param("~max_ang", 0.2))          # 最终角速度限幅(rad/s)
        self.straight_lin_min = float(rospy.get_param("~straight_lin_min", 0.03))  # 判定直行的最小线速
        self.straight_ang_deadband = float(rospy.get_param("~straight_ang_deadband", 0.05))  # 判定直行的角速阈值
        self.spin_limit = float(rospy.get_param("~spin_limit", 0.8))    # 认为在快速旋转的角速度阈值(rad/s)
        # 低通 + 零速校零
        self.gyro_alpha = float(rospy.get_param("~gyro_alpha", 0.3))    # 角速度 EMA 系数（0~1，大=更跟随，小=更平滑）
        self.gyro_static_thresh = float(rospy.get_param("~gyro_static_thresh", 0.05))  # 认为静止的阈值
        self.gyro_bias_alpha = float(rospy.get_param("~gyro_bias_alpha", 0.001))  # 零偏自适应系数（静止时缓慢校零）

        # 状态
        self.yaw = 0.0
        self.yaw_ref = None
        self.last_imu_time = None
        self.gyro_z = 0.0
        self.gyro_z_filt = 0.0
        self.gyro_bias = 0.0

        # 话题
        self.cmd_pub = rospy.Publisher("/yolo_car/cmd_vel", Twist, queue_size=1)
        self.yaw_pub = rospy.Publisher("/yolo_car/yaw", Float32, queue_size=1)
        self.gyro_pub = rospy.Publisher("/yolo_car/gyro_z_filt", Float32, queue_size=1)
        self.cmd_sub = rospy.Subscriber("/yolo_car/cmd_vel_raw", Twist, self.cmd_cb, queue_size=1)
        self.imu_sub = rospy.Subscriber("/yolo_car/imu_raw", Imu, self.imu_cb, queue_size=20)

        rospy.loginfo("yaw_filter 参数: k_yaw=%.3f deadband=%.3f max_ang=%.3f gyro_alpha=%.3f",
                      self.k_yaw, self.yaw_deadband, self.max_ang, self.gyro_alpha)

    def imu_cb(self, msg: Imu):
        # 积分角速度 z 轴，得到相对航向
        now = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()
        if self.last_imu_time is not None:
            dt = now - self.last_imu_time
            if dt > 0.0005:
                raw_gz = msg.angular_velocity.z
                # 静止时缓慢校零
                if abs(raw_gz) < self.gyro_static_thresh:
                    self.gyro_bias = (1.0 - self.gyro_bias_alpha) * self.gyro_bias + self.gyro_bias_alpha * raw_gz
                # 一阶低通滤波
                self.gyro_z_filt = (1.0 - self.gyro_alpha) * self.gyro_z_filt + self.gyro_alpha * (raw_gz - self.gyro_bias)
                self.gyro_z = self.gyro_z_filt

                self.yaw += self.gyro_z * dt
                # 归一化到 [-pi, pi]
                self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
                # 发布当前 yaw（调试用）
                yaw_msg = Float32()
                yaw_msg.data = float(self.yaw)
                self.yaw_pub.publish(yaw_msg)
                gz_msg = Float32()
                gz_msg.data = float(self.gyro_z)
                self.gyro_pub.publish(gz_msg)
        self.last_imu_time = now

    def cmd_cb(self, msg: Twist):
        # 如果没有 IMU 数据，直接透传
        if self.last_imu_time is None:
            self.cmd_pub.publish(msg)
            return

        linear_x = msg.linear.x
        ang_raw = msg.angular.z

        # 判断是否处于“直行段”，用 yaw_ref 锁定航向
        is_straight = (abs(ang_raw) < self.straight_ang_deadband) and (abs(linear_x) > self.straight_lin_min)
        if is_straight or self.yaw_ref is None:
            # 更新参考航向
            self.yaw_ref = self.yaw

        yaw_error = 0.0
        if self.yaw_ref is not None:
            yaw_error = self.yaw - self.yaw_ref
            # 归一化误差
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        ang_out = ang_raw

        # 航向纠偏：仅在“直行”时施加
        if is_straight:
            if abs(yaw_error) < self.yaw_deadband:
                yaw_error = 0.0
            ang_out = ang_raw - self.k_yaw * yaw_error

        # 防止原地快速旋转：若角速度大且线速度很小，则抑制
        if abs(self.gyro_z) > self.spin_limit and abs(linear_x) < self.straight_lin_min:
            ang_out = 0.0

        # 限幅
        ang_out = max(-self.max_ang, min(self.max_ang, ang_out))

        out_msg = Twist()
        out_msg.linear = msg.linear
        out_msg.angular = msg.angular
        out_msg.angular.z = ang_out
        self.cmd_pub.publish(out_msg)


def main():
    try:
        YoloCarYawFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

