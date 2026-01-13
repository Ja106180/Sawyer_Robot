#!/usr/bin/env python3
"""
yolo_car_control.py - YOLO小车控制节点（矩形巡线）

功能：
1. 订阅牌子检测结果
2. 让牌子中心沿既定矩形边顺时针巡线（图像坐标），恒速前进 + 法向纠偏
3. 发布速度指令到 /yolo_car/cmd_vel
4. 识别不到时停车并舵机扫描
"""

import rospy
import numpy as np
import cv2
from geometry_msgs.msg import Twist, Point
from yolo_car.msg import ObjectDetections
from std_msgs.msg import Float32


class YOLOCarControl:
    def __init__(self):
        rospy.init_node("yolo_car_control", anonymous=False)

        # 参数：矩形巡线控制
        # PID 控制参数（航向/法向误差）
        self.kp_ang = float(rospy.get_param("~k_ang_p", 0.6))
        self.ki_ang = float(rospy.get_param("~k_ang_i", 0.0))
        self.kd_ang = float(rospy.get_param("~k_ang_d", 0.0))
        self.i_limit = float(rospy.get_param("~k_ang_i_limit", 0.3))  # 积分限幅

        self.edge_deadband = float(rospy.get_param("~edge_deadband", 0.02))  # 靠线死区，米
        # 中间死区，误差小于此直接设角速度=0，避免单边打圈
        self.mid_deadband = float(rospy.get_param("~mid_deadband", 0.05))
        self.switch_threshold = float(rospy.get_param("~switch_threshold", 0.05))  # 顶点切边阈值，米
        # TT 马达需要一定占空比才动，适当提高基准和最小速度
        self.base_linear_speed = float(rospy.get_param("~base_linear_speed", 0.05))
        self.max_linear_vel = float(rospy.get_param("~max_linear_vel", 0.15))
        self.max_angular_vel = float(rospy.get_param("~max_angular_vel", 0.05))
        self.angular_scale = float(rospy.get_param("~angular_scale", 0.12))
        self.invert_x_control = bool(rospy.get_param("~invert_x_control", False))
        self.control_rate = float(rospy.get_param("~control_rate", 20.0))
        self.lost_timeout = float(rospy.get_param("~lost_timeout", 3.0))

        # PID 内部状态
        self.int_ang = 0.0
        self.prev_ang_err = 0.0
        self.prev_time = rospy.Time.now()

        # 状态变量
        self.car_center = None  # 牌子中心点 (x, y)
        self.car_center_ground = None  # 映射到地面坐标的中心点
        self.last_cmd_vel = Twist()
        self.last_detection_time = None
        self.car_detected = False

        # 舵机扫描相关
        self.servo_scan_angles = [0, 90, 180, 90, 0, 90, 180]
        self.servo_scan_index = 0
        self.servo_scan_interval = 1.5
        self.last_servo_time = None
        self.servo_scanning = False

        # 发布者
        self.cmd_vel_pub = rospy.Publisher("/yolo_car/cmd_vel", Twist, queue_size=1)
        self.servo_angle_pub = rospy.Publisher("/yolo_car/servo_angle", Float32, queue_size=1)

        # 订阅者
        self.detections_sub = rospy.Subscriber("/yolo_car/detections", ObjectDetections, self.detections_callback)
        # 保留订阅以兼容，但不用于控制
        self.target_point_sub = rospy.Subscriber("/yolo_car/target_point", Point, self.target_point_callback)

        # 矩形顶点（顺时针）
        # 像素与地面点（米）对应，默认使用讨论确定的矩形
        pixel_default = [
            [214.0, 173.0],  # 左上
            [370.0, 170.0],  # 右上
            [397.0, 354.0],  # 右下
            [121.0, 361.0],  # 左下
        ]
        ground_default = [
            [0.0, 0.0],
            [0.60, 0.0],
            [0.60, 1.20],
            [0.0, 1.20],
        ]
        # 参数可覆盖，格式 [x0,y0,x1,y1,x2,y2,x3,y3]
        pixel_param = rospy.get_param("~rect_pixels", [])
        ground_param = rospy.get_param("~rect_ground", [])
        if len(pixel_param) == 8:
            pixel_default = [[pixel_param[0], pixel_param[1]],
                             [pixel_param[2], pixel_param[3]],
                             [pixel_param[4], pixel_param[5]],
                             [pixel_param[6], pixel_param[7]]]
        if len(ground_param) == 8:
            ground_default = [[ground_param[0], ground_param[1]],
                              [ground_param[2], ground_param[3]],
                              [ground_param[4], ground_param[5]],
                              [ground_param[6], ground_param[7]]]

        self.pixel_vertices = np.array(pixel_default, dtype=np.float32)
        self.rect_vertices = [np.array(p, dtype=np.float32) for p in ground_default]
        self.H = self._compute_homography(self.pixel_vertices, np.array(ground_default, dtype=np.float32))
        self.current_edge = 0  # 当前边索引

        rospy.loginfo("YOLO小车控制节点已启动")
        rospy.loginfo("PID控制 - kp=%.4f ki=%.4f kd=%.4f", self.kp_ang, self.ki_ang, self.kd_ang)
        rospy.loginfo("靠线死区: %.3f m, 切边阈值: %.3f m", self.edge_deadband, self.switch_threshold)
        rospy.loginfo("速度限制: 线速度=%.2f m/s, 角速度=%.2f rad/s", self.max_linear_vel, self.max_angular_vel)
        if self.H is None:
            rospy.logwarn("未能计算单应矩阵，控制将保持像素坐标系")
        else:
            rospy.loginfo("单应矩阵已计算，可将像素映射到地面坐标")

        # 启动控制循环
        self.control_loop()

    def detections_callback(self, msg):
        """检测结果回调"""
        if len(msg.objects) > 0:
            obj = msg.objects[0]
            self.car_center = (obj.center_x, obj.center_y)
            if self.H is not None:
                self.car_center_ground = self._pixel_to_ground(self.car_center)
            else:
                self.car_center_ground = None
            self.last_detection_time = rospy.Time.now()
            self.car_detected = True
            self.servo_scanning = False
            self.servo_scan_index = 0
            # 初始选最近边
            if self.H is not None and self.car_center_ground is not None:
                self.current_edge = self._nearest_edge(np.array(self.car_center_ground))
            else:
                self.current_edge = self._nearest_edge(np.array([obj.center_x, obj.center_y]))
        else:
            self.car_detected = False

    def target_point_callback(self, msg):
        """兼容接口，未使用"""
        pass

    def _point_to_segment_distance(self, p, a, b):
        """点到线段距离"""
        ab = b - a
        ap = p - a
        ab_len2 = np.dot(ab, ab)
        if ab_len2 == 0:
            return np.linalg.norm(ap)
        t = np.clip(np.dot(ap, ab) / ab_len2, 0.0, 1.0)
        proj = a + t * ab
        return np.linalg.norm(p - proj)

    def _compute_homography(self, pixels, ground):
        """从像素四点到地面四点计算单应矩阵"""
        try:
            H, status = cv2.findHomography(pixels, ground, method=0)
            if H is None:
                return None
            return H
        except Exception as e:
            rospy.logwarn("计算单应矩阵失败: %s", e)
            return None

    def _pixel_to_ground(self, pt):
        """像素点 -> 地面坐标"""
        if self.H is None:
            return None
        u, v = pt
        src = np.array([u, v, 1.0], dtype=np.float32)
        dst = self.H.dot(src)
        if dst[2] == 0:
            return None
        x = dst[0] / dst[2]
        y = dst[1] / dst[2]
        return (float(x), float(y))

    def _nearest_edge(self, pt):
        """找到距离点最近的边索引"""
        min_dist = 1e9
        min_idx = 0
        n = len(self.rect_vertices)
        for i in range(n):
            a = self.rect_vertices[i]
            b = self.rect_vertices[(i + 1) % n]
            dist = self._point_to_segment_distance(pt, a, b)
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        return min_idx

    def _edge_frame(self, idx):
        """返回边的起点、终点、切向、法向、长度"""
        a = self.rect_vertices[idx]
        b = self.rect_vertices[(idx + 1) % len(self.rect_vertices)]
        ab = b - a
        length = np.linalg.norm(ab)
        if length == 0:
            return a, b, np.array([1.0, 0.0]), np.array([0.0, 1.0]), 1.0
        t = ab / length
        n = np.array([-t[1], t[0]])  # 左手法向
        return a, b, t, n, length

    def control_loop(self):
        """控制循环"""
        rate = rospy.Rate(self.control_rate)

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()

            if self.car_detected and self.car_center is not None:
                if self.H is not None and self.car_center_ground is not None:
                    p = np.array(self.car_center_ground, dtype=np.float32)
                else:
                    p = np.array([self.car_center[0], self.car_center[1]], dtype=np.float32)

                # 首选当前边，偏离过大才重选最近边（避免频繁切换导致打圈）
                a, b, t_vec, n_vec, length = self._edge_frame(self.current_edge)
                ap = p - a
                s = np.dot(ap, t_vec)
                s_clamped = np.clip(s, 0.0, length)
                proj = a + s_clamped * t_vec
                en = np.dot(p - proj, n_vec)
                dist_edge = abs(en)
                if dist_edge > (0.25 if self.H is not None else 160.0):
                    self.current_edge = self._nearest_edge(p)
                a, b, t_vec, n_vec, length = self._edge_frame(self.current_edge)
                ap = p - a
                s = np.dot(ap, t_vec)      # 沿边方向投影
                s_clamped = np.clip(s, 0.0, length)
                proj = a + s_clamped * t_vec  # 最近点投影
                en = np.dot(p - proj, n_vec)  # 法向距离（投影点）

                # 到达顶点则切换下一边
                if s >= (length - self.switch_threshold):
                    self.current_edge = (self.current_edge + 1) % len(self.rect_vertices)
                    a, b, t_vec, n_vec, length = self._edge_frame(self.current_edge)
                    ap = p - a
                    s = np.dot(ap, t_vec)
                    s_clamped = np.clip(s, 0.0, length)
                    proj = a + s_clamped * t_vec
                    en = np.dot(p - proj, n_vec)

                # 靠线死区
                if abs(en) < self.edge_deadband:
                    en = 0.0

                # 中间死区，避免小偏差也打圈
                if abs(en) < self.mid_deadband:
                    angular_vel = 0.0
                else:
                    # PID 控制
                    now = rospy.Time.now()
                    dt = (now - self.prev_time).to_sec()
                    dt = max(dt, 1e-3)
                    # 积分
                    self.int_ang += en * dt
                    self.int_ang = np.clip(self.int_ang, -self.i_limit, self.i_limit)
                    # 微分
                    der = (en - self.prev_ang_err) / dt
                    pid_out = self.kp_ang * en + self.ki_ang * self.int_ang + self.kd_ang * der
                    self.prev_ang_err = en
                    self.prev_time = now

                    angular_vel = pid_out
                    if self.invert_x_control:
                        angular_vel = -angular_vel
                    angular_vel *= self.angular_scale
                    angular_vel = np.clip(angular_vel, -self.max_angular_vel, self.max_angular_vel)

                # 线速度：偏差大减速，贴线后恒速；最低保留 0.04 以克服起步阻力
                slow_factor = 1.0 - min(abs(en) / (0.30 if self.H is not None else 200.0), 0.85)
                linear_vel = self.base_linear_speed * slow_factor
                linear_vel = max(linear_vel, 0.04)
                linear_vel = np.clip(linear_vel, -self.max_linear_vel, self.max_linear_vel)

                # 发布速度
                cmd_vel = Twist()
                cmd_vel.linear.x = linear_vel
                cmd_vel.angular.z = angular_vel
                self.cmd_vel_pub.publish(cmd_vel)
                self.last_cmd_vel = cmd_vel

            else:
                # 识别不到牌子：立即停止
                cmd_vel = Twist()
                self.cmd_vel_pub.publish(cmd_vel)
                self.last_cmd_vel = cmd_vel

                # 舵机扫描
                if self.last_servo_time is None or (current_time - self.last_servo_time).to_sec() >= self.servo_scan_interval:
                    angle = self.servo_scan_angles[self.servo_scan_index]
                    servo_msg = Float32()
                    servo_msg.data = float(angle)
                    self.servo_angle_pub.publish(servo_msg)

                    # 更新索引
                    self.servo_scan_index = (self.servo_scan_index + 1) % len(self.servo_scan_angles)
                    self.last_servo_time = current_time

            rate.sleep()


def main():
    try:
        YOLOCarControl()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("收到键盘中断信号")
    finally:
        # 停止小车
        cmd_vel = Twist()
        rospy.Publisher("/yolo_car/cmd_vel", Twist, queue_size=1).publish(cmd_vel)


if __name__ == "__main__":
    main()

