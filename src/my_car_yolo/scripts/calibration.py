#!/usr/bin/env python3
"""
calibration.py - 手眼标定脚本

功能：
1. 采集标定数据：同时记录相机像素坐标和机械臂世界坐标
2. 计算单应矩阵：像素坐标系到世界坐标系的变换
3. 保存标定结果：输出YAML配置文件供运行时使用

使用方法：
1. 将标定板放在机械臂可达范围内
2. 运行脚本：rosrun my_car_yolo calibration.py
3. 移动机械臂到不同位置，脚本会自动采集数据点
4. 采集足够点后，计算并保存标定矩阵

参数：
- min_points: 最少标定点数 (默认8)
- save_path: 标定文件保存路径
"""

import rospy
import cv2
import numpy as np
import yaml
import os
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
import tf2_ros
import tf2_geometry_msgs


class HandEyeCalibration:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('hand_eye_calibration', anonymous=False)

        # 获取参数
        self.min_points = rospy.get_param('~min_points', 8)              # 最少标定点数
        self.save_path = rospy.get_param('~save_path', '')               # 保存路径
        self.camera_topic = rospy.get_param('~camera_topic', '/usb_cam/image_raw')

        # 如果没有指定保存路径，使用默认路径
        if not self.save_path:
            pkg_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            self.save_path = os.path.join(pkg_path, 'config', 'calibration.yaml')

        # 标定数据存储
        self.pixel_points = []    # 像素坐标点 [(u, v), ...]
        self.world_points = []    # 世界坐标点 [(x, y), ...]

        # ROS接口
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(self.camera_topic, Image, self.image_callback)
        self.pose_sub = None  # 稍后设置

        # TF监听器 (用于获取机械臂末端位姿)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 状态
        self.collecting = False
        self.calibrated = False

        # 创建保存目录
        save_dir = os.path.dirname(self.save_path)
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)

        rospy.loginfo("手眼标定节点已启动")
        rospy.loginfo("标定文件保存路径: %s", self.save_path)
        rospy.loginfo("最少标定点数: %d", self.min_points)
        rospy.loginfo("请将标定板放在合适位置，然后按 'c' 开始采集数据")
        rospy.loginfo("移动机械臂到不同位置，脚本会自动记录对应点")
        rospy.loginfo("采集足够点数后按 's' 保存标定结果")

    def image_callback(self, msg):
        """图像回调函数"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
            return

        # 显示图像和采集状态
        display_image = cv_image.copy()

        # 显示采集的点
        for i, (u, v) in enumerate(self.pixel_points):
            cv2.circle(display_image, (int(u), int(v)), 5, (0, 255, 0), -1)
            cv2.putText(display_image, str(i+1), (int(u)+10, int(v)-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # 显示状态信息
        status_text = f"Points: {len(self.pixel_points)}/{self.min_points}"
        if self.calibrated:
            status_text += " (Calibrated)"
        elif self.collecting:
            status_text += " (Collecting...)"

        cv2.putText(display_image, status_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)

        cv2.imshow("Hand-Eye Calibration", display_image)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('c'):  # 开始采集
            self.start_collection()
        elif key == ord('s'):  # 保存标定
            if len(self.pixel_points) >= self.min_points:
                self.compute_calibration()
                self.save_calibration()
            else:
                rospy.logwarn("标定点数不足，需要至少 %d 个点", self.min_points)
        elif key == ord('r'):  # 重置
            self.reset_calibration()
        elif key == ord('q'):  # 退出
            rospy.signal_shutdown("用户退出")

    def start_collection(self):
        """开始采集标定数据"""
        if self.collecting:
            rospy.loginfo("已经在采集数据了")
            return

        self.collecting = True
        self.pose_sub = rospy.Subscriber('/endpoint_state', PoseStamped, self.pose_callback)
        rospy.loginfo("开始采集标定数据...")
        rospy.loginfo("移动机械臂到标定板上不同位置，系统会自动记录对应点")

    def pose_callback(self, msg):
        """机械臂位姿回调"""
        if not self.collecting:
            return

        try:
            # 获取当前图像中心作为像素点 (简化版本)
            # 实际应用中应该检测标定板特征点
            pixel_u = 320  # 图像中心X
            pixel_v = 240  # 图像中心Y

            # 获取世界坐标
            world_x = msg.pose.position.x
            world_y = msg.pose.position.y

            # 记录数据点
            self.pixel_points.append((pixel_u, pixel_v))
            self.world_points.append((world_x, world_y))

            point_idx = len(self.pixel_points)
            rospy.loginfo("记录标定点 %d: 像素(%.1f, %.1f) -> 世界(%.3f, %.3f)",
                         point_idx, pixel_u, pixel_v, world_x, world_y)

            # 检查是否采集够点数
            if len(self.pixel_points) >= self.min_points:
                self.collecting = False
                if self.pose_sub:
                    self.pose_sub.unregister()
                self.compute_calibration()

        except Exception as e:
            rospy.logerr("记录标定点失败: %s", str(e))

    def compute_calibration(self):
        """计算单应矩阵"""
        if len(self.pixel_points) < 4:
            rospy.logwarn("标定点数太少，至少需要4个点")
            return

        # 转换为numpy数组
        pixel_array = np.array(self.pixel_points, dtype=np.float32)
        world_array = np.array(self.world_points, dtype=np.float32)

        # 计算单应矩阵
        self.homography_matrix, status = cv2.findHomography(pixel_array, world_array, cv2.RANSAC, 5.0)

        if self.homography_matrix is not None and status.sum() >= 4:
            self.calibrated = True
            rospy.loginfo("标定计算成功！")
            self.print_matrix()
        else:
            rospy.logerr("标定计算失败，请检查数据点")
            self.calibrated = False

    def print_matrix(self):
        """打印单应矩阵"""
        if not self.calibrated:
            return

        matrix = self.homography_matrix
        rospy.loginfo("单应矩阵:")
        for i in range(3):
            rospy.loginfo("  [%.6f, %.6f, %.6f]",
                         matrix[i, 0], matrix[i, 1], matrix[i, 2])

    def save_calibration(self):
        """保存标定结果到YAML文件"""
        if not self.calibrated:
            rospy.logwarn("还未完成标定，无法保存")
            return

        try:
            # 准备YAML数据
            calibration_data = {
                'homography_matrix': self.homography_matrix.tolist(),
                'calibration_date': str(rospy.Time.now()),
                'num_points': len(self.pixel_points),
                'pixel_points': self.pixel_points,
                'world_points': self.world_points
            }

            # 保存到文件
            with open(self.save_path, 'w') as file:
                yaml.dump(calibration_data, file, default_flow_style=False)

            rospy.loginfo("标定结果已保存到: %s", self.save_path)

        except Exception as e:
            rospy.logerr("保存标定文件失败: %s", str(e))

    def reset_calibration(self):
        """重置标定数据"""
        self.pixel_points.clear()
        self.world_points.clear()
        self.calibrated = False
        self.collecting = False

        if self.pose_sub:
            self.pose_sub.unregister()
            self.pose_sub = None

        rospy.loginfo("标定数据已重置")

    def test_calibration(self, test_pixel_u, test_pixel_v):
        """测试标定结果"""
        if not self.calibrated:
            return None

        # 测试点变换
        test_point = np.array([[test_pixel_u], [test_pixel_v], [1.0]], dtype=np.float32)
        result = self.homography_matrix @ test_point

        if result[2, 0] != 0:
            world_x = result[0, 0] / result[2, 0]
            world_y = result[1, 0] / result[2, 0]
            return (world_x, world_y)

        return None


def main():
    try:
        calibrator = HandEyeCalibration()
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        rospy.loginfo("标定程序已退出")
    except Exception as e:
        cv2.destroyAllWindows()
        rospy.logerr("标定程序异常: %s", str(e))


if __name__ == '__main__':
    main()
