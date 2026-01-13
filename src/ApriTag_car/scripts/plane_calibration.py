#!/usr/bin/env python3
"""
plane_calibration.py - 平面坐标映射标定脚本

功能：
1. 手动把 AprilTag 放到 4 个已知平面坐标点
2. 记录每个位置时 tag 在相机坐标系下的位姿
3. 计算单应性矩阵 H，将相机坐标映射到平面坐标
4. 保存 H 矩阵到 YAML 文件

坐标系约定：
- 平面坐标原点 = 图像左上角
- X 轴 = 图像向右
- Y 轴 = 图像向下
"""

import rospy
import numpy as np
import cv2
import yaml
import os
import rospkg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class PlaneCalibrator:
    def __init__(self):
        rospy.init_node("plane_calibration", anonymous=False)
        
        # 标定点对
        self.calibration_pairs = []  # [(plane_x, plane_y, camera_x, camera_y), ...]
        
        # 已知平面坐标点（4个点，按顺序标定）
        # 你可以修改这4个点的坐标，但要确保它们在地面上能放得下
        self.plane_points = [
            (0.0, 0.0),      # 原点（图像左上角对应）
            (0.5, 0.0),      # X+ 方向 50cm
            (0.5, 0.5),      # X+ Y+ 方向 50cm
            (0.0, 0.5),      # Y+ 方向 50cm
        ]
        
        self.current_point_idx = 0
        
        # 订阅 tag 位姿
        self.pose_sub = rospy.Subscriber("/apriltag_car/tag_pose", PoseStamped, 
                                        self.pose_callback)
        
        # 当前位姿缓存
        self.current_pose = None
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("平面坐标映射标定程序")
        rospy.loginfo("=" * 60)
        rospy.loginfo("请按以下步骤操作：")
        rospy.loginfo("1. 将 AprilTag 放在已知平面坐标点上")
        rospy.loginfo("2. 按空格键记录当前点对")
        rospy.loginfo("3. 重复步骤1-2，直到记录完4个点")
        rospy.loginfo("4. 按 's' 键保存标定结果")
        rospy.loginfo("5. 按 'q' 键退出")
        rospy.loginfo("=" * 60)
        self.print_next_point()
    
    def print_next_point(self):
        """打印下一个要标定的点"""
        if self.current_point_idx < len(self.plane_points):
            px, py = self.plane_points[self.current_point_idx]
            rospy.loginfo("")
            rospy.loginfo(">>> 请将 tag 放到平面坐标点 (%.2f, %.2f) 米处", px, py)
            rospy.loginfo(">>> 然后按空格键记录")
        else:
            rospy.loginfo("")
            rospy.loginfo(">>> 所有点已记录完成！按 's' 保存标定结果")
    
    def pose_callback(self, msg):
        """Tag 位姿回调"""
        self.current_pose = msg
    
    def record_point(self):
        """记录当前点对"""
        if self.current_pose is None:
            rospy.logwarn("未收到 tag 位姿数据，请确保 tag 在相机视野内")
            return False
        
        if self.current_point_idx >= len(self.plane_points):
            rospy.logwarn("所有点已记录完成")
            return False
        
        # 获取平面坐标
        plane_x, plane_y = self.plane_points[self.current_point_idx]
        
        # 获取相机坐标（只取 X, Y，忽略 Z）
        camera_x = self.current_pose.pose.position.x
        camera_y = self.current_pose.pose.position.y
        
        # 记录点对
        self.calibration_pairs.append((plane_x, plane_y, camera_x, camera_y))
        
        rospy.loginfo("已记录点 %d: 平面(%.3f, %.3f) <-> 相机(%.3f, %.3f)",
                     self.current_point_idx + 1, plane_x, plane_y, camera_x, camera_y)
        
        self.current_point_idx += 1
        self.print_next_point()
        
        return True
    
    def compute_homography(self):
        """计算单应性矩阵 H"""
        if len(self.calibration_pairs) < 4:
            rospy.logerr("点对数量不足！需要至少4个点，当前: %d", len(self.calibration_pairs))
            return None
        
        # 准备数据
        plane_points = np.array([[p[0], p[1]] for p in self.calibration_pairs], dtype=np.float32)
        camera_points = np.array([[p[2], p[3]] for p in self.calibration_pairs], dtype=np.float32)
        
        # 使用 OpenCV 计算单应性矩阵
        H, mask = cv2.findHomography(camera_points, plane_points, cv2.RANSAC, 5.0)
        
        if H is None:
            rospy.logerr("计算单应性矩阵失败！")
            return None
        
        rospy.loginfo("单应性矩阵 H 计算成功:")
        rospy.loginfo("\n%s", H)
        
        return H
    
    def save_calibration(self, H):
        """保存标定结果"""
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('ApriTag_car')
        config_dir = os.path.join(package_path, 'config')
        
        if not os.path.exists(config_dir):
            os.makedirs(config_dir)
        
        output_path = os.path.join(config_dir, 'plane_calibration.yaml')
        
        calibration_data = {
            'calibration_date': rospy.get_time(),
            'num_points': len(self.calibration_pairs),
            'calibration_pairs': [
                {
                    'plane': [float(p[0]), float(p[1])],
                    'camera': [float(p[2]), float(p[3])]
                }
                for p in self.calibration_pairs
            ],
            'homography_matrix': H.tolist()
        }
        
        with open(output_path, 'w') as f:
            yaml.dump(calibration_data, f, default_flow_style=False)
        
        rospy.loginfo("标定结果已保存到: %s", output_path)
        return output_path
    
    def run(self):
        """主循环"""
        import sys
        import select
        import termios
        import tty
        
        # 设置终端为非阻塞模式
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        
        rospy.loginfo("标定程序已启动，等待键盘输入...")
        
        try:
            while not rospy.is_shutdown():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    
                    if key == ' ':  # 空格键：记录当前点
                        self.record_point()
                    
                    elif key == 's':  # s 键：保存标定
                        if len(self.calibration_pairs) < 4:
                            rospy.logwarn("点对数量不足！需要4个点，当前: %d", len(self.calibration_pairs))
                        else:
                            H = self.compute_homography()
                            if H is not None:
                                self.save_calibration(H)
                                rospy.loginfo("标定完成！可以按 'q' 退出")
                    
                    elif key == 'q':  # q 键：退出
                        break
                
                rospy.sleep(0.1)
        
        finally:
            # 恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            rospy.loginfo("标定程序已退出")


def main():
    try:
        calibrator = PlaneCalibrator()
        calibrator.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("标定程序异常: %s", e)


if __name__ == "__main__":
    main()

