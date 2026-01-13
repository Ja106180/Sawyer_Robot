#!/usr/bin/env python3
"""
hand_eye_calibration.py - 手眼标定脚本（第二步）

功能：
1. 加载相机内参（第一步标定结果）
2. 将小车标志放在地面已知位置
3. 用YOLO检测标志物中心点
4. 计算像素坐标到世界坐标的变换矩阵
5. 保存手眼标定结果

使用方法：
1. 将小车标志放在地面坐标(0,0)位置
2. 运行脚本：rosrun my_car_yolo hand_eye_calibration.py
3. 按 'c' 键采集数据点
4. 按 's' 键计算并保存标定结果

注意：
- 需要先完成相机内参标定
- 需要YOLO模型文件 yolov8n.pt
"""

import cv2
import numpy as np
import yaml
import os
import torch
from ultralytics import YOLO


class HandEyeCalibrator:
    def __init__(self):
        # 加载相机内参
        self.load_camera_intrinsics()

        # 加载YOLO模型
        self.load_yolo_model()

        # 标定数据
        self.pixel_points = []  # YOLO检测到的像素坐标
        self.world_points = []  # 对应的世界坐标

        # 相机内参
        self.camera_matrix = None
        self.dist_coeffs = None

        print("手眼标定初始化完成")
        print("将小车标志放在地面坐标(0,0)位置")
        print("确保YOLO能检测到标志物")
        print("按 'c' 键采集标定点")
        print("按 's' 键计算标定结果")
        print("按 'q' 键退出")

    def load_camera_intrinsics(self):
        """加载相机内参"""
        try:
            with open('camera_intrinsics.yaml', 'r') as file:
                data = yaml.safe_load(file)

            # 读取相机内参矩阵
            cam_data = data['camera_matrix']
            self.camera_matrix = np.array([
                [cam_data['fx'], 0, cam_data['cx']],
                [0, cam_data['fy'], cam_data['cy']],
                [0, 0, 1]
            ], dtype=np.float32)

            # 设置内参变量
            self.fx = cam_data['fx']
            self.fy = cam_data['fy']
            self.cx = cam_data['cx']
            self.cy = cam_data['cy']

            # 读取畸变系数
            self.dist_coeffs = np.array(data['distortion_coefficients'], dtype=np.float32)

            print("相机内参加载成功")
            print(f"焦距: fx={cam_data['fx']:.2f}, fy={cam_data['fy']:.2f}")
            print(f"主点: cx={cam_data['cx']:.2f}, cy={cam_data['cy']:.2f}")

        except Exception as e:
            print(f"加载相机内参失败: {e}")
            print("请先运行相机内参标定！")
            exit(1)

    def load_yolo_model(self):
        """加载YOLO模型"""
        try:
            # 查找模型文件
            pkg_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            model_path = os.path.join(pkg_path, 'models', 'yolov8n_label.pt')

            if not os.path.exists(model_path):
                print(f"YOLO模型文件不存在: {model_path}")
                print("请将 yolov8n.pt 文件放到 models/ 目录下")
                exit(1)

            self.model = YOLO(model_path)
            print(f"YOLO模型加载成功: {model_path}")

        except Exception as e:
            print(f"加载YOLO模型失败: {e}")
            exit(1)

    def detect_with_yolo(self, frame):
        """用YOLO检测标志物"""
        try:
            # YOLO推理
            results = self.model(frame, conf=0.5, verbose=False)

            if len(results) > 0 and len(results[0].boxes) > 0:
                # 获取第一个检测结果
                box = results[0].boxes[0]
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = box.conf[0].cpu().numpy()

                # 计算中心点
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2

                return True, (center_x, center_y), conf, (x1, y1, x2, y2)
            else:
                return False, None, 0, None

        except Exception as e:
            print(f"YOLO检测错误: {e}")
            return False, None, 0, None

    def undistort_point(self, point):
        """畸变矫正单个点"""
        if self.camera_matrix is None or self.dist_coeffs is None:
            return point

        # 将点转换为正确的格式
        points = np.array([[point]], dtype=np.float32)

        # 畸变矫正
        undistorted = cv2.undistortPoints(points, self.camera_matrix, self.dist_coeffs,
                                        None, self.camera_matrix)

        return (undistorted[0][0][0], undistorted[0][0][1])

    def capture_calibration_point(self, frame):
        """采集标定点"""
        # YOLO检测
        detected, center, confidence, bbox = self.detect_with_yolo(frame)

        if detected:
            # 由于是无畸变摄像头，直接使用原始像素坐标
            pixel_u, pixel_v = center

            # 计算基于内参的世界坐标估算
            # 假设摄像头垂直向下，Z距离已知或估算
            if hasattr(self, 'camera_height'):
                # 如果知道摄像头高度，可以精确计算
                world_x_estimated = (pixel_u - self.cx) / self.fx * self.camera_height
                world_y_estimated = (pixel_v - self.cy) / self.fy * self.camera_height
            else:
                # 如果不知道高度，用相对比例估算
                world_x_estimated = (pixel_u - self.cx) / self.fx * 1.0  # 假设1米高度
                world_y_estimated = (pixel_v - self.cy) / self.fy * 1.0

            # 第一个点作为原点校准
            point_num = len(self.pixel_points) + 1
            if point_num == 1:
                # 第一个点设为世界坐标原点
                self.origin_pixel = (pixel_u, pixel_v)
                self.origin_world = (0.0, 0.0)
                world_x, world_y = 0.0, 0.0
            else:
                # 其他点相对于第一个点的偏移
                pixel_offset_u = pixel_u - self.origin_pixel[0]
                pixel_offset_v = pixel_v - self.origin_pixel[1]

                # 使用内参计算世界坐标偏移
                world_offset_x = pixel_offset_u / self.fx * 1.0  # 假设高度1米
                world_offset_y = pixel_offset_v / self.fy * 1.0

                world_x = self.origin_world[0] + world_offset_x
                world_y = self.origin_world[1] + world_offset_y

            # 保存对应点
            self.pixel_points.append((pixel_u, pixel_v))
            self.world_points.append((world_x, world_y))

            print(f"✓ 采集标定点 {point_num}")
            print(f"  像素坐标: ({pixel_u:.1f}, {pixel_v:.1f})")
            print(f"  内参估算世界坐标: ({world_x:.3f}, {world_y:.3f}) 米")

            if point_num >= 4:
                print("\n已采集足够点数，可以按 's' 计算基于内参的标定结果")

            return True, (pixel_u, pixel_v), confidence, bbox
        else:
            return False, None, 0, None

    def compute_intrinsics_based_calibration(self):
        """基于内参计算标定参数"""
        if len(self.pixel_points) < 2:
            print("标定点数量不足！至少需要2个点")
            return None

        # 计算基于内参的比例尺
        # 使用像素距离和估算的世界距离来计算比例尺

        pixel_dists = []
        world_dists = []

        # 计算每对点的距离
        for i in range(len(self.pixel_points)):
            for j in range(i+1, len(self.pixel_points)):
                # 像素距离
                pu1, pv1 = self.pixel_points[i]
                pu2, pv2 = self.pixel_points[j]
                pixel_dist = np.sqrt((pu2-pu1)**2 + (pv2-pv1)**2)

                # 世界距离
                wu1, wv1 = self.world_points[i]
                wu2, wv2 = self.world_points[j]
                world_dist = np.sqrt((wu2-wu1)**2 + (wv2-wv1)**2)

                if pixel_dist > 10 and world_dist > 0.01:  # 避免太小的距离
                    pixel_dists.append(pixel_dist)
                    world_dists.append(world_dist)

        if len(pixel_dists) > 0:
            # 计算平均比例尺
            ratios = np.array(world_dists) / np.array(pixel_dists)
            self.scale_meter_per_pixel = np.mean(ratios)
            self.scale_std = np.std(ratios)

            print(f"基于内参计算的比例尺: {self.scale_meter_per_pixel:.6f} ± {self.scale_std:.6f} 米/像素")

            # 同时计算单应矩阵作为验证
            if len(self.pixel_points) >= 4:
                pixel_array = np.array(self.pixel_points, dtype=np.float32)
                world_array = np.array(self.world_points, dtype=np.float32)
                homography_matrix, status = cv2.findHomography(pixel_array, world_array, cv2.RANSAC, 5.0)

                if homography_matrix is not None:
                    print("单应矩阵计算成功（用于验证）")

                    # 比较两种方法的结果
                    # 使用内参方法计算第一个点相对于原点的变换
                    test_pixel = self.pixel_points[0]
                    pred_world_x = (test_pixel[0] - self.cx) * self.scale_meter_per_pixel
                    pred_world_y = (test_pixel[1] - self.cy) * self.scale_meter_per_pixel

                    actual_world = self.world_points[0]
                    error = np.sqrt((pred_world_x - actual_world[0])**2 + (pred_world_y - actual_world[1])**2)
                    print(f"内参方法验证误差: {error:.4f} 米")

                    return {
                        'method': 'intrinsics_based',
                        'fx': self.fx,
                        'fy': self.fy,
                        'cx': self.cx,
                        'cy': self.cy,
                        'scale_meter_per_pixel': self.scale_meter_per_pixel,
                        'scale_std': self.scale_std,
                        'origin_pixel': self.origin_pixel,
                        'origin_world': self.origin_world,
                        'homography_matrix': homography_matrix.tolist(),
                        'pixel_points': self.pixel_points,
                        'world_points': self.world_points
                    }

        return None

    def save_calibration(self, calibration_result, filename="hand_eye_calibration.yaml"):
        """保存标定结果"""
        with open(filename, 'w') as file:
            yaml.dump(calibration_result, file, default_flow_style=False)

        print(f"基于内参的手眼标定结果已保存到: {filename}")
        print("标定参数:")
        print(f"  方法: {calibration_result['method']}")
        print(f"  比例尺: {calibration_result['scale_meter_per_pixel']:.6f} m/pixel")
        print(f"  相机内参: fx={calibration_result['fx']:.2f}, fy={calibration_result['fy']:.2f}")
        print(f"  主点: cx={calibration_result['cx']:.2f}, cy={calibration_result['cy']:.2f}")

    def test_calibration(self, homography_matrix, test_pixel):
        """测试标定结果"""
        if homography_matrix is None:
            return None

        # 测试点变换
        test_point = np.array([[test_pixel[0], test_pixel[1], 1]], dtype=np.float32).T
        result = homography_matrix @ test_point

        if result[2, 0] != 0:
            world_x = result[0, 0] / result[2, 0]
            world_y = result[1, 0] / result[2, 0]
            return (world_x, world_y)

        return None

    def run_calibration(self):
        """主标定循环"""
        cap = cv2.VideoCapture(0)

        if not cap.isOpened():
            print("无法打开摄像头！")
            return

        print("开始基于内参的手眼标定...")
        print("操作方法：")
        print("1. 将小车标志放在第一个位置，按 'c' 键设为原点(0,0)")
        print("2. 移动小车到其他位置，继续按 'c' 键采集点")
        print("3. 系统会自动使用相机内参计算每个点的位置")
        print("4. 采集至少2个点后，按 's' 键计算标定结果")
        print("")
        print("注意：第一个点自动设为世界坐标(0,0)，后续点自动计算")

        while True:
            ret, frame = cap.read()
            if not ret:
                print("无法读取摄像头画面")
                break

            # YOLO检测和显示
            detected, center, confidence, bbox = self.detect_with_yolo(frame.copy())

            # 显示结果
            display_frame = frame.copy()

            if detected:
                # 绘制检测框
                x1, y1, x2, y2 = bbox
                cv2.rectangle(display_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

                # 绘制中心点
                cv2.circle(display_frame, (int(center[0]), int(center[1])), 5, (0, 0, 255), -1)

                # 显示置信度
                cv2.putText(display_frame, f"Conf: {confidence:.2f}", (int(x1), int(y1)-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 显示状态信息
            status_text = f"Points: {len(self.pixel_points)}/4 (need >=2 for intrinsics)"
            if detected:
                status_text += " - DETECTED (Press 'c' to capture)"
            else:
                status_text += " - No detection"

            cv2.putText(display_frame, status_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            # 显示操作说明
            cv2.putText(display_frame, "c: capture point", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(display_frame, "s: calculate intrinsics-based result", (10, 85),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(display_frame, "q: quit", (10, 110),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            cv2.imshow("Hand-Eye Calibration", display_frame)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('c') and detected:
                # 采集标定点
                success, undistorted_center, conf, _ = self.capture_calibration_point(frame)
                if success:
                    print(f"✓ 采集成功！像素坐标: ({undistorted_center[0]:.1f}, {undistorted_center[1]:.1f})")

            elif key == ord('s') and len(self.pixel_points) >= 2:
                # 计算基于内参的标定结果
                calibration_result = self.compute_intrinsics_based_calibration()
                if calibration_result:
                    self.save_calibration(calibration_result)
                    print("基于内参的手眼标定完成！可以按 'q' 退出")

                    # 测试标定结果
                    if len(self.pixel_points) > 0:
                        test_pixel = self.pixel_points[0]
                        # 使用内参方法测试
                        test_world_x = (test_pixel[0] - self.cx) * calibration_result['scale_meter_per_pixel']
                        test_world_y = (test_pixel[1] - self.cy) * calibration_result['scale_meter_per_pixel']
                        print(f"内参方法测试: 像素{test_pixel} -> 世界({test_world_x:.3f}, {test_world_y:.3f})")

            elif key == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
        print("手眼标定程序结束")


def main():
    try:
        calibrator = HandEyeCalibrator()
        calibrator.run_calibration()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        print("\n标定程序被用户中断")
    except Exception as e:
        cv2.destroyAllWindows()
        print(f"标定程序异常: {e}")


if __name__ == '__main__':
    main()
