#!/usr/bin/env python3
"""
camera_calibration.py - 相机内参标定脚本

功能：
1. 使用VisionLab标定板标定相机内参 (fx, fy, cx, cy)
2. 计算相机内参矩阵和畸变系数
3. 保存标定结果到YAML文件

使用方法：
1. 将VisionLab标定板放在不同位置和角度
2. 运行脚本：rosrun my_car_yolo camera_calibration.py
3. 按 'c' 键拍摄图片，按 's' 保存标定结果
4. 需要至少10张不同角度的标定板图片

VisionLab标定板参数：
- 型号: Q12-25-1.5
- 格子数: 12x9 (11x8个角点)
- 方格边长: 1.5mm
"""

import cv2
import numpy as np
import yaml
import os
import glob
from datetime import datetime


class CameraCalibrator:
    def __init__(self):
        # VisionLab标定板参数 (根据你提供的规格)
        self.board_size = (8, 11)  # (列数-1, 行数-1) = (8, 11) 角点
        self.square_size = 1.5     # 方格边长 1.5mm

        # 标定参数
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # 存储标定数据
        self.objpoints = []  # 世界坐标系中的3D点
        self.imgpoints = []  # 图像坐标系中的2D点
        self.images = []     # 保存的图像

        # 相机参数
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rvecs = None
        self.tvecs = None

        # 创建保存目录
        self.calib_dir = "calibration_images"
        if not os.path.exists(self.calib_dir):
            os.makedirs(self.calib_dir)

        print("相机内参标定初始化完成")
        print(f"标定板角点数: {self.board_size[0]}x{self.board_size[1]} = {self.board_size[0]*self.board_size[1]}")
        print(f"方格边长: {self.square_size}mm")
        print("按 'c' 键拍摄标定图片")
        print("按 's' 键开始标定计算")
        print("按 'q' 键退出")

    def prepare_object_points(self):
        """准备世界坐标系中的3D点"""
        objp = np.zeros((self.board_size[0] * self.board_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.board_size[0], 0:self.board_size[1]].T.reshape(-1, 2)
        objp *= self.square_size  # 乘以方格边长得到实际尺寸(mm)
        return objp

    def capture_image(self, frame):
        """处理单帧图像，检测角点"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 查找棋盘格角点
        ret, corners = cv2.findChessboardCorners(gray, self.board_size, None)

        if ret:
            # 精确化角点位置
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)

            # 绘制角点
            cv2.drawChessboardCorners(frame, self.board_size, corners2, ret)

            return True, corners2, gray
        else:
            return False, None, gray

    def save_calibration_image(self, frame, corners, image_count):
        """保存标定图片"""
        # 在图像上绘制信息
        info_text = f"Image {image_count}"
        cv2.putText(frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # 保存图像
        filename = f"{self.calib_dir}/calib_{image_count:02d}.png"
        cv2.imwrite(filename, frame)
        self.images.append(frame.copy())

        print(f"保存标定图片: {filename}")

    def calibrate_camera(self):
        """执行相机标定"""
        if len(self.objpoints) < 5:
            print(f"标定图片太少！当前: {len(self.objpoints)} 张，需要至少5张")
            return False

        print(f"开始标定，使用 {len(self.objpoints)} 张图片...")

        # 相机标定 - 获取图像尺寸 (width, height)
        image_size = (self.images[0].shape[1], self.images[0].shape[0])  # (width, height)

        ret, self.camera_matrix, self.dist_coeffs, self.rvecs, self.tvecs = cv2.calibrateCamera(
            self.objpoints, self.imgpoints, image_size, None, None
        )

        if ret:
            print("相机标定成功！")
            self.print_calibration_results()
            return True
        else:
            print("相机标定失败！")
            return False

    def print_calibration_results(self):
        """打印标定结果"""
        print("\n=== 相机标定结果 ===")
        print("相机内参矩阵:")
        print(self.camera_matrix)
        print(f"\n焦距 fx: {self.camera_matrix[0,0]:.2f}")
        print(f"焦距 fy: {self.camera_matrix[1,1]:.2f}")
        print(f"主点 cx: {self.camera_matrix[0,2]:.2f}")
        print(f"主点 cy: {self.camera_matrix[1,2]:.2f}")

        print(f"\n畸变系数: {self.dist_coeffs.ravel()}")

        # 计算重投影误差
        mean_error = 0
        for i in range(len(self.objpoints)):
            imgpoints2, _ = cv2.projectPoints(self.objpoints[i], self.rvecs[i], self.tvecs[i],
                                            self.camera_matrix, self.dist_coeffs)
            error = cv2.norm(self.imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            mean_error += error

        mean_error /= len(self.objpoints)
        print(f"平均重投影误差: {mean_error:.4f} 像素")

        if mean_error < 1.0:
            print("✓ 标定精度良好！")
        elif mean_error < 2.0:
            print("⚠ 标定精度一般，可以接受")
        else:
            print("✗ 标定精度较差，建议重新标定")

    def save_calibration(self, filename="camera_intrinsics.yaml"):
        """保存标定结果"""
        if self.camera_matrix is None:
            print("没有标定数据可以保存！")
            return

        calibration_data = {
            'calibration_date': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            'board_size': list(self.board_size),
            'square_size': self.square_size,
            'num_images': len(self.objpoints),
            'camera_matrix': {
                'fx': float(self.camera_matrix[0, 0]),
                'fy': float(self.camera_matrix[1, 1]),
                'cx': float(self.camera_matrix[0, 2]),
                'cy': float(self.camera_matrix[1, 2])
            },
            'distortion_coefficients': self.dist_coeffs.tolist(),
            'camera_matrix_full': self.camera_matrix.tolist()
        }

        with open(filename, 'w') as file:
            yaml.dump(calibration_data, file, default_flow_style=False)

        print(f"标定结果已保存到: {filename}")

    def run_calibration(self):
        """主标定循环"""
        cap = cv2.VideoCapture(0)  # 使用默认摄像头

        if not cap.isOpened():
            print("无法打开摄像头！")
            return

        image_count = 0

        print("开始相机标定...")
        print("将VisionLab标定板放在不同位置和角度，确保标定板完全在画面中")
        print("移动标定板到至少5个不同位置，每个位置拍摄一张图片")

        while True:
            ret, frame = cap.read()
            if not ret:
                print("无法读取摄像头画面")
                break

            # 检测角点
            found, corners, gray = self.capture_image(frame.copy())

            # 显示状态
            status_text = f"Images: {image_count}"
            if found:
                status_text += " - FOUND corners!"
                cv2.putText(frame, "Corners detected!", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            else:
                cv2.putText(frame, "No corners found", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            cv2.putText(frame, status_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            cv2.imshow("Camera Calibration", frame)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('c') and found:
                # 拍摄标定图片
                image_count += 1
                self.save_calibration_image(frame, corners, image_count)

                # 添加到标定数据
                objp = self.prepare_object_points()
                self.objpoints.append(objp)
                self.imgpoints.append(corners)

                print(f"添加标定图片 {image_count}")

            elif key == ord('s'):
                # 开始标定
                if self.calibrate_camera():
                    self.save_calibration()
                    print("标定完成！可以按 'q' 退出")
                else:
                    print("标定失败，请添加更多图片")

            elif key == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

        print(f"标定完成，共拍摄 {image_count} 张图片")


def main():
    try:
        calibrator = CameraCalibrator()
        calibrator.run_calibration()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        print("\n标定程序被用户中断")
    except Exception as e:
        cv2.destroyAllWindows()
        print(f"标定程序异常: {e}")


if __name__ == '__main__':
    main()
