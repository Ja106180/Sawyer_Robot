#!/usr/bin/env python3
"""
yolo_detection.py - YOLO物体检测节点

功能：
1. 使用YOLO模型检测摄像头中的小车标志
2. 在窗口上显示检测结果和目标点
3. 发布检测结果到 /my_car_yolo/detections 话题
"""

import cv2
import os
import time
import rospy
import rospkg
import numpy as np
from ultralytics import YOLO
from my_car_yolo.msg import ObjectDetections, ObjectDetection
from std_msgs.msg import Header


class YOLODetection:
    def __init__(self):
        rospy.init_node("yolo_detection", anonymous=False)

        # 获取ROS参数
        model_path = rospy.get_param("~model", None)

        # 处理空字符串的情况（launch文件可能传递空字符串）
        if model_path == "" or model_path is None:
            model_path = None

        # 如果没有指定模型路径，尝试从包内的models目录加载默认模型
        if model_path is None:
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('my_car_yolo')
            default_model = os.path.join(package_path, 'models', 'yolov8n_label.pt')

            if os.path.exists(default_model):
                model_path = default_model
                rospy.loginfo("使用默认模型: %s", model_path)
            else:
                rospy.logerr("未找到模型文件！")
                rospy.logerr("请将模型文件 'yolov8n_label.pt' 放到 %s 目录", os.path.join(package_path, 'models'))
                rospy.logerr("或者通过参数指定模型路径: _model:=path/to/model.pt")
                rospy.signal_shutdown("缺少模型文件")

        self.conf_threshold = float(rospy.get_param("~conf_threshold", 0.25))
        window_name = rospy.get_param("~window_name", "My Car YOLO Detection")
        source = int(rospy.get_param("~source", 0))  # 摄像头固定为0

        # 检查是否有GUI环境（此时还没创建窗口，不要访问 self.display_window）
        display_available = os.environ.get('DISPLAY', '') != ''

        if not display_available:
            # 在没有GUI的环境中，我们不加载YOLO模型，而是模拟检测
            self.model = None
            self.simulation_mode = True
            rospy.loginfo("GUI环境不可用，进入仿真模式")
        else:
            # 加载YOLO模型
            rospy.loginfo("加载YOLO模型: %s", model_path)
            try:
                self.model = YOLO(model_path)
                self.simulation_mode = False
            except Exception as e:
                rospy.logerr("加载模型失败: %s", e)
                # 如果加载失败，进入仿真模式
                self.model = None
                self.simulation_mode = True
                rospy.logwarn("模型加载失败，进入仿真模式")

        # 创建ROS发布者
        self.detections_pub = rospy.Publisher("/my_car_yolo/detections", ObjectDetections, queue_size=1)

        # 检查GUI环境
        self.display_window = False
        try:
            # 尝试创建一个测试窗口来检查GUI可用性
            test_window = "test_window_" + str(rospy.get_time())
            cv2.namedWindow(test_window, cv2.WINDOW_NORMAL)
            cv2.destroyWindow(test_window)
            self.display_window = True
            self.window_name = window_name
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            rospy.loginfo("GUI环境可用，创建检测窗口")
        except Exception as e:
            self.display_window = False
            self.window_name = window_name
            rospy.logwarn("GUI环境不可用，运行在服务器模式: %s", str(e))

        # 当前帧的检测结果 [(center_x, center_y, bbox)]
        self.current_frame_detections = []

        # 目标点 (只有一个A点)
        self.target_points = [
            (282, 127),  # A点（与ROS控制端保持一致）
        ]

        # 启动YOLO检测流
        rospy.loginfo("启动YOLO检测，使用摄像头 (source=0)")
        rospy.loginfo("按 'q' 键退出")

        try:
            while not rospy.is_shutdown():
                try:
                    if self.simulation_mode:
                        # 仿真模式：生成模拟的检测结果
                        time.sleep(0.1)  # 模拟处理时间
                        got_frame = True

                        # 创建模拟的检测结果
                        class MockResult:
                            def __init__(self):
                                self.boxes = None
                                self.orig_shape = (480, 640)

                        result = MockResult()
                        self.process_detection(result)
                        continue

                    got_frame = False
                    for result in self.model(source=source, stream=True, conf=self.conf_threshold, verbose=False):
                        got_frame = True
                        # 处理检测结果
                        self.process_detection(result)

                        # 显示图像（如果有窗口的话）
                        if self.display_window:
                            plotted = result.plot()
                            self.draw_target_points(plotted)
                            self.draw_status_info(plotted, result)
                            cv2.imshow(self.window_name, plotted)

                            # 检查退出
                            if cv2.waitKey(1) & 0xFF == ord('q'):
                                rospy.loginfo("收到退出信号")
                                rospy.signal_shutdown("用户退出")
                                break
                        else:
                            # 服务器模式：只输出检测信息
                            if self.current_frame_detections:
                                detection_info = []
                                for i, (center_x, center_y, _) in enumerate(self.current_frame_detections):
                                    detection_info.append(f"检测{i+1}: ({int(center_x)}, {int(center_y)})")
                                rospy.loginfo("检测结果: %s", ", ".join(detection_info))
                            else:
                                rospy.loginfo_throttle(5, "未检测到目标物体")

                    if not got_frame:
                        rospy.logwarn("未从摄像头获取到任何帧，1秒后重试...")
                        try:
                            if self.display_window:
                                blank = np.zeros((480, 640, 3), dtype=np.uint8)
                                cv2.putText(blank, "No frames from camera, retrying...", (20, 240),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)
                                cv2.imshow(self.window_name, blank)
                                cv2.waitKey(1)
                        except Exception:
                            pass
                        rospy.sleep(1.0)

                except Exception as e:
                    rospy.logerr("YOLO流异常: %s，1秒后重试", e)
                    try:
                        if self.display_window:
                            blank = np.zeros((480, 640, 3), dtype=np.uint8)
                            cv2.putText(blank, "Camera error, retrying...", (20, 240),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)
                            cv2.imshow(self.window_name, blank)
                            cv2.waitKey(1)
                    except Exception:
                        pass
                    rospy.sleep(1.0)
        except Exception as e:
            rospy.logerr("检测过程中出错: %s", e)
        finally:
            if self.display_window:
                cv2.destroyAllWindows()
            rospy.loginfo("YOLO检测节点已关闭")

    def process_detection(self, result):
        """处理检测结果并发布ROS消息"""
        msg = ObjectDetections()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "camera_frame"

        # 每帧先清空当前检测缓存
        self.current_frame_detections = []

        if result.boxes is None or len(result.boxes) == 0:
            # 没有检测到物体
            self.detections_pub.publish(msg)
            return

        # 获取图像尺寸
        if result.orig_shape:
            img_height, img_width = result.orig_shape
        else:
            img_height, img_width = result.plot().shape[:2]

        # 提取所有检测信息
        detections = []
        for box in result.boxes:
            bbox = box.xyxy[0].cpu().numpy()  # [x1, y1, x2, y2]
            x1, y1, x2, y2 = bbox
            center_x = (x1 + x2) / 2.0
            center_y = (y1 + y2) / 2.0
            confidence = float(box.conf[0].cpu().numpy()) if box.conf is not None else 0.0
            detections.append((center_x, center_y, (x1, y1, x2, y2), confidence, box))

        # 只保留前两个检测结果（按置信度排序，取最高的两个）
        if len(detections) > 2:
            detections.sort(key=lambda x: x[3], reverse=True)  # 按置信度降序排序
            detections = detections[:2]  # 只保留前两个

        # 保存当前帧的检测结果，供draw_labels使用（不再区分ID）
        self.current_frame_detections = []

        # 创建ROS消息（为兼容消息字段，仍填充 object_id，但不做跟踪）
        for idx, (center_x, center_y, (x1, y1, x2, y2), confidence, box) in enumerate(detections):
            self.current_frame_detections.append((center_x, center_y, (x1, y1, x2, y2)))

            obj_msg = ObjectDetection()
            obj_msg.header = msg.header
            obj_msg.object_id = idx + 1  # 简单序号
            obj_msg.center_x = float(center_x)
            obj_msg.center_y = float(center_y)
            obj_msg.confidence = confidence
            obj_msg.bbox_x = float(x1)
            obj_msg.bbox_y = float(y1)
            obj_msg.bbox_width = float(x2 - x1)
            obj_msg.bbox_height = float(y2 - y1)
            obj_msg.image_width = img_width
            obj_msg.image_height = img_height

            msg.objects.append(obj_msg)

        # 发布消息
        self.detections_pub.publish(msg)

    def draw_target_points(self, image):
        """在图像上绘制目标点"""
        # 只有一个A点，直接绘制
        x, y = self.target_points[0]
        color = (0, 255, 0)  # 绿色

        # 目标点用绿色小点
        cv2.circle(image, (int(x), int(y)), 3, color, -1)  # 实心小圆点
        # 添加简单的A标签
        cv2.putText(image, "A", (int(x) + 8, int(y) - 8),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    def draw_status_info(self, image, result):
        """绘制状态信息"""
        # 显示当前目标点信息
        target_x, target_y = self.target_points[0]  # 只有一个A点

        status_text = f"Target A: ({int(target_x)}, {int(target_y)}) | Conf > {self.conf_threshold}"
        cv2.putText(image, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                   0.7, (255, 255, 255), 2)

        # 显示检测到的标志物信息
        if self.current_frame_detections:
            detection_text = f"Detections: {len(self.current_frame_detections)}"
            cv2.putText(image, detection_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                       0.7, (0, 255, 0), 2)

            # 显示检测到的第一个标志物的中心点坐标
            if self.current_frame_detections:
                center_x, center_y, _ = self.current_frame_detections[0]
                coord_text = f"Car Center: ({int(center_x)}, {int(center_y)})"
                cv2.putText(image, coord_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX,
                           0.7, (255, 255, 0), 2)

                # 注意：不再做“A点在框内=到达”的视觉判定，避免误导调参/测试结论
        else:
            cv2.putText(image, "No Detections", (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                       0.7, (0, 0, 255), 2)


def main():
    try:
        node = YOLODetection()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("收到键盘中断信号")
    finally:
        try:
            cv2.destroyAllWindows()
        except:
            pass


if __name__ == "__main__":
    main()
