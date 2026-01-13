#!/usr/bin/env python3
"""
yolo_car_detection.py - YOLO物体检测节点

功能：
1. 使用YOLO模型检测USB摄像头中的物体
2. 在窗口上显示检测结果，标注物体ID（Object 1, Object 2...）和中心点坐标（label1, label2...）
3. 发布检测结果到 /yolo_car/detections 话题

使用方法：
    rosrun yolo_car yolo_car_detection.py
    或通过launch文件启动
"""

import cv2
import os
import rospy
import rospkg
import numpy as np
from ultralytics import YOLO
from yolo_car.msg import ObjectDetections, ObjectDetection
from std_msgs.msg import Header
from geometry_msgs.msg import Point


class YOLOCarDetection:
    def __init__(self):
        rospy.init_node("yolo_car_detection", anonymous=False)
        
        # 获取ROS参数
        model_path = rospy.get_param("~model", None)
        
        # 处理空字符串的情况（launch文件可能传递空字符串）
        if model_path == "" or model_path is None:
            model_path = None
        
        # 如果没有指定模型路径，尝试从包内的models目录加载默认模型
        if model_path is None:
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('yolo_car')
            default_model = os.path.join(package_path, 'models', 'yolov8n_label.pt')
            
            if os.path.exists(default_model):
                model_path = default_model
                rospy.loginfo("使用默认模型: %s", model_path)
            else:
                rospy.logerr("未找到模型文件！")
                rospy.logerr("请将模型文件 'yolov8n_label.pt' 放到 %s 目录", os.path.join(package_path, 'models'))
                rospy.logerr("或者通过参数指定模型路径: _model:=path/to/model.pt")
                rospy.signal_shutdown("缺少模型文件")
        
        conf_threshold = float(rospy.get_param("~conf_threshold", 0.25))
        window_name = rospy.get_param("~window_name", "YOLO Car Detection")
        source = int(rospy.get_param("~source", 0))  # USB摄像头固定为0
        
        # 加载YOLO模型
        rospy.loginfo("加载YOLO模型: %s", model_path)
        try:
            self.model = YOLO(model_path)
        except Exception as e:
            rospy.logerr("加载模型失败: %s", e)
            rospy.signal_shutdown("模型加载失败")
            return
        
        # 创建ROS发布者
        self.detections_pub = rospy.Publisher("/yolo_car/detections", ObjectDetections, queue_size=1)
        self.target_point_pub = rospy.Publisher("/yolo_car/target_point", Point, queue_size=1)
        
        # 初始化窗口
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        self.window_name = window_name
        
        # 当前帧的检测结果 [(center_x, center_y, bbox)]
        self.current_frame_detections = []

        # 动态叠加点的路径（顺时针四个顶点）
        self.path_vertices = [
            (103, 351),
            (413, 363),
            (367, 179),
            (187, 173),
        ]
        self.edge_lengths = []
        self.path_perimeter = 0.0
        self._compute_path_lengths()

        # 运动参数：30秒一圈，相隔半圈
        self.point_speed = self.path_perimeter / 30.0 if self.path_perimeter > 0 else 0.0
        self.point_phase_offset = self.path_perimeter / 2.0 if self.path_perimeter > 0 else 0.0
        self.path_dist = 0.0
        self.last_time = None
        
        # 启动YOLO检测流
        rospy.loginfo("启动YOLO检测，使用USB摄像头 (source=0)")
        rospy.loginfo("按 'q' 键退出")
        
        try:
            while not rospy.is_shutdown():
                try:
                    got_frame = False
                    for result in self.model(source=source, stream=True, conf=conf_threshold):
                        got_frame = True
                        if rospy.is_shutdown():
                            break

                        # 处理检测结果
                        self.process_detection(result)

                        # 显示图像
                        plotted = result.plot()

                        # 绘制叠加动画（不影响检测）
                        self.draw_overlay(plotted)

                        # 在图像上添加中心点坐标标注
                        self.draw_labels(plotted, result)

                        cv2.imshow(self.window_name, plotted)

                        # 检查退出
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            rospy.loginfo("收到退出信号")
                            rospy.signal_shutdown("用户退出")
                            break

                    if not got_frame:
                        rospy.logwarn("未从摄像头获取到任何帧，1秒后重试...")
                        try:
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
            cv2.destroyAllWindows()
            rospy.loginfo("YOLO检测节点已关闭")
    
    def process_detection(self, result):
        """处理检测结果并发布ROS消息"""
        msg = ObjectDetections()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "camera_frame"

        # 每帧先清空当前检测缓存，避免上一帧残留
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
    
    def draw_labels(self, image, result):
        """在图像上绘制中心点坐标标签（仅坐标，无ID）"""
        if not self.current_frame_detections:
            return
        
        # 直接使用当前帧保存的检测结果（最多两个）
        for center_x, center_y, (x1, y1, x2, y2) in self.current_frame_detections:
            center_x_int = int(center_x)
            center_y_int = int(center_y)
            
            # 只显示坐标，不显示label和ID
            coord_text = f"({center_x_int}, {center_y_int})"
            
            # 获取文本大小
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.6
            thickness = 2
            
            # 计算文本位置
            (text_width_coord, text_height_coord), _ = cv2.getTextSize(coord_text, font, font_scale, thickness)
            
            # 在检测框上方绘制标签，往上移更多避免遮挡YOLO的置信度显示
            text_x = int(x1)
            text_y_coord = int(y1) - text_height_coord - 11  # 往上移更多，留出空间给置信度
            
            # 确保文本不超出图像边界
            if text_y_coord < text_height_coord:
                text_y_coord = int(y2) + text_height_coord + 5
            
            # 绘制背景矩形（提高可读性）
            cv2.rectangle(image, 
                         (text_x - 2, text_y_coord - text_height_coord - 2),
                         (text_x + text_width_coord + 2, text_y_coord + 2),
                         (0, 0, 0), -1)
            
            # 绘制坐标文本
            cv2.putText(image, coord_text, (text_x, text_y_coord), 
                       font, font_scale, (255, 255, 0), thickness)
            
            # 在中心点绘制标记
            cv2.circle(image, (center_x_int, center_y_int), 5, (0, 0, 255), -1)
            cv2.circle(image, (center_x_int, center_y_int), 8, (0, 0, 255), 2)
            
    def _compute_path_lengths(self):
        """预计算路径各边长度和总周长"""
        self.edge_lengths = []
        self.path_perimeter = 0.0
        n = len(self.path_vertices)
        if n < 2:
            return
        for i in range(n):
            x1, y1 = self.path_vertices[i]
            x2, y2 = self.path_vertices[(i + 1) % n]
            length = np.hypot(x2 - x1, y2 - y1)
            self.edge_lengths.append(length)
            self.path_perimeter += length

    def _point_at_distance(self, dist):
        """根据沿路径的距离获取插值点坐标"""
        if self.path_perimeter <= 0 or not self.edge_lengths:
            return (0, 0)
        d = dist % self.path_perimeter
        n = len(self.path_vertices)
        for i, edge_len in enumerate(self.edge_lengths):
            if d <= edge_len:
                x1, y1 = self.path_vertices[i]
                x2, y2 = self.path_vertices[(i + 1) % n]
                t = d / edge_len if edge_len > 0 else 0.0
                x = x1 + t * (x2 - x1)
                y = y1 + t * (y2 - y1)
                return (int(x), int(y))
            d -= edge_len
        # 理论上不会走到这里，兜底返回最后一点
        return tuple(map(int, self.path_vertices[-1]))

    def draw_overlay(self, image):
        """绘制两个沿矩形边线运动的点和矩形轮廓"""
        if self.path_perimeter <= 0:
            return

        now = rospy.Time.now()
        if self.last_time is None:
            dt = 0.0
        else:
            dt = (now - self.last_time).to_sec()
        self.last_time = now

        # 更新时间与距离
        self.path_dist = (self.path_dist + self.point_speed * dt) % self.path_perimeter
        dist1 = self.path_dist
        dist2 = (self.path_dist + self.point_phase_offset) % self.path_perimeter

        p1 = self._point_at_distance(dist1)
        p2 = self._point_at_distance(dist2)

        # 发布红色点位置（点1）到ROS话题
        target_point = Point()
        target_point.x = float(p1[0])
        target_point.y = float(p1[1])
        target_point.z = 0.0
        self.target_point_pub.publish(target_point)

        # 绘制矩形轮廓（淡灰色）
        color_frame = (150, 150, 150)
        thickness_frame = 1
        n = len(self.path_vertices)
        for i in range(n):
            pt1 = tuple(map(int, self.path_vertices[i]))
            pt2 = tuple(map(int, self.path_vertices[(i + 1) % n]))
            cv2.line(image, pt1, pt2, color_frame, thickness_frame, lineType=cv2.LINE_AA)

        # 绘制两个点
        radius = 6
        thickness = -1
        # 点1：红色
        cv2.circle(image, p1, radius, (0, 0, 255), thickness, lineType=cv2.LINE_AA)
        cv2.circle(image, p1, radius + 2, (0, 0, 255), 2, lineType=cv2.LINE_AA)
        # 点2：蓝色
        cv2.circle(image, p2, radius, (255, 0, 0), thickness, lineType=cv2.LINE_AA)
        cv2.circle(image, p2, radius + 2, (255, 0, 0), 2, lineType=cv2.LINE_AA)

        # 在点旁显示坐标
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        text_thickness = 2
        offset = (10, -10)

        def _draw_coord(pt, color):
            text = f"({pt[0]}, {pt[1]})"
            tx = pt[0] + offset[0]
            ty = pt[1] + offset[1]
            # 确保文本不跑出顶部
            ty = max(ty, 15)
            cv2.putText(image, text, (tx, ty), font, font_scale, color, text_thickness, lineType=cv2.LINE_AA)

        _draw_coord(p1, (0, 0, 255))   # 红点坐标
        _draw_coord(p2, (255, 0, 0))   # 蓝点坐标


def main():
    try:
        node = YOLOCarDetection()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("收到键盘中断信号")
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

