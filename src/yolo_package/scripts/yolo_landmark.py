#!/usr/bin/env python3
"""
yolo_landmark.py - YOLO物体检测节点（用于视觉抓取）

功能：
1. 使用YOLO模型检测桌面上的物体（纸盒）
2. 发布检测结果到 /yolo_pose/keypoints 话题
3. 供 visual_grasping 节点使用

使用方法：
    rosrun yolo_package yolo_landmark.py
"""

import sys
from typing import Any, Union, Optional
from threading import Lock
import numpy as np

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import intera_interface
from arm_follow.msg import PersonsKeypoints, PersonKeypoints


class ROSImageStream:
    """A wrapper to convert ROS Image topic into a generator for YOLO."""
    
    def __init__(self, topic: str, timeout: float = 30.0):
        self.topic = topic
        self.timeout = timeout
        self.bridge = CvBridge()
        self.latest_image: Optional[np.ndarray] = None
        self.lock = Lock()
        self.sub = rospy.Subscriber(topic, Image, self._image_callback, queue_size=1, buff_size=2**23)
        # 简化日志：只在节点启动时提示一次订阅，后续不再频繁打印
        rospy.loginfo("YOLO image subscriber created on topic: %s", topic)
        
    def _image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.lock:
                self.latest_image = cv_image.copy()
        except CvBridgeError as e:
            rospy.logerr("CvBridge error: %s", e)
    
    def __iter__(self):
        return self
    
    def __next__(self):
        # Wait for the first image; 不再超时退出，而是一直等待，防止节点自动关闭
        while True:
            with self.lock:
                if self.latest_image is not None:
                    # 不再打印“等待图像/第一次收到图像”等重复信息
                    return self.latest_image.copy()
            
            if rospy.is_shutdown():
                raise StopIteration("ROS shutdown requested")
            rospy.sleep(0.01)


def _normalize_source(raw_source: Any) -> Union[int, str]:
    """
    Allow ROS params like "0" or "1" to be converted into integers that match
    OpenCV/Ultralytics expectations. Non-numeric strings are returned as-is.
    """
    if isinstance(raw_source, str):
        stripped = raw_source.strip()
        if stripped.lstrip("-").isdigit():
            try:
                return int(stripped)
            except ValueError:
                pass
    return raw_source


def extract_detections_from_results(
    results,
    image_shape,
    conf_threshold,
):
    """
    Extract object detections from YOLO results and convert to ROS message format.
    用于视觉抓取：检测物体（纸盒）而不是人体姿态。
    
    Args:
        results: YOLO inference results (object detection, not pose)
        image_shape: Tuple (height, width) of the image
        conf_threshold: Detection confidence threshold
        
    Returns:
        PersonsKeypoints ROS message (reusing message format for compatibility)
    """
    msg = PersonsKeypoints()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "camera_frame"
    
    if not results or len(results) == 0:
        return msg
    
    result = results[0]  # Get first result
    
    # Check if result has boxes (object detection)
    if not hasattr(result, 'boxes') or result.boxes is None or len(result.boxes) == 0:
        return msg
    
    h, w = image_shape[:2]
    
    object_id = 0
    for box in result.boxes:
        # Extract bounding box
        bbox = box.xyxy[0].cpu().numpy()  # [x1, y1, x2, y2]
        bbox_area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])
        
        # Extract detection confidence
        bbox_conf = float(box.conf[0].cpu().numpy()) if hasattr(box, 'conf') and box.conf is not None else 0.0
        
        # Filter by confidence threshold
        if bbox_conf < conf_threshold:
            continue
        
        # Create PersonKeypoints message (reusing format for compatibility with C++ node)
        person = PersonKeypoints()
        person.header = msg.header
        person.person_id = object_id  # 实际上这是物体ID
        person.bbox_x = float(bbox[0])
        person.bbox_y = float(bbox[1])
        person.bbox_width = float(bbox[2] - bbox[0])
        person.bbox_height = float(bbox[3] - bbox[1])
        person.bbox_area = float(bbox_area)
        person.bbox_confidence = bbox_conf  # YOLO检测框的置信度
        person.image_width = w
        person.image_height = h
        
        # 视觉抓取不需要关键点，全部置为 0
        person.right_shoulder_x = 0.0
        person.right_shoulder_y = 0.0
        person.right_elbow_x = 0.0
        person.right_elbow_y = 0.0
        person.right_wrist_x = 0.0
        person.right_wrist_y = 0.0
        
        msg.persons.append(person)
        object_id += 1
    
    return msg


def main() -> None:
    rospy.init_node("yolo_detection", anonymous=False)

    # 默认使用机械臂头部摄像头和纸盒检测模型（yolov8n_landmark.pt）
    model_path = rospy.get_param("~model", "/home/mycar/YOLO/ultralytics-8.3.163/yolov8n_landmark.pt")
    source = _normalize_source(rospy.get_param("~source", 1))  # 默认使用头部摄像头
    stream = bool(rospy.get_param("~stream", True))
    window_name = rospy.get_param("~window_name", "YOLO Inference")
    camera_topic = rospy.get_param(
        "~camera_topic", 
        "/io/internal_camera/head_camera/image_raw"
    )
    # 物体检测置信度阈值（用于视觉抓取）
    # 支持从launch文件传入conf_threshold，如果没有则使用默认值0.5
    conf_threshold = float(rospy.get_param("~conf_threshold", rospy.get_param("~avg_conf_threshold", 0.5)))
    startup_delay = float(rospy.get_param("~startup_delay", 0.0))

    if startup_delay > 0.0:
        rospy.loginfo("YOLO node will start after %.1f seconds delay to wait for arm initialization.", startup_delay)
        rospy.sleep(startup_delay)

    rospy.loginfo("Loading YOLO model: %s", model_path)
    model = YOLO(model_path)

    # If source is 1, use ROS topic for head camera
    if source == 1:
        rospy.loginfo("Setting up head camera for YOLO inference...")
        
        # Initialize camera interface and start streaming
        cameras = intera_interface.Cameras()
        camera_name = "head_camera"
        
        # Verify camera exists
        if not cameras.verify_camera_exists(camera_name):
            rospy.logerr("Head camera '{}' not found! Exiting.".format(camera_name))
            return
        
        # Start camera streaming
        rospy.loginfo("Starting head camera streaming...")
        if not cameras.start_streaming(camera_name):
            rospy.logerr("Failed to start head camera streaming! Exiting.")
            return
        
        rospy.loginfo("Head camera streaming started successfully.")
        
        # Set camera to auto exposure and auto gain for better image quality
        rospy.loginfo("Setting camera to auto exposure and auto gain...")
        if cameras.set_exposure(camera_name, -1):  # -1 means auto-exposure
            rospy.loginfo("Auto exposure enabled")
        else:
            rospy.logwarn("Failed to set auto exposure")
        
        if cameras.set_gain(camera_name, -1):  # -1 means auto-gain
            rospy.loginfo("Auto gain enabled")
        else:
            rospy.logwarn("Failed to set auto gain")
        
        # Wait for exposure and gain to stabilize (8-10 seconds)
        rospy.loginfo("Waiting 10 seconds for camera exposure and gain to stabilize...")
        rospy.sleep(10.0)
        rospy.loginfo("Camera exposure stabilization complete, starting detection...")
        
        # Use rectified color image topic (image_rect_color)
        # Based on intera_interface code, rectified topic is: /io/internal_camera/head_camera/image_rect_color
        camera_topic = "/io/internal_camera/{}/image_rect_color".format(camera_name)
        rospy.loginfo("Using ROS topic for head camera: %s", camera_topic)
        
        # Wait a bit for the topic to start publishing
        rospy.loginfo("Waiting for camera topic to be available...")
        try:
            rospy.wait_for_message(camera_topic, Image, timeout=5.0)
            rospy.loginfo("Camera topic is now available!")
        except rospy.ROSException:
            rospy.logwarn("Could not receive message from %s yet, continuing anyway...", camera_topic)
        
        image_stream = ROSImageStream(camera_topic, timeout=30.0)
        
        def _shutdown_hook():
            rospy.loginfo("Stopping camera streaming...")
            try:
                cameras.stop_streaming(camera_name)
            except:
                pass
            cv2.destroyAllWindows()
            rospy.loginfo("YOLO inference node stopped.")

        rospy.on_shutdown(_shutdown_hook)
        
        # Create publisher for detections (reusing /yolo_pose/keypoints topic for compatibility)
        detections_pub = rospy.Publisher("/yolo_pose/keypoints", PersonsKeypoints, queue_size=1)
        detection_count = 0

        try:
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            rospy.loginfo("YOLO detection node ready. Starting detection loop...")
            for cv_image in image_stream:
                if rospy.is_shutdown():
                    break
                # Run YOLO inference on the image (object detection)
                results = model(cv_image, verbose=False, conf=conf_threshold)
                if len(results) > 0:
                    plotted = results[0].plot()
                    cv2.imshow(window_name, plotted)
                    
                    # Extract and publish detections
                    detections_msg = extract_detections_from_results(
                        results,
                        cv_image.shape,
                        conf_threshold,
                    )
                    detections_pub.publish(detections_msg)
                    # 可以在需要时查看 detection_count，但默认不打印
                    detection_count += len(detections_msg.persons)
                
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    rospy.loginfo("Received 'q' keypress, stopping inference.")
                    break
        except StopIteration as e:
            rospy.logerr("Image stream stopped: %s", e)
        except (rospy.ROSInterruptException, KeyboardInterrupt):
            rospy.loginfo("YOLO inference interrupted.")
        finally:
            _shutdown_hook()
    else:
        # For USB cameras (source=0 or other device indices), use standard approach
        rospy.loginfo("Starting inference from source=%s (stream=%s)", source, stream)
        results = model(source=source, stream=stream)

        def _shutdown_hook():
            cv2.destroyAllWindows()
            rospy.loginfo("YOLO inference node stopped.")

        rospy.on_shutdown(_shutdown_hook)

        # Create publisher for detections (for USB camera mode)
        detections_pub = rospy.Publisher("/yolo_pose/keypoints", PersonsKeypoints, queue_size=1)
        detection_count = 0
        
        try:
            for result in results:
                if rospy.is_shutdown():
                    break
                
                # Get image from result if available
                if hasattr(result, 'orig_img'):
                    cv_image = result.orig_img
                    image_shape = cv_image.shape
                else:
                    # Fallback: create dummy shape if image not available
                    image_shape = (480, 640, 3)
                
                plotted = result.plot()
                cv2.imshow(window_name, plotted)
                
                # Extract and publish detections
                results_list = [result]  # Wrap in list for compatibility
                detections_msg = extract_detections_from_results(
                    results_list,
                    image_shape,
                    conf_threshold,
                )
                detections_pub.publish(detections_msg)
                detection_count += len(detections_msg.persons)
                
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    rospy.loginfo("Received 'q' keypress, stopping inference.")
                    break
        except (rospy.ROSInterruptException, KeyboardInterrupt):
            rospy.loginfo("YOLO inference interrupted.")
        finally:
            _shutdown_hook()


if __name__ == "__main__":
    main()

