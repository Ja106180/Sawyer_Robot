#!/usr/bin/env python3

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
        rospy.loginfo("Subscribed to ROS image topic: %s", topic)
        
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
        # Wait for the first image (with longer timeout)
        start_time = rospy.Time.now()
        rospy.loginfo("Waiting for first image from camera...")
        while True:
            with self.lock:
                if self.latest_image is not None:
                    return self.latest_image.copy()
            
            if rospy.is_shutdown():
                raise StopIteration("ROS shutdown requested")
            
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > self.timeout:
                raise StopIteration("Timeout waiting for image from ROS topic (waited {}s)".format(self.timeout))
            
            if elapsed > 1.0 and int(elapsed) % 2 == 0 and elapsed < 2.1:
                rospy.loginfo("Still waiting for image... ({}s elapsed)".format(int(elapsed)))
            
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


def extract_keypoints_from_results(
    results,
    image_shape,
    avg_conf_threshold,
    keypoint_presence_threshold,
):
    """
    Extract keypoints from YOLO pose results and convert to ROS message format.
    
    Args:
        results: YOLO inference results
        image_shape: Tuple (height, width) of the image
        person_id_counter: Counter for assigning person IDs
        
    Returns:
        PersonsKeypoints ROS message
    """
    msg = PersonsKeypoints()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "camera_frame"
    
    if not results or len(results) == 0:
        return msg
    
    result = results[0]  # Get first result
    
    # Check if result has keypoints
    if not hasattr(result, 'keypoints') or result.keypoints is None:
        return msg
    
    h, w = image_shape[:2]
    
    # YOLO COCO pose keypoint indices:
    # 6: right_shoulder, 8: right_elbow, 10: right_wrist
    RIGHT_SHOULDER_IDX = 6
    RIGHT_ELBOW_IDX = 8
    RIGHT_WRIST_IDX = 10
    
    required_body_indices = [5, 6, 11, 12]  # shoulders and hips
    required_arm_indices = [7, 8, 9, 10]     # elbows and wrists (left/right)
    
    person_id = 0
    for box, keypoints in zip(result.boxes, result.keypoints):
        if keypoints is None or keypoints.data is None:
            continue
        
        # Extract bounding box
        bbox = box.xyxy[0].cpu().numpy()  # [x1, y1, x2, y2]
        bbox_area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])
        
        person = PersonKeypoints()
        person.header = msg.header
        person.person_id = person_id
        person.bbox_x = float(bbox[0])
        person.bbox_y = float(bbox[1])
        person.bbox_width = float(bbox[2] - bbox[0])
        person.bbox_height = float(bbox[3] - bbox[1])
        person.bbox_area = float(bbox_area)
        person.image_width = w
        person.image_height = h
        
        # Extract keypoints data
        kpts = keypoints.data[0].cpu().numpy()  # Shape: [num_keypoints, 3] (x, y, conf)

        # Check keypoint completeness: torso (4 pts) + both arms (4 pts total: elbows & wrists)
        def _has_required_points(indices):
            return all(
                idx < len(kpts) and kpts[idx, 2] >= keypoint_presence_threshold
                for idx in indices
            )

        if not (_has_required_points(required_body_indices) and
                _has_required_points(required_arm_indices)):
            continue

        avg_confidence = float(np.mean(kpts[:, 2]))
        if avg_confidence < avg_conf_threshold:
            continue
        
        if len(kpts) > RIGHT_SHOULDER_IDX:
            person.right_shoulder_x = float(kpts[RIGHT_SHOULDER_IDX, 0])
            person.right_shoulder_y = float(kpts[RIGHT_SHOULDER_IDX, 1])
            person.right_shoulder_conf = float(kpts[RIGHT_SHOULDER_IDX, 2])
        else:
            person.right_shoulder_conf = 0.0
        
        if len(kpts) > RIGHT_ELBOW_IDX:
            person.right_elbow_x = float(kpts[RIGHT_ELBOW_IDX, 0])
            person.right_elbow_y = float(kpts[RIGHT_ELBOW_IDX, 1])
            person.right_elbow_conf = float(kpts[RIGHT_ELBOW_IDX, 2])
        else:
            person.right_elbow_conf = 0.0
        
        if len(kpts) > RIGHT_WRIST_IDX:
            person.right_wrist_x = float(kpts[RIGHT_WRIST_IDX, 0])
            person.right_wrist_y = float(kpts[RIGHT_WRIST_IDX, 1])
            person.right_wrist_conf = float(kpts[RIGHT_WRIST_IDX, 2])
        else:
            person.right_wrist_conf = 0.0
        
        msg.persons.append(person)
        person_id += 1
    
    return msg


def main() -> None:
    rospy.init_node("yolo_inference", anonymous=False)

    # 默认使用机械臂头部摄像头和指定的模型路径
    model_path = rospy.get_param("~model", "/home/mycar/YOLO/ultralytics-8.3.163/yolov8n-pose.pt")
    source = _normalize_source(rospy.get_param("~source", 1))  # 默认使用头部摄像头
    stream = bool(rospy.get_param("~stream", True))
    window_name = rospy.get_param("~window_name", "YOLO Inference")
    camera_topic = rospy.get_param(
        "~camera_topic", 
        "/io/internal_camera/head_camera/image_raw"
    )
    avg_conf_threshold = float(rospy.get_param("~avg_conf_threshold", 0.6))
    keypoint_presence_threshold = float(rospy.get_param("~keypoint_presence_threshold", 0.5))

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
        
        # Create publisher for keypoints
        keypoints_pub = rospy.Publisher("/yolo_pose/keypoints", PersonsKeypoints, queue_size=1)
        person_id_counter = 0

        try:
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            rospy.loginfo("Starting YOLO inference loop. Press 'q' to quit.")
            for cv_image in image_stream:
                if rospy.is_shutdown():
                    break
                # Run YOLO inference on the image
                results = model(cv_image, verbose=False)
                if len(results) > 0:
                    plotted = results[0].plot()
                    cv2.imshow(window_name, plotted)
                    
                    # Extract and publish keypoints
                    keypoints_msg = extract_keypoints_from_results(
                        results,
                        cv_image.shape,
                        avg_conf_threshold,
                        keypoint_presence_threshold,
                    )
                    keypoints_pub.publish(keypoints_msg)
                    person_id_counter += len(keypoints_msg.persons)
                
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

        # Create publisher for keypoints (for USB camera mode)
        keypoints_pub = rospy.Publisher("/yolo_pose/keypoints", PersonsKeypoints, queue_size=1)
        person_id_counter = 0
        
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
                
                # Extract and publish keypoints
                results_list = [result]  # Wrap in list for compatibility
                keypoints_msg = extract_keypoints_from_results(
                    results_list,
                    image_shape,
                    avg_conf_threshold,
                    keypoint_presence_threshold,
                )
                keypoints_pub.publish(keypoints_msg)
                person_id_counter += len(keypoints_msg.persons)
                
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    rospy.loginfo("Received 'q' keypress, stopping inference.")
                    break
        except (rospy.ROSInterruptException, KeyboardInterrupt):
            rospy.loginfo("YOLO inference interrupted.")
        finally:
            _shutdown_hook()


if __name__ == "__main__":
    main()

