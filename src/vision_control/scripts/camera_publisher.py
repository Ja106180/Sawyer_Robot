#!/usr/bin/env python3
"""
camera_publisher.py

USB摄像头图像发布节点：
1. 从USB摄像头读取图像
2. 发布图像到 /camera/image_raw 话题
3. 供其他节点（vision_control_node, face_follow）订阅使用
"""

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Header


class CameraPublisher:
    def __init__(self):
        rospy.init_node("camera_publisher", anonymous=False)
        
        # 参数配置
        self.camera_id = rospy.get_param("~camera_id", 0)
        self.frame_width = rospy.get_param("~frame_width", 640)
        self.frame_height = rospy.get_param("~frame_height", 480)
        self.fps = rospy.get_param("~fps", 30.0)
        
        # 初始化摄像头
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            rospy.logerr("Failed to open camera ID: %d", self.camera_id)
            rospy.signal_shutdown("Camera open failed")
            return
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        # CvBridge
        self.bridge = CvBridge()
        
        # 发布者：发布原始图像到统一话题
        self.image_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=1)
        
        rospy.loginfo("Camera publisher node started")
        rospy.loginfo("Camera ID: %d, Resolution: %dx%d, FPS: %.1f Hz", 
                     self.camera_id, self.frame_width, self.frame_height, self.fps)
        rospy.loginfo("Publishing to topic: /camera/image_raw")
        
        # 设置循环频率
        self.rate = rospy.Rate(self.fps)
    
    def run(self):
        """主循环"""
        rospy.loginfo("Camera publisher node running...")
        
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            
            if not ret:
                rospy.logwarn("Failed to read camera frame")
                self.rate.sleep()
                continue
            
            try:
                # 转换为ROS图像消息
                image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                image_msg.header = Header()
                image_msg.header.stamp = rospy.Time.now()
                image_msg.header.frame_id = "camera_frame"
                
                # 发布图像
                self.image_pub.publish(image_msg)
                
            except Exception as e:
                rospy.logerr("Publish image error: %s", e)
            
            self.rate.sleep()
        
        # 清理
        self.cap.release()
        rospy.loginfo("Camera publisher node closed")


if __name__ == "__main__":
    try:
        publisher = CameraPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Node exception: %s", e)

