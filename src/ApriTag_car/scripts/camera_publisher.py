#!/usr/bin/env python3
"""
camera_publisher.py - USB 相机图像发布节点

功能：
1. 从 USB 摄像头读取图像
2. 发布图像到 /camera/image_raw 话题

使用方法：
rosrun ApriTag_car camera_publisher.py
"""

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Header


class CameraPublisher:
    def __init__(self):
        rospy.init_node("camera_publisher", anonymous=True)
        
        # 获取参数
        self.camera_id = rospy.get_param("~camera_id", 0)  # 默认摄像头 ID
        self.frame_rate = rospy.get_param("~frame_rate", 30.0)  # 帧率
        
        # 初始化摄像头
        self.cap = cv2.VideoCapture(self.camera_id)
        
        if not self.cap.isOpened():
            rospy.logerr("无法打开摄像头 ID: %d", self.camera_id)
            rospy.signal_shutdown("摄像头打开失败")
            return
        
        # 设置摄像头参数（可选）
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)
        
        # CvBridge
        self.bridge = CvBridge()
        
        # 发布者
        self.image_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=1)
        
        rospy.loginfo("相机发布节点已启动")
        rospy.loginfo("摄像头 ID: %d", self.camera_id)
        rospy.loginfo("发布话题: /camera/image_raw")
        rospy.loginfo("帧率: %.1f Hz", self.frame_rate)
        
        # 设置循环频率
        self.rate = rospy.Rate(self.frame_rate)
    
    def run(self):
        """主循环"""
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            
            if not ret:
                rospy.logwarn("无法读取摄像头画面")
                self.rate.sleep()
                continue
            
            try:
                # 转换为 ROS 图像消息
                image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                image_msg.header = Header()
                image_msg.header.stamp = rospy.Time.now()
                image_msg.header.frame_id = "camera_frame"
                
                # 发布图像
                self.image_pub.publish(image_msg)
                
            except Exception as e:
                rospy.logerr("发布图像错误: %s", e)
            
            self.rate.sleep()
        
        # 清理
        self.cap.release()
        rospy.loginfo("相机发布节点已关闭")


def main():
    try:
        publisher = CameraPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("节点异常: %s", e)


if __name__ == "__main__":
    main()

