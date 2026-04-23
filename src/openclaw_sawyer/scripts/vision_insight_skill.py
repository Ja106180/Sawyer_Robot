#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import os
import json
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger, TriggerResponse

class VisionInsightSkill:
    def __init__(self):
        rospy.init_node('vision_insight_skill')
        self.bridge = CvBridge()
        self.latest_image = None
        
        # Head camera topic from research
        self.camera_topic = "/io/internal_camera/head_camera/image_rect_color"
        self.sub = rospy.Subscriber(self.camera_topic, Image, self.image_callback)
        
        # Service
        rospy.Service("~get_insight", Trigger, self.handle_get_insight)
        
        # Save path for OpenClaw to "see"
        self.save_dir = "/tmp/openclaw_vision"
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            
        rospy.loginfo("Vision Insight Skill ready on topic: %s", self.camera_topic)

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            pass

    def handle_get_insight(self, req):
        if self.latest_image is None:
            return TriggerResponse(success=False, message=json.dumps({"error": "No image available from head camera."}))
        
        save_path = os.path.join(self.save_dir, "head_insight.jpg")
        cv2.imwrite(save_path, self.latest_image)
        
        # Return JSON with the path and some metadata
        result = {
            "success": True,
            "image_path": save_path,
            "description": "Head camera snapshot captured.",
            "camera_name": "Sawyer Head Camera"
        }
        return TriggerResponse(success=True, message=json.dumps(result))

if __name__ == '__main__':
    try:
        skill = VisionInsightSkill()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
