;; Auto-generated. Do not edit!


(when (boundp 'arm_follow::PersonsKeypoints)
  (if (not (find-package "ARM_FOLLOW"))
    (make-package "ARM_FOLLOW"))
  (shadow 'PersonsKeypoints (find-package "ARM_FOLLOW")))
(unless (find-package "ARM_FOLLOW::PERSONSKEYPOINTS")
  (make-package "ARM_FOLLOW::PERSONSKEYPOINTS"))

(in-package "ROS")
;;//! \htmlinclude PersonsKeypoints.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass arm_follow::PersonsKeypoints
  :super ros::object
  :slots (_header _persons ))

(defmethod arm_follow::PersonsKeypoints
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:persons __persons) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _persons __persons)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:persons
   (&rest __persons)
   (if (keywordp (car __persons))
       (send* _persons __persons)
     (progn
       (if __persons (setq _persons (car __persons)))
       _persons)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; arm_follow/PersonKeypoints[] _persons
    (apply #'+ (send-all _persons :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; arm_follow/PersonKeypoints[] _persons
     (write-long (length _persons) s)
     (dolist (elem _persons)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; arm_follow/PersonKeypoints[] _persons
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _persons (let (r) (dotimes (i n) (push (instance arm_follow::PersonKeypoints :init) r)) r))
     (dolist (elem- _persons)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get arm_follow::PersonsKeypoints :md5sum-) "2cc6b7f8f80dc708d5b607449f5f648f")
(setf (get arm_follow::PersonsKeypoints :datatype-) "arm_follow/PersonsKeypoints")
(setf (get arm_follow::PersonsKeypoints :definition-)
      "# PersonsKeypoints.msg
# 多个人体的关键点数据列表

std_msgs/Header header
PersonKeypoints[] persons  # 所有人体的关键点列表


================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: arm_follow/PersonKeypoints
# PersonKeypoints.msg
# 一个人体的关键点数据

std_msgs/Header header
uint32 person_id          # 人员ID
float32 bbox_x            # 检测框左上角x
float32 bbox_y            # 检测框左上角y
float32 bbox_width        # 检测框宽度
float32 bbox_height       # 检测框高度
float32 bbox_area         # 检测框面积
float32 bbox_confidence   # 检测框置信度（YOLO显示的置信度）

# 右手关键点（COCO格式中通常是右手腕，索引10）
float32 right_wrist_x     # 右手腕x坐标
float32 right_wrist_y     # 右手腕y坐标
float32 right_wrist_conf  # 右手腕置信度

# 右手肘（索引8）
float32 right_elbow_x
float32 right_elbow_y
float32 right_elbow_conf

# 右肩膀（用于判断右手是否伸出）
float32 right_shoulder_x
float32 right_shoulder_y
float32 right_shoulder_conf

# 图像尺寸（用于计算角度）
uint32 image_width
uint32 image_height


")



(provide :arm_follow/PersonsKeypoints "2cc6b7f8f80dc708d5b607449f5f648f")


