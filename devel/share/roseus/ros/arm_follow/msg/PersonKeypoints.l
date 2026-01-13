;; Auto-generated. Do not edit!


(when (boundp 'arm_follow::PersonKeypoints)
  (if (not (find-package "ARM_FOLLOW"))
    (make-package "ARM_FOLLOW"))
  (shadow 'PersonKeypoints (find-package "ARM_FOLLOW")))
(unless (find-package "ARM_FOLLOW::PERSONKEYPOINTS")
  (make-package "ARM_FOLLOW::PERSONKEYPOINTS"))

(in-package "ROS")
;;//! \htmlinclude PersonKeypoints.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass arm_follow::PersonKeypoints
  :super ros::object
  :slots (_header _person_id _bbox_x _bbox_y _bbox_width _bbox_height _bbox_area _bbox_confidence _right_wrist_x _right_wrist_y _right_wrist_conf _right_elbow_x _right_elbow_y _right_elbow_conf _right_shoulder_x _right_shoulder_y _right_shoulder_conf _image_width _image_height ))

(defmethod arm_follow::PersonKeypoints
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:person_id __person_id) 0)
    ((:bbox_x __bbox_x) 0.0)
    ((:bbox_y __bbox_y) 0.0)
    ((:bbox_width __bbox_width) 0.0)
    ((:bbox_height __bbox_height) 0.0)
    ((:bbox_area __bbox_area) 0.0)
    ((:bbox_confidence __bbox_confidence) 0.0)
    ((:right_wrist_x __right_wrist_x) 0.0)
    ((:right_wrist_y __right_wrist_y) 0.0)
    ((:right_wrist_conf __right_wrist_conf) 0.0)
    ((:right_elbow_x __right_elbow_x) 0.0)
    ((:right_elbow_y __right_elbow_y) 0.0)
    ((:right_elbow_conf __right_elbow_conf) 0.0)
    ((:right_shoulder_x __right_shoulder_x) 0.0)
    ((:right_shoulder_y __right_shoulder_y) 0.0)
    ((:right_shoulder_conf __right_shoulder_conf) 0.0)
    ((:image_width __image_width) 0)
    ((:image_height __image_height) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _person_id (round __person_id))
   (setq _bbox_x (float __bbox_x))
   (setq _bbox_y (float __bbox_y))
   (setq _bbox_width (float __bbox_width))
   (setq _bbox_height (float __bbox_height))
   (setq _bbox_area (float __bbox_area))
   (setq _bbox_confidence (float __bbox_confidence))
   (setq _right_wrist_x (float __right_wrist_x))
   (setq _right_wrist_y (float __right_wrist_y))
   (setq _right_wrist_conf (float __right_wrist_conf))
   (setq _right_elbow_x (float __right_elbow_x))
   (setq _right_elbow_y (float __right_elbow_y))
   (setq _right_elbow_conf (float __right_elbow_conf))
   (setq _right_shoulder_x (float __right_shoulder_x))
   (setq _right_shoulder_y (float __right_shoulder_y))
   (setq _right_shoulder_conf (float __right_shoulder_conf))
   (setq _image_width (round __image_width))
   (setq _image_height (round __image_height))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:person_id
   (&optional __person_id)
   (if __person_id (setq _person_id __person_id)) _person_id)
  (:bbox_x
   (&optional __bbox_x)
   (if __bbox_x (setq _bbox_x __bbox_x)) _bbox_x)
  (:bbox_y
   (&optional __bbox_y)
   (if __bbox_y (setq _bbox_y __bbox_y)) _bbox_y)
  (:bbox_width
   (&optional __bbox_width)
   (if __bbox_width (setq _bbox_width __bbox_width)) _bbox_width)
  (:bbox_height
   (&optional __bbox_height)
   (if __bbox_height (setq _bbox_height __bbox_height)) _bbox_height)
  (:bbox_area
   (&optional __bbox_area)
   (if __bbox_area (setq _bbox_area __bbox_area)) _bbox_area)
  (:bbox_confidence
   (&optional __bbox_confidence)
   (if __bbox_confidence (setq _bbox_confidence __bbox_confidence)) _bbox_confidence)
  (:right_wrist_x
   (&optional __right_wrist_x)
   (if __right_wrist_x (setq _right_wrist_x __right_wrist_x)) _right_wrist_x)
  (:right_wrist_y
   (&optional __right_wrist_y)
   (if __right_wrist_y (setq _right_wrist_y __right_wrist_y)) _right_wrist_y)
  (:right_wrist_conf
   (&optional __right_wrist_conf)
   (if __right_wrist_conf (setq _right_wrist_conf __right_wrist_conf)) _right_wrist_conf)
  (:right_elbow_x
   (&optional __right_elbow_x)
   (if __right_elbow_x (setq _right_elbow_x __right_elbow_x)) _right_elbow_x)
  (:right_elbow_y
   (&optional __right_elbow_y)
   (if __right_elbow_y (setq _right_elbow_y __right_elbow_y)) _right_elbow_y)
  (:right_elbow_conf
   (&optional __right_elbow_conf)
   (if __right_elbow_conf (setq _right_elbow_conf __right_elbow_conf)) _right_elbow_conf)
  (:right_shoulder_x
   (&optional __right_shoulder_x)
   (if __right_shoulder_x (setq _right_shoulder_x __right_shoulder_x)) _right_shoulder_x)
  (:right_shoulder_y
   (&optional __right_shoulder_y)
   (if __right_shoulder_y (setq _right_shoulder_y __right_shoulder_y)) _right_shoulder_y)
  (:right_shoulder_conf
   (&optional __right_shoulder_conf)
   (if __right_shoulder_conf (setq _right_shoulder_conf __right_shoulder_conf)) _right_shoulder_conf)
  (:image_width
   (&optional __image_width)
   (if __image_width (setq _image_width __image_width)) _image_width)
  (:image_height
   (&optional __image_height)
   (if __image_height (setq _image_height __image_height)) _image_height)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _person_id
    4
    ;; float32 _bbox_x
    4
    ;; float32 _bbox_y
    4
    ;; float32 _bbox_width
    4
    ;; float32 _bbox_height
    4
    ;; float32 _bbox_area
    4
    ;; float32 _bbox_confidence
    4
    ;; float32 _right_wrist_x
    4
    ;; float32 _right_wrist_y
    4
    ;; float32 _right_wrist_conf
    4
    ;; float32 _right_elbow_x
    4
    ;; float32 _right_elbow_y
    4
    ;; float32 _right_elbow_conf
    4
    ;; float32 _right_shoulder_x
    4
    ;; float32 _right_shoulder_y
    4
    ;; float32 _right_shoulder_conf
    4
    ;; uint32 _image_width
    4
    ;; uint32 _image_height
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _person_id
       (write-long _person_id s)
     ;; float32 _bbox_x
       (sys::poke _bbox_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _bbox_y
       (sys::poke _bbox_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _bbox_width
       (sys::poke _bbox_width (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _bbox_height
       (sys::poke _bbox_height (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _bbox_area
       (sys::poke _bbox_area (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _bbox_confidence
       (sys::poke _bbox_confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_wrist_x
       (sys::poke _right_wrist_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_wrist_y
       (sys::poke _right_wrist_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_wrist_conf
       (sys::poke _right_wrist_conf (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_elbow_x
       (sys::poke _right_elbow_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_elbow_y
       (sys::poke _right_elbow_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_elbow_conf
       (sys::poke _right_elbow_conf (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_shoulder_x
       (sys::poke _right_shoulder_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_shoulder_y
       (sys::poke _right_shoulder_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_shoulder_conf
       (sys::poke _right_shoulder_conf (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _image_width
       (write-long _image_width s)
     ;; uint32 _image_height
       (write-long _image_height s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _person_id
     (setq _person_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _bbox_x
     (setq _bbox_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _bbox_y
     (setq _bbox_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _bbox_width
     (setq _bbox_width (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _bbox_height
     (setq _bbox_height (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _bbox_area
     (setq _bbox_area (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _bbox_confidence
     (setq _bbox_confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_wrist_x
     (setq _right_wrist_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_wrist_y
     (setq _right_wrist_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_wrist_conf
     (setq _right_wrist_conf (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_elbow_x
     (setq _right_elbow_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_elbow_y
     (setq _right_elbow_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_elbow_conf
     (setq _right_elbow_conf (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_shoulder_x
     (setq _right_shoulder_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_shoulder_y
     (setq _right_shoulder_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_shoulder_conf
     (setq _right_shoulder_conf (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _image_width
     (setq _image_width (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _image_height
     (setq _image_height (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get arm_follow::PersonKeypoints :md5sum-) "b8aef0b6f60577e67360824121ee7a78")
(setf (get arm_follow::PersonKeypoints :datatype-) "arm_follow/PersonKeypoints")
(setf (get arm_follow::PersonKeypoints :definition-)
      "# PersonKeypoints.msg
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

")



(provide :arm_follow/PersonKeypoints "b8aef0b6f60577e67360824121ee7a78")


