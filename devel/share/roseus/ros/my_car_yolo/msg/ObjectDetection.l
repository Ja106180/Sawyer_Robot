;; Auto-generated. Do not edit!


(when (boundp 'my_car_yolo::ObjectDetection)
  (if (not (find-package "MY_CAR_YOLO"))
    (make-package "MY_CAR_YOLO"))
  (shadow 'ObjectDetection (find-package "MY_CAR_YOLO")))
(unless (find-package "MY_CAR_YOLO::OBJECTDETECTION")
  (make-package "MY_CAR_YOLO::OBJECTDETECTION"))

(in-package "ROS")
;;//! \htmlinclude ObjectDetection.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass my_car_yolo::ObjectDetection
  :super ros::object
  :slots (_header _object_id _center_x _center_y _confidence _bbox_x _bbox_y _bbox_width _bbox_height _image_width _image_height ))

(defmethod my_car_yolo::ObjectDetection
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:object_id __object_id) 0)
    ((:center_x __center_x) 0.0)
    ((:center_y __center_y) 0.0)
    ((:confidence __confidence) 0.0)
    ((:bbox_x __bbox_x) 0.0)
    ((:bbox_y __bbox_y) 0.0)
    ((:bbox_width __bbox_width) 0.0)
    ((:bbox_height __bbox_height) 0.0)
    ((:image_width __image_width) 0)
    ((:image_height __image_height) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _object_id (round __object_id))
   (setq _center_x (float __center_x))
   (setq _center_y (float __center_y))
   (setq _confidence (float __confidence))
   (setq _bbox_x (float __bbox_x))
   (setq _bbox_y (float __bbox_y))
   (setq _bbox_width (float __bbox_width))
   (setq _bbox_height (float __bbox_height))
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
  (:object_id
   (&optional __object_id)
   (if __object_id (setq _object_id __object_id)) _object_id)
  (:center_x
   (&optional __center_x)
   (if __center_x (setq _center_x __center_x)) _center_x)
  (:center_y
   (&optional __center_y)
   (if __center_y (setq _center_y __center_y)) _center_y)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
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
    ;; uint32 _object_id
    4
    ;; float32 _center_x
    4
    ;; float32 _center_y
    4
    ;; float32 _confidence
    4
    ;; float32 _bbox_x
    4
    ;; float32 _bbox_y
    4
    ;; float32 _bbox_width
    4
    ;; float32 _bbox_height
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
     ;; uint32 _object_id
       (write-long _object_id s)
     ;; float32 _center_x
       (sys::poke _center_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _center_y
       (sys::poke _center_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _bbox_x
       (sys::poke _bbox_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _bbox_y
       (sys::poke _bbox_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _bbox_width
       (sys::poke _bbox_width (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _bbox_height
       (sys::poke _bbox_height (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
   ;; uint32 _object_id
     (setq _object_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _center_x
     (setq _center_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _center_y
     (setq _center_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _bbox_x
     (setq _bbox_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _bbox_y
     (setq _bbox_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _bbox_width
     (setq _bbox_width (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _bbox_height
     (setq _bbox_height (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _image_width
     (setq _image_width (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _image_height
     (setq _image_height (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get my_car_yolo::ObjectDetection :md5sum-) "74b64be95b23c1ba8f1d1679b6ef3c59")
(setf (get my_car_yolo::ObjectDetection :datatype-) "my_car_yolo/ObjectDetection")
(setf (get my_car_yolo::ObjectDetection :definition-)
      "# ObjectDetection.msg
# 单个物体的检测结果

std_msgs/Header header
uint32 object_id          # 物体ID (1, 2, 3...)
float32 center_x          # 中心点x坐标（像素）
float32 center_y          # 中心点y坐标（像素）
float32 confidence        # 检测置信度
float32 bbox_x            # 边界框左上角x
float32 bbox_y            # 边界框左上角y
float32 bbox_width        # 边界框宽度
float32 bbox_height        # 边界框高度
uint32 image_width        # 图像宽度
uint32 image_height       # 图像高度

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



(provide :my_car_yolo/ObjectDetection "74b64be95b23c1ba8f1d1679b6ef3c59")


