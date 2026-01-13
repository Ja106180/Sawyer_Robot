;; Auto-generated. Do not edit!


(when (boundp 'my_car_yolo::ObjectDetections)
  (if (not (find-package "MY_CAR_YOLO"))
    (make-package "MY_CAR_YOLO"))
  (shadow 'ObjectDetections (find-package "MY_CAR_YOLO")))
(unless (find-package "MY_CAR_YOLO::OBJECTDETECTIONS")
  (make-package "MY_CAR_YOLO::OBJECTDETECTIONS"))

(in-package "ROS")
;;//! \htmlinclude ObjectDetections.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass my_car_yolo::ObjectDetections
  :super ros::object
  :slots (_header _objects ))

(defmethod my_car_yolo::ObjectDetections
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:objects __objects) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _objects __objects)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:objects
   (&rest __objects)
   (if (keywordp (car __objects))
       (send* _objects __objects)
     (progn
       (if __objects (setq _objects (car __objects)))
       _objects)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; my_car_yolo/ObjectDetection[] _objects
    (apply #'+ (send-all _objects :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; my_car_yolo/ObjectDetection[] _objects
     (write-long (length _objects) s)
     (dolist (elem _objects)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; my_car_yolo/ObjectDetection[] _objects
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _objects (let (r) (dotimes (i n) (push (instance my_car_yolo::ObjectDetection :init) r)) r))
     (dolist (elem- _objects)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get my_car_yolo::ObjectDetections :md5sum-) "4612b7e38ebc3d1f73882710c9d084e0")
(setf (get my_car_yolo::ObjectDetections :datatype-) "my_car_yolo/ObjectDetections")
(setf (get my_car_yolo::ObjectDetections :definition-)
      "# ObjectDetections.msg
# 多个物体的检测结果列表

std_msgs/Header header
ObjectDetection[] objects  # 所有检测到的物体列表

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
MSG: my_car_yolo/ObjectDetection
# ObjectDetection.msg
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

")



(provide :my_car_yolo/ObjectDetections "4612b7e38ebc3d1f73882710c9d084e0")


