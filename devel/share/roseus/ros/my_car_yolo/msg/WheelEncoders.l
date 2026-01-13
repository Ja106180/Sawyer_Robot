;; Auto-generated. Do not edit!


(when (boundp 'my_car_yolo::WheelEncoders)
  (if (not (find-package "MY_CAR_YOLO"))
    (make-package "MY_CAR_YOLO"))
  (shadow 'WheelEncoders (find-package "MY_CAR_YOLO")))
(unless (find-package "MY_CAR_YOLO::WHEELENCODERS")
  (make-package "MY_CAR_YOLO::WHEELENCODERS"))

(in-package "ROS")
;;//! \htmlinclude WheelEncoders.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass my_car_yolo::WheelEncoders
  :super ros::object
  :slots (_header _left_pulses _right_pulses _left_velocity _right_velocity ))

(defmethod my_car_yolo::WheelEncoders
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:left_pulses __left_pulses) 0)
    ((:right_pulses __right_pulses) 0)
    ((:left_velocity __left_velocity) 0.0)
    ((:right_velocity __right_velocity) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _left_pulses (round __left_pulses))
   (setq _right_pulses (round __right_pulses))
   (setq _left_velocity (float __left_velocity))
   (setq _right_velocity (float __right_velocity))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:left_pulses
   (&optional __left_pulses)
   (if __left_pulses (setq _left_pulses __left_pulses)) _left_pulses)
  (:right_pulses
   (&optional __right_pulses)
   (if __right_pulses (setq _right_pulses __right_pulses)) _right_pulses)
  (:left_velocity
   (&optional __left_velocity)
   (if __left_velocity (setq _left_velocity __left_velocity)) _left_velocity)
  (:right_velocity
   (&optional __right_velocity)
   (if __right_velocity (setq _right_velocity __right_velocity)) _right_velocity)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _left_pulses
    4
    ;; int32 _right_pulses
    4
    ;; float64 _left_velocity
    8
    ;; float64 _right_velocity
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _left_pulses
       (write-long _left_pulses s)
     ;; int32 _right_pulses
       (write-long _right_pulses s)
     ;; float64 _left_velocity
       (sys::poke _left_velocity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _right_velocity
       (sys::poke _right_velocity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _left_pulses
     (setq _left_pulses (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _right_pulses
     (setq _right_pulses (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _left_velocity
     (setq _left_velocity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _right_velocity
     (setq _right_velocity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get my_car_yolo::WheelEncoders :md5sum-) "04b36400bf0a0e74c73173ddcaa70f42")
(setf (get my_car_yolo::WheelEncoders :datatype-) "my_car_yolo/WheelEncoders")
(setf (get my_car_yolo::WheelEncoders :definition-)
      "# WheelEncoders.msg
# 轮子编码器原始数据

std_msgs/Header header
int32 left_pulses              # 左轮编码器脉冲数
int32 right_pulses             # 右轮编码器脉冲数
float64 left_velocity          # 左轮速度 (m/s)
float64 right_velocity         # 右轮速度 (m/s)

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



(provide :my_car_yolo/WheelEncoders "04b36400bf0a0e74c73173ddcaa70f42")


