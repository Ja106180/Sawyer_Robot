;; Auto-generated. Do not edit!


(when (boundp 'my_car_yolo::ImuProcessed)
  (if (not (find-package "MY_CAR_YOLO"))
    (make-package "MY_CAR_YOLO"))
  (shadow 'ImuProcessed (find-package "MY_CAR_YOLO")))
(unless (find-package "MY_CAR_YOLO::IMUPROCESSED")
  (make-package "MY_CAR_YOLO::IMUPROCESSED"))

(in-package "ROS")
;;//! \htmlinclude ImuProcessed.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass my_car_yolo::ImuProcessed
  :super ros::object
  :slots (_header _yaw _yaw_filtered _yaw_rate _gyro_bias_x _gyro_bias_y _gyro_bias_z _accel_x _accel_y _accel_z ))

(defmethod my_car_yolo::ImuProcessed
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:yaw __yaw) 0.0)
    ((:yaw_filtered __yaw_filtered) 0.0)
    ((:yaw_rate __yaw_rate) 0.0)
    ((:gyro_bias_x __gyro_bias_x) 0.0)
    ((:gyro_bias_y __gyro_bias_y) 0.0)
    ((:gyro_bias_z __gyro_bias_z) 0.0)
    ((:accel_x __accel_x) 0.0)
    ((:accel_y __accel_y) 0.0)
    ((:accel_z __accel_z) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _yaw (float __yaw))
   (setq _yaw_filtered (float __yaw_filtered))
   (setq _yaw_rate (float __yaw_rate))
   (setq _gyro_bias_x (float __gyro_bias_x))
   (setq _gyro_bias_y (float __gyro_bias_y))
   (setq _gyro_bias_z (float __gyro_bias_z))
   (setq _accel_x (float __accel_x))
   (setq _accel_y (float __accel_y))
   (setq _accel_z (float __accel_z))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:yaw_filtered
   (&optional __yaw_filtered)
   (if __yaw_filtered (setq _yaw_filtered __yaw_filtered)) _yaw_filtered)
  (:yaw_rate
   (&optional __yaw_rate)
   (if __yaw_rate (setq _yaw_rate __yaw_rate)) _yaw_rate)
  (:gyro_bias_x
   (&optional __gyro_bias_x)
   (if __gyro_bias_x (setq _gyro_bias_x __gyro_bias_x)) _gyro_bias_x)
  (:gyro_bias_y
   (&optional __gyro_bias_y)
   (if __gyro_bias_y (setq _gyro_bias_y __gyro_bias_y)) _gyro_bias_y)
  (:gyro_bias_z
   (&optional __gyro_bias_z)
   (if __gyro_bias_z (setq _gyro_bias_z __gyro_bias_z)) _gyro_bias_z)
  (:accel_x
   (&optional __accel_x)
   (if __accel_x (setq _accel_x __accel_x)) _accel_x)
  (:accel_y
   (&optional __accel_y)
   (if __accel_y (setq _accel_y __accel_y)) _accel_y)
  (:accel_z
   (&optional __accel_z)
   (if __accel_z (setq _accel_z __accel_z)) _accel_z)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _yaw
    8
    ;; float64 _yaw_filtered
    8
    ;; float64 _yaw_rate
    8
    ;; float64 _gyro_bias_x
    8
    ;; float64 _gyro_bias_y
    8
    ;; float64 _gyro_bias_z
    8
    ;; float64 _accel_x
    8
    ;; float64 _accel_y
    8
    ;; float64 _accel_z
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw_filtered
       (sys::poke _yaw_filtered (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw_rate
       (sys::poke _yaw_rate (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _gyro_bias_x
       (sys::poke _gyro_bias_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _gyro_bias_y
       (sys::poke _gyro_bias_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _gyro_bias_z
       (sys::poke _gyro_bias_z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _accel_x
       (sys::poke _accel_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _accel_y
       (sys::poke _accel_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _accel_z
       (sys::poke _accel_z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _yaw
     (setq _yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw_filtered
     (setq _yaw_filtered (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw_rate
     (setq _yaw_rate (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _gyro_bias_x
     (setq _gyro_bias_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _gyro_bias_y
     (setq _gyro_bias_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _gyro_bias_z
     (setq _gyro_bias_z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _accel_x
     (setq _accel_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _accel_y
     (setq _accel_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _accel_z
     (setq _accel_z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get my_car_yolo::ImuProcessed :md5sum-) "f8eb09331f7dc4e2c7e0240f0df9da14")
(setf (get my_car_yolo::ImuProcessed :datatype-) "my_car_yolo/ImuProcessed")
(setf (get my_car_yolo::ImuProcessed :definition-)
      "# ImuProcessed.msg
# 处理后的IMU数据

std_msgs/Header header
float64 yaw                    # 航向角 (度，0-360，已处理360°跳变)
float64 yaw_filtered           # 滤波后的航向角
float64 yaw_rate               # 角速度 (rad/s)
float64 gyro_bias_x            # X轴陀螺仪零偏
float64 gyro_bias_y            # Y轴陀螺仪零偏
float64 gyro_bias_z            # Z轴陀螺仪零偏
float64 accel_x                # X轴加速度 (m/s²)
float64 accel_y                # Y轴加速度 (m/s²)
float64 accel_z                # Z轴加速度 (m/s²)

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



(provide :my_car_yolo/ImuProcessed "f8eb09331f7dc4e2c7e0240f0df9da14")


