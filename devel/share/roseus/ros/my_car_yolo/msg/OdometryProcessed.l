;; Auto-generated. Do not edit!


(when (boundp 'my_car_yolo::OdometryProcessed)
  (if (not (find-package "MY_CAR_YOLO"))
    (make-package "MY_CAR_YOLO"))
  (shadow 'OdometryProcessed (find-package "MY_CAR_YOLO")))
(unless (find-package "MY_CAR_YOLO::ODOMETRYPROCESSED")
  (make-package "MY_CAR_YOLO::ODOMETRYPROCESSED"))

(in-package "ROS")
;;//! \htmlinclude OdometryProcessed.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass my_car_yolo::OdometryProcessed
  :super ros::object
  :slots (_header _child_frame_id _pose _twist _distance_left _distance_right _linear_velocity _angular_velocity _imu_yaw _fused_yaw _yaw_confidence _is_slipping _slip_ratio _encoder_errors ))

(defmethod my_car_yolo::OdometryProcessed
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:child_frame_id __child_frame_id) "")
    ((:pose __pose) (instance geometry_msgs::PoseWithCovariance :init))
    ((:twist __twist) (instance geometry_msgs::TwistWithCovariance :init))
    ((:distance_left __distance_left) 0.0)
    ((:distance_right __distance_right) 0.0)
    ((:linear_velocity __linear_velocity) 0.0)
    ((:angular_velocity __angular_velocity) 0.0)
    ((:imu_yaw __imu_yaw) 0.0)
    ((:fused_yaw __fused_yaw) 0.0)
    ((:yaw_confidence __yaw_confidence) 0.0)
    ((:is_slipping __is_slipping) nil)
    ((:slip_ratio __slip_ratio) 0.0)
    ((:encoder_errors __encoder_errors) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _child_frame_id (string __child_frame_id))
   (setq _pose __pose)
   (setq _twist __twist)
   (setq _distance_left (float __distance_left))
   (setq _distance_right (float __distance_right))
   (setq _linear_velocity (float __linear_velocity))
   (setq _angular_velocity (float __angular_velocity))
   (setq _imu_yaw (float __imu_yaw))
   (setq _fused_yaw (float __fused_yaw))
   (setq _yaw_confidence (float __yaw_confidence))
   (setq _is_slipping __is_slipping)
   (setq _slip_ratio (float __slip_ratio))
   (setq _encoder_errors (round __encoder_errors))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:child_frame_id
   (&optional __child_frame_id)
   (if __child_frame_id (setq _child_frame_id __child_frame_id)) _child_frame_id)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:twist
   (&rest __twist)
   (if (keywordp (car __twist))
       (send* _twist __twist)
     (progn
       (if __twist (setq _twist (car __twist)))
       _twist)))
  (:distance_left
   (&optional __distance_left)
   (if __distance_left (setq _distance_left __distance_left)) _distance_left)
  (:distance_right
   (&optional __distance_right)
   (if __distance_right (setq _distance_right __distance_right)) _distance_right)
  (:linear_velocity
   (&optional __linear_velocity)
   (if __linear_velocity (setq _linear_velocity __linear_velocity)) _linear_velocity)
  (:angular_velocity
   (&optional __angular_velocity)
   (if __angular_velocity (setq _angular_velocity __angular_velocity)) _angular_velocity)
  (:imu_yaw
   (&optional __imu_yaw)
   (if __imu_yaw (setq _imu_yaw __imu_yaw)) _imu_yaw)
  (:fused_yaw
   (&optional __fused_yaw)
   (if __fused_yaw (setq _fused_yaw __fused_yaw)) _fused_yaw)
  (:yaw_confidence
   (&optional __yaw_confidence)
   (if __yaw_confidence (setq _yaw_confidence __yaw_confidence)) _yaw_confidence)
  (:is_slipping
   (&optional (__is_slipping :null))
   (if (not (eq __is_slipping :null)) (setq _is_slipping __is_slipping)) _is_slipping)
  (:slip_ratio
   (&optional __slip_ratio)
   (if __slip_ratio (setq _slip_ratio __slip_ratio)) _slip_ratio)
  (:encoder_errors
   (&optional __encoder_errors)
   (if __encoder_errors (setq _encoder_errors __encoder_errors)) _encoder_errors)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _child_frame_id
    4 (length _child_frame_id)
    ;; geometry_msgs/PoseWithCovariance _pose
    (send _pose :serialization-length)
    ;; geometry_msgs/TwistWithCovariance _twist
    (send _twist :serialization-length)
    ;; float64 _distance_left
    8
    ;; float64 _distance_right
    8
    ;; float64 _linear_velocity
    8
    ;; float64 _angular_velocity
    8
    ;; float64 _imu_yaw
    8
    ;; float64 _fused_yaw
    8
    ;; float64 _yaw_confidence
    8
    ;; bool _is_slipping
    1
    ;; float64 _slip_ratio
    8
    ;; uint32 _encoder_errors
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _child_frame_id
       (write-long (length _child_frame_id) s) (princ _child_frame_id s)
     ;; geometry_msgs/PoseWithCovariance _pose
       (send _pose :serialize s)
     ;; geometry_msgs/TwistWithCovariance _twist
       (send _twist :serialize s)
     ;; float64 _distance_left
       (sys::poke _distance_left (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _distance_right
       (sys::poke _distance_right (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _linear_velocity
       (sys::poke _linear_velocity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _angular_velocity
       (sys::poke _angular_velocity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _imu_yaw
       (sys::poke _imu_yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _fused_yaw
       (sys::poke _fused_yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw_confidence
       (sys::poke _yaw_confidence (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _is_slipping
       (if _is_slipping (write-byte -1 s) (write-byte 0 s))
     ;; float64 _slip_ratio
       (sys::poke _slip_ratio (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint32 _encoder_errors
       (write-long _encoder_errors s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _child_frame_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _child_frame_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/PoseWithCovariance _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; geometry_msgs/TwistWithCovariance _twist
     (send _twist :deserialize buf ptr-) (incf ptr- (send _twist :serialization-length))
   ;; float64 _distance_left
     (setq _distance_left (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _distance_right
     (setq _distance_right (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _linear_velocity
     (setq _linear_velocity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _angular_velocity
     (setq _angular_velocity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _imu_yaw
     (setq _imu_yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _fused_yaw
     (setq _fused_yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw_confidence
     (setq _yaw_confidence (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _is_slipping
     (setq _is_slipping (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _slip_ratio
     (setq _slip_ratio (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint32 _encoder_errors
     (setq _encoder_errors (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get my_car_yolo::OdometryProcessed :md5sum-) "0a93869a001531267c92fa7909c7c4ce")
(setf (get my_car_yolo::OdometryProcessed :datatype-) "my_car_yolo/OdometryProcessed")
(setf (get my_car_yolo::OdometryProcessed :definition-)
      "# 处理后的里程计数据消息
# 包含运动学模型、航迹推演、IMU融合结果

std_msgs/Header header
string child_frame_id

# 位姿估计 (世界坐标系)
geometry_msgs/PoseWithCovariance pose

# 速度估计
geometry_msgs/TwistWithCovariance twist

# 里程计特有字段
float64 distance_left    # 左轮行驶距离 (米)
float64 distance_right   # 右轮行驶距离 (米)

# 运动学参数
float64 linear_velocity  # 当前线速度 (m/s)
float64 angular_velocity # 当前角速度 (rad/s)

# IMU融合相关
float64 imu_yaw          # IMU航向角 (用于融合)
float64 fused_yaw        # IMU+编码器融合航向角
float64 yaw_confidence   # 航向角置信度 (0-1)

# 状态标志
bool is_slipping         # 是否检测到打滑
float64 slip_ratio       # 打滑比例 (0-1)
uint32 encoder_errors    # 编码器错误计数

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
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/TwistWithCovariance
# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3 linear
Vector3 angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :my_car_yolo/OdometryProcessed "0a93869a001531267c92fa7909c7c4ce")


