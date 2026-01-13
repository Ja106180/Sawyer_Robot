; Auto-generated. Do not edit!


(cl:in-package my_car_yolo-msg)


;//! \htmlinclude WheelEncoders.msg.html

(cl:defclass <WheelEncoders> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (left_pulses
    :reader left_pulses
    :initarg :left_pulses
    :type cl:integer
    :initform 0)
   (right_pulses
    :reader right_pulses
    :initarg :right_pulses
    :type cl:integer
    :initform 0)
   (left_velocity
    :reader left_velocity
    :initarg :left_velocity
    :type cl:float
    :initform 0.0)
   (right_velocity
    :reader right_velocity
    :initarg :right_velocity
    :type cl:float
    :initform 0.0))
)

(cl:defclass WheelEncoders (<WheelEncoders>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WheelEncoders>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WheelEncoders)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_car_yolo-msg:<WheelEncoders> is deprecated: use my_car_yolo-msg:WheelEncoders instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <WheelEncoders>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_car_yolo-msg:header-val is deprecated.  Use my_car_yolo-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'left_pulses-val :lambda-list '(m))
(cl:defmethod left_pulses-val ((m <WheelEncoders>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_car_yolo-msg:left_pulses-val is deprecated.  Use my_car_yolo-msg:left_pulses instead.")
  (left_pulses m))

(cl:ensure-generic-function 'right_pulses-val :lambda-list '(m))
(cl:defmethod right_pulses-val ((m <WheelEncoders>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_car_yolo-msg:right_pulses-val is deprecated.  Use my_car_yolo-msg:right_pulses instead.")
  (right_pulses m))

(cl:ensure-generic-function 'left_velocity-val :lambda-list '(m))
(cl:defmethod left_velocity-val ((m <WheelEncoders>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_car_yolo-msg:left_velocity-val is deprecated.  Use my_car_yolo-msg:left_velocity instead.")
  (left_velocity m))

(cl:ensure-generic-function 'right_velocity-val :lambda-list '(m))
(cl:defmethod right_velocity-val ((m <WheelEncoders>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_car_yolo-msg:right_velocity-val is deprecated.  Use my_car_yolo-msg:right_velocity instead.")
  (right_velocity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WheelEncoders>) ostream)
  "Serializes a message object of type '<WheelEncoders>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'left_pulses)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'right_pulses)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'left_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'right_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WheelEncoders>) istream)
  "Deserializes a message object of type '<WheelEncoders>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'left_pulses) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'right_pulses) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_velocity) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_velocity) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WheelEncoders>)))
  "Returns string type for a message object of type '<WheelEncoders>"
  "my_car_yolo/WheelEncoders")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WheelEncoders)))
  "Returns string type for a message object of type 'WheelEncoders"
  "my_car_yolo/WheelEncoders")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WheelEncoders>)))
  "Returns md5sum for a message object of type '<WheelEncoders>"
  "04b36400bf0a0e74c73173ddcaa70f42")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WheelEncoders)))
  "Returns md5sum for a message object of type 'WheelEncoders"
  "04b36400bf0a0e74c73173ddcaa70f42")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WheelEncoders>)))
  "Returns full string definition for message of type '<WheelEncoders>"
  (cl:format cl:nil "# WheelEncoders.msg~%# 轮子编码器原始数据~%~%std_msgs/Header header~%int32 left_pulses              # 左轮编码器脉冲数~%int32 right_pulses             # 右轮编码器脉冲数~%float64 left_velocity          # 左轮速度 (m/s)~%float64 right_velocity         # 右轮速度 (m/s)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WheelEncoders)))
  "Returns full string definition for message of type 'WheelEncoders"
  (cl:format cl:nil "# WheelEncoders.msg~%# 轮子编码器原始数据~%~%std_msgs/Header header~%int32 left_pulses              # 左轮编码器脉冲数~%int32 right_pulses             # 右轮编码器脉冲数~%float64 left_velocity          # 左轮速度 (m/s)~%float64 right_velocity         # 右轮速度 (m/s)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WheelEncoders>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WheelEncoders>))
  "Converts a ROS message object to a list"
  (cl:list 'WheelEncoders
    (cl:cons ':header (header msg))
    (cl:cons ':left_pulses (left_pulses msg))
    (cl:cons ':right_pulses (right_pulses msg))
    (cl:cons ':left_velocity (left_velocity msg))
    (cl:cons ':right_velocity (right_velocity msg))
))
