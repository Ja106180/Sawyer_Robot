; Auto-generated. Do not edit!


(cl:in-package my_car_yolo-msg)


;//! \htmlinclude ObjectDetection.msg.html

(cl:defclass <ObjectDetection> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (object_id
    :reader object_id
    :initarg :object_id
    :type cl:integer
    :initform 0)
   (center_x
    :reader center_x
    :initarg :center_x
    :type cl:float
    :initform 0.0)
   (center_y
    :reader center_y
    :initarg :center_y
    :type cl:float
    :initform 0.0)
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:float
    :initform 0.0)
   (bbox_x
    :reader bbox_x
    :initarg :bbox_x
    :type cl:float
    :initform 0.0)
   (bbox_y
    :reader bbox_y
    :initarg :bbox_y
    :type cl:float
    :initform 0.0)
   (bbox_width
    :reader bbox_width
    :initarg :bbox_width
    :type cl:float
    :initform 0.0)
   (bbox_height
    :reader bbox_height
    :initarg :bbox_height
    :type cl:float
    :initform 0.0)
   (image_width
    :reader image_width
    :initarg :image_width
    :type cl:integer
    :initform 0)
   (image_height
    :reader image_height
    :initarg :image_height
    :type cl:integer
    :initform 0))
)

(cl:defclass ObjectDetection (<ObjectDetection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObjectDetection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObjectDetection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_car_yolo-msg:<ObjectDetection> is deprecated: use my_car_yolo-msg:ObjectDetection instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ObjectDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_car_yolo-msg:header-val is deprecated.  Use my_car_yolo-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'object_id-val :lambda-list '(m))
(cl:defmethod object_id-val ((m <ObjectDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_car_yolo-msg:object_id-val is deprecated.  Use my_car_yolo-msg:object_id instead.")
  (object_id m))

(cl:ensure-generic-function 'center_x-val :lambda-list '(m))
(cl:defmethod center_x-val ((m <ObjectDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_car_yolo-msg:center_x-val is deprecated.  Use my_car_yolo-msg:center_x instead.")
  (center_x m))

(cl:ensure-generic-function 'center_y-val :lambda-list '(m))
(cl:defmethod center_y-val ((m <ObjectDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_car_yolo-msg:center_y-val is deprecated.  Use my_car_yolo-msg:center_y instead.")
  (center_y m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <ObjectDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_car_yolo-msg:confidence-val is deprecated.  Use my_car_yolo-msg:confidence instead.")
  (confidence m))

(cl:ensure-generic-function 'bbox_x-val :lambda-list '(m))
(cl:defmethod bbox_x-val ((m <ObjectDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_car_yolo-msg:bbox_x-val is deprecated.  Use my_car_yolo-msg:bbox_x instead.")
  (bbox_x m))

(cl:ensure-generic-function 'bbox_y-val :lambda-list '(m))
(cl:defmethod bbox_y-val ((m <ObjectDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_car_yolo-msg:bbox_y-val is deprecated.  Use my_car_yolo-msg:bbox_y instead.")
  (bbox_y m))

(cl:ensure-generic-function 'bbox_width-val :lambda-list '(m))
(cl:defmethod bbox_width-val ((m <ObjectDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_car_yolo-msg:bbox_width-val is deprecated.  Use my_car_yolo-msg:bbox_width instead.")
  (bbox_width m))

(cl:ensure-generic-function 'bbox_height-val :lambda-list '(m))
(cl:defmethod bbox_height-val ((m <ObjectDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_car_yolo-msg:bbox_height-val is deprecated.  Use my_car_yolo-msg:bbox_height instead.")
  (bbox_height m))

(cl:ensure-generic-function 'image_width-val :lambda-list '(m))
(cl:defmethod image_width-val ((m <ObjectDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_car_yolo-msg:image_width-val is deprecated.  Use my_car_yolo-msg:image_width instead.")
  (image_width m))

(cl:ensure-generic-function 'image_height-val :lambda-list '(m))
(cl:defmethod image_height-val ((m <ObjectDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_car_yolo-msg:image_height-val is deprecated.  Use my_car_yolo-msg:image_height instead.")
  (image_height m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObjectDetection>) ostream)
  "Serializes a message object of type '<ObjectDetection>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'object_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'object_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'object_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'object_id)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'center_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'center_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'bbox_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'bbox_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'bbox_width))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'bbox_height))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_height)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObjectDetection>) istream)
  "Deserializes a message object of type '<ObjectDetection>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'object_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'object_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'object_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'object_id)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'center_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'center_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'confidence) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bbox_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bbox_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bbox_width) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bbox_height) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_height)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObjectDetection>)))
  "Returns string type for a message object of type '<ObjectDetection>"
  "my_car_yolo/ObjectDetection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObjectDetection)))
  "Returns string type for a message object of type 'ObjectDetection"
  "my_car_yolo/ObjectDetection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObjectDetection>)))
  "Returns md5sum for a message object of type '<ObjectDetection>"
  "74b64be95b23c1ba8f1d1679b6ef3c59")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObjectDetection)))
  "Returns md5sum for a message object of type 'ObjectDetection"
  "74b64be95b23c1ba8f1d1679b6ef3c59")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObjectDetection>)))
  "Returns full string definition for message of type '<ObjectDetection>"
  (cl:format cl:nil "# ObjectDetection.msg~%# 单个物体的检测结果~%~%std_msgs/Header header~%uint32 object_id          # 物体ID (1, 2, 3...)~%float32 center_x          # 中心点x坐标（像素）~%float32 center_y          # 中心点y坐标（像素）~%float32 confidence        # 检测置信度~%float32 bbox_x            # 边界框左上角x~%float32 bbox_y            # 边界框左上角y~%float32 bbox_width        # 边界框宽度~%float32 bbox_height        # 边界框高度~%uint32 image_width        # 图像宽度~%uint32 image_height       # 图像高度~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObjectDetection)))
  "Returns full string definition for message of type 'ObjectDetection"
  (cl:format cl:nil "# ObjectDetection.msg~%# 单个物体的检测结果~%~%std_msgs/Header header~%uint32 object_id          # 物体ID (1, 2, 3...)~%float32 center_x          # 中心点x坐标（像素）~%float32 center_y          # 中心点y坐标（像素）~%float32 confidence        # 检测置信度~%float32 bbox_x            # 边界框左上角x~%float32 bbox_y            # 边界框左上角y~%float32 bbox_width        # 边界框宽度~%float32 bbox_height        # 边界框高度~%uint32 image_width        # 图像宽度~%uint32 image_height       # 图像高度~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObjectDetection>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObjectDetection>))
  "Converts a ROS message object to a list"
  (cl:list 'ObjectDetection
    (cl:cons ':header (header msg))
    (cl:cons ':object_id (object_id msg))
    (cl:cons ':center_x (center_x msg))
    (cl:cons ':center_y (center_y msg))
    (cl:cons ':confidence (confidence msg))
    (cl:cons ':bbox_x (bbox_x msg))
    (cl:cons ':bbox_y (bbox_y msg))
    (cl:cons ':bbox_width (bbox_width msg))
    (cl:cons ':bbox_height (bbox_height msg))
    (cl:cons ':image_width (image_width msg))
    (cl:cons ':image_height (image_height msg))
))
