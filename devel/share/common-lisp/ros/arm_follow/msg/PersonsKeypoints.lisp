; Auto-generated. Do not edit!


(cl:in-package arm_follow-msg)


;//! \htmlinclude PersonsKeypoints.msg.html

(cl:defclass <PersonsKeypoints> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (persons
    :reader persons
    :initarg :persons
    :type (cl:vector arm_follow-msg:PersonKeypoints)
   :initform (cl:make-array 0 :element-type 'arm_follow-msg:PersonKeypoints :initial-element (cl:make-instance 'arm_follow-msg:PersonKeypoints))))
)

(cl:defclass PersonsKeypoints (<PersonsKeypoints>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PersonsKeypoints>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PersonsKeypoints)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name arm_follow-msg:<PersonsKeypoints> is deprecated: use arm_follow-msg:PersonsKeypoints instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PersonsKeypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_follow-msg:header-val is deprecated.  Use arm_follow-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'persons-val :lambda-list '(m))
(cl:defmethod persons-val ((m <PersonsKeypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_follow-msg:persons-val is deprecated.  Use arm_follow-msg:persons instead.")
  (persons m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PersonsKeypoints>) ostream)
  "Serializes a message object of type '<PersonsKeypoints>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'persons))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'persons))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PersonsKeypoints>) istream)
  "Deserializes a message object of type '<PersonsKeypoints>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'persons) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'persons)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'arm_follow-msg:PersonKeypoints))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PersonsKeypoints>)))
  "Returns string type for a message object of type '<PersonsKeypoints>"
  "arm_follow/PersonsKeypoints")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PersonsKeypoints)))
  "Returns string type for a message object of type 'PersonsKeypoints"
  "arm_follow/PersonsKeypoints")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PersonsKeypoints>)))
  "Returns md5sum for a message object of type '<PersonsKeypoints>"
  "2cc6b7f8f80dc708d5b607449f5f648f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PersonsKeypoints)))
  "Returns md5sum for a message object of type 'PersonsKeypoints"
  "2cc6b7f8f80dc708d5b607449f5f648f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PersonsKeypoints>)))
  "Returns full string definition for message of type '<PersonsKeypoints>"
  (cl:format cl:nil "# PersonsKeypoints.msg~%# 多个人体的关键点数据列表~%~%std_msgs/Header header~%PersonKeypoints[] persons  # 所有人体的关键点列表~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: arm_follow/PersonKeypoints~%# PersonKeypoints.msg~%# 一个人体的关键点数据~%~%std_msgs/Header header~%uint32 person_id          # 人员ID~%float32 bbox_x            # 检测框左上角x~%float32 bbox_y            # 检测框左上角y~%float32 bbox_width        # 检测框宽度~%float32 bbox_height       # 检测框高度~%float32 bbox_area         # 检测框面积~%float32 bbox_confidence   # 检测框置信度（YOLO显示的置信度）~%~%# 右手关键点（COCO格式中通常是右手腕，索引10）~%float32 right_wrist_x     # 右手腕x坐标~%float32 right_wrist_y     # 右手腕y坐标~%float32 right_wrist_conf  # 右手腕置信度~%~%# 右手肘（索引8）~%float32 right_elbow_x~%float32 right_elbow_y~%float32 right_elbow_conf~%~%# 右肩膀（用于判断右手是否伸出）~%float32 right_shoulder_x~%float32 right_shoulder_y~%float32 right_shoulder_conf~%~%# 图像尺寸（用于计算角度）~%uint32 image_width~%uint32 image_height~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PersonsKeypoints)))
  "Returns full string definition for message of type 'PersonsKeypoints"
  (cl:format cl:nil "# PersonsKeypoints.msg~%# 多个人体的关键点数据列表~%~%std_msgs/Header header~%PersonKeypoints[] persons  # 所有人体的关键点列表~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: arm_follow/PersonKeypoints~%# PersonKeypoints.msg~%# 一个人体的关键点数据~%~%std_msgs/Header header~%uint32 person_id          # 人员ID~%float32 bbox_x            # 检测框左上角x~%float32 bbox_y            # 检测框左上角y~%float32 bbox_width        # 检测框宽度~%float32 bbox_height       # 检测框高度~%float32 bbox_area         # 检测框面积~%float32 bbox_confidence   # 检测框置信度（YOLO显示的置信度）~%~%# 右手关键点（COCO格式中通常是右手腕，索引10）~%float32 right_wrist_x     # 右手腕x坐标~%float32 right_wrist_y     # 右手腕y坐标~%float32 right_wrist_conf  # 右手腕置信度~%~%# 右手肘（索引8）~%float32 right_elbow_x~%float32 right_elbow_y~%float32 right_elbow_conf~%~%# 右肩膀（用于判断右手是否伸出）~%float32 right_shoulder_x~%float32 right_shoulder_y~%float32 right_shoulder_conf~%~%# 图像尺寸（用于计算角度）~%uint32 image_width~%uint32 image_height~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PersonsKeypoints>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'persons) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PersonsKeypoints>))
  "Converts a ROS message object to a list"
  (cl:list 'PersonsKeypoints
    (cl:cons ':header (header msg))
    (cl:cons ':persons (persons msg))
))
