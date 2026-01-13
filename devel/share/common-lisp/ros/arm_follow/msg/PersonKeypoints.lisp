; Auto-generated. Do not edit!


(cl:in-package arm_follow-msg)


;//! \htmlinclude PersonKeypoints.msg.html

(cl:defclass <PersonKeypoints> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (person_id
    :reader person_id
    :initarg :person_id
    :type cl:integer
    :initform 0)
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
   (bbox_area
    :reader bbox_area
    :initarg :bbox_area
    :type cl:float
    :initform 0.0)
   (bbox_confidence
    :reader bbox_confidence
    :initarg :bbox_confidence
    :type cl:float
    :initform 0.0)
   (right_wrist_x
    :reader right_wrist_x
    :initarg :right_wrist_x
    :type cl:float
    :initform 0.0)
   (right_wrist_y
    :reader right_wrist_y
    :initarg :right_wrist_y
    :type cl:float
    :initform 0.0)
   (right_wrist_conf
    :reader right_wrist_conf
    :initarg :right_wrist_conf
    :type cl:float
    :initform 0.0)
   (right_elbow_x
    :reader right_elbow_x
    :initarg :right_elbow_x
    :type cl:float
    :initform 0.0)
   (right_elbow_y
    :reader right_elbow_y
    :initarg :right_elbow_y
    :type cl:float
    :initform 0.0)
   (right_elbow_conf
    :reader right_elbow_conf
    :initarg :right_elbow_conf
    :type cl:float
    :initform 0.0)
   (right_shoulder_x
    :reader right_shoulder_x
    :initarg :right_shoulder_x
    :type cl:float
    :initform 0.0)
   (right_shoulder_y
    :reader right_shoulder_y
    :initarg :right_shoulder_y
    :type cl:float
    :initform 0.0)
   (right_shoulder_conf
    :reader right_shoulder_conf
    :initarg :right_shoulder_conf
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

(cl:defclass PersonKeypoints (<PersonKeypoints>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PersonKeypoints>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PersonKeypoints)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name arm_follow-msg:<PersonKeypoints> is deprecated: use arm_follow-msg:PersonKeypoints instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PersonKeypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_follow-msg:header-val is deprecated.  Use arm_follow-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'person_id-val :lambda-list '(m))
(cl:defmethod person_id-val ((m <PersonKeypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_follow-msg:person_id-val is deprecated.  Use arm_follow-msg:person_id instead.")
  (person_id m))

(cl:ensure-generic-function 'bbox_x-val :lambda-list '(m))
(cl:defmethod bbox_x-val ((m <PersonKeypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_follow-msg:bbox_x-val is deprecated.  Use arm_follow-msg:bbox_x instead.")
  (bbox_x m))

(cl:ensure-generic-function 'bbox_y-val :lambda-list '(m))
(cl:defmethod bbox_y-val ((m <PersonKeypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_follow-msg:bbox_y-val is deprecated.  Use arm_follow-msg:bbox_y instead.")
  (bbox_y m))

(cl:ensure-generic-function 'bbox_width-val :lambda-list '(m))
(cl:defmethod bbox_width-val ((m <PersonKeypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_follow-msg:bbox_width-val is deprecated.  Use arm_follow-msg:bbox_width instead.")
  (bbox_width m))

(cl:ensure-generic-function 'bbox_height-val :lambda-list '(m))
(cl:defmethod bbox_height-val ((m <PersonKeypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_follow-msg:bbox_height-val is deprecated.  Use arm_follow-msg:bbox_height instead.")
  (bbox_height m))

(cl:ensure-generic-function 'bbox_area-val :lambda-list '(m))
(cl:defmethod bbox_area-val ((m <PersonKeypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_follow-msg:bbox_area-val is deprecated.  Use arm_follow-msg:bbox_area instead.")
  (bbox_area m))

(cl:ensure-generic-function 'bbox_confidence-val :lambda-list '(m))
(cl:defmethod bbox_confidence-val ((m <PersonKeypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_follow-msg:bbox_confidence-val is deprecated.  Use arm_follow-msg:bbox_confidence instead.")
  (bbox_confidence m))

(cl:ensure-generic-function 'right_wrist_x-val :lambda-list '(m))
(cl:defmethod right_wrist_x-val ((m <PersonKeypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_follow-msg:right_wrist_x-val is deprecated.  Use arm_follow-msg:right_wrist_x instead.")
  (right_wrist_x m))

(cl:ensure-generic-function 'right_wrist_y-val :lambda-list '(m))
(cl:defmethod right_wrist_y-val ((m <PersonKeypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_follow-msg:right_wrist_y-val is deprecated.  Use arm_follow-msg:right_wrist_y instead.")
  (right_wrist_y m))

(cl:ensure-generic-function 'right_wrist_conf-val :lambda-list '(m))
(cl:defmethod right_wrist_conf-val ((m <PersonKeypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_follow-msg:right_wrist_conf-val is deprecated.  Use arm_follow-msg:right_wrist_conf instead.")
  (right_wrist_conf m))

(cl:ensure-generic-function 'right_elbow_x-val :lambda-list '(m))
(cl:defmethod right_elbow_x-val ((m <PersonKeypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_follow-msg:right_elbow_x-val is deprecated.  Use arm_follow-msg:right_elbow_x instead.")
  (right_elbow_x m))

(cl:ensure-generic-function 'right_elbow_y-val :lambda-list '(m))
(cl:defmethod right_elbow_y-val ((m <PersonKeypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_follow-msg:right_elbow_y-val is deprecated.  Use arm_follow-msg:right_elbow_y instead.")
  (right_elbow_y m))

(cl:ensure-generic-function 'right_elbow_conf-val :lambda-list '(m))
(cl:defmethod right_elbow_conf-val ((m <PersonKeypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_follow-msg:right_elbow_conf-val is deprecated.  Use arm_follow-msg:right_elbow_conf instead.")
  (right_elbow_conf m))

(cl:ensure-generic-function 'right_shoulder_x-val :lambda-list '(m))
(cl:defmethod right_shoulder_x-val ((m <PersonKeypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_follow-msg:right_shoulder_x-val is deprecated.  Use arm_follow-msg:right_shoulder_x instead.")
  (right_shoulder_x m))

(cl:ensure-generic-function 'right_shoulder_y-val :lambda-list '(m))
(cl:defmethod right_shoulder_y-val ((m <PersonKeypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_follow-msg:right_shoulder_y-val is deprecated.  Use arm_follow-msg:right_shoulder_y instead.")
  (right_shoulder_y m))

(cl:ensure-generic-function 'right_shoulder_conf-val :lambda-list '(m))
(cl:defmethod right_shoulder_conf-val ((m <PersonKeypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_follow-msg:right_shoulder_conf-val is deprecated.  Use arm_follow-msg:right_shoulder_conf instead.")
  (right_shoulder_conf m))

(cl:ensure-generic-function 'image_width-val :lambda-list '(m))
(cl:defmethod image_width-val ((m <PersonKeypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_follow-msg:image_width-val is deprecated.  Use arm_follow-msg:image_width instead.")
  (image_width m))

(cl:ensure-generic-function 'image_height-val :lambda-list '(m))
(cl:defmethod image_height-val ((m <PersonKeypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_follow-msg:image_height-val is deprecated.  Use arm_follow-msg:image_height instead.")
  (image_height m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PersonKeypoints>) ostream)
  "Serializes a message object of type '<PersonKeypoints>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'person_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'person_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'person_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'person_id)) ostream)
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'bbox_area))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'bbox_confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_wrist_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_wrist_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_wrist_conf))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_elbow_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_elbow_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_elbow_conf))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_shoulder_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_shoulder_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_shoulder_conf))))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PersonKeypoints>) istream)
  "Deserializes a message object of type '<PersonKeypoints>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'person_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'person_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'person_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'person_id)) (cl:read-byte istream))
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bbox_area) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bbox_confidence) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_wrist_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_wrist_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_wrist_conf) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_elbow_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_elbow_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_elbow_conf) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_shoulder_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_shoulder_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_shoulder_conf) (roslisp-utils:decode-single-float-bits bits)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PersonKeypoints>)))
  "Returns string type for a message object of type '<PersonKeypoints>"
  "arm_follow/PersonKeypoints")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PersonKeypoints)))
  "Returns string type for a message object of type 'PersonKeypoints"
  "arm_follow/PersonKeypoints")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PersonKeypoints>)))
  "Returns md5sum for a message object of type '<PersonKeypoints>"
  "b8aef0b6f60577e67360824121ee7a78")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PersonKeypoints)))
  "Returns md5sum for a message object of type 'PersonKeypoints"
  "b8aef0b6f60577e67360824121ee7a78")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PersonKeypoints>)))
  "Returns full string definition for message of type '<PersonKeypoints>"
  (cl:format cl:nil "# PersonKeypoints.msg~%# 一个人体的关键点数据~%~%std_msgs/Header header~%uint32 person_id          # 人员ID~%float32 bbox_x            # 检测框左上角x~%float32 bbox_y            # 检测框左上角y~%float32 bbox_width        # 检测框宽度~%float32 bbox_height       # 检测框高度~%float32 bbox_area         # 检测框面积~%float32 bbox_confidence   # 检测框置信度（YOLO显示的置信度）~%~%# 右手关键点（COCO格式中通常是右手腕，索引10）~%float32 right_wrist_x     # 右手腕x坐标~%float32 right_wrist_y     # 右手腕y坐标~%float32 right_wrist_conf  # 右手腕置信度~%~%# 右手肘（索引8）~%float32 right_elbow_x~%float32 right_elbow_y~%float32 right_elbow_conf~%~%# 右肩膀（用于判断右手是否伸出）~%float32 right_shoulder_x~%float32 right_shoulder_y~%float32 right_shoulder_conf~%~%# 图像尺寸（用于计算角度）~%uint32 image_width~%uint32 image_height~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PersonKeypoints)))
  "Returns full string definition for message of type 'PersonKeypoints"
  (cl:format cl:nil "# PersonKeypoints.msg~%# 一个人体的关键点数据~%~%std_msgs/Header header~%uint32 person_id          # 人员ID~%float32 bbox_x            # 检测框左上角x~%float32 bbox_y            # 检测框左上角y~%float32 bbox_width        # 检测框宽度~%float32 bbox_height       # 检测框高度~%float32 bbox_area         # 检测框面积~%float32 bbox_confidence   # 检测框置信度（YOLO显示的置信度）~%~%# 右手关键点（COCO格式中通常是右手腕，索引10）~%float32 right_wrist_x     # 右手腕x坐标~%float32 right_wrist_y     # 右手腕y坐标~%float32 right_wrist_conf  # 右手腕置信度~%~%# 右手肘（索引8）~%float32 right_elbow_x~%float32 right_elbow_y~%float32 right_elbow_conf~%~%# 右肩膀（用于判断右手是否伸出）~%float32 right_shoulder_x~%float32 right_shoulder_y~%float32 right_shoulder_conf~%~%# 图像尺寸（用于计算角度）~%uint32 image_width~%uint32 image_height~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PersonKeypoints>))
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
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PersonKeypoints>))
  "Converts a ROS message object to a list"
  (cl:list 'PersonKeypoints
    (cl:cons ':header (header msg))
    (cl:cons ':person_id (person_id msg))
    (cl:cons ':bbox_x (bbox_x msg))
    (cl:cons ':bbox_y (bbox_y msg))
    (cl:cons ':bbox_width (bbox_width msg))
    (cl:cons ':bbox_height (bbox_height msg))
    (cl:cons ':bbox_area (bbox_area msg))
    (cl:cons ':bbox_confidence (bbox_confidence msg))
    (cl:cons ':right_wrist_x (right_wrist_x msg))
    (cl:cons ':right_wrist_y (right_wrist_y msg))
    (cl:cons ':right_wrist_conf (right_wrist_conf msg))
    (cl:cons ':right_elbow_x (right_elbow_x msg))
    (cl:cons ':right_elbow_y (right_elbow_y msg))
    (cl:cons ':right_elbow_conf (right_elbow_conf msg))
    (cl:cons ':right_shoulder_x (right_shoulder_x msg))
    (cl:cons ':right_shoulder_y (right_shoulder_y msg))
    (cl:cons ':right_shoulder_conf (right_shoulder_conf msg))
    (cl:cons ':image_width (image_width msg))
    (cl:cons ':image_height (image_height msg))
))
