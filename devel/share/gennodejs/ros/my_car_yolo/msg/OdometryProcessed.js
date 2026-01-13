// Auto-generated. Do not edit!

// (in-package my_car_yolo.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class OdometryProcessed {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.child_frame_id = null;
      this.pose = null;
      this.twist = null;
      this.distance_left = null;
      this.distance_right = null;
      this.linear_velocity = null;
      this.angular_velocity = null;
      this.imu_yaw = null;
      this.fused_yaw = null;
      this.yaw_confidence = null;
      this.is_slipping = null;
      this.slip_ratio = null;
      this.encoder_errors = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('child_frame_id')) {
        this.child_frame_id = initObj.child_frame_id
      }
      else {
        this.child_frame_id = '';
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.PoseWithCovariance();
      }
      if (initObj.hasOwnProperty('twist')) {
        this.twist = initObj.twist
      }
      else {
        this.twist = new geometry_msgs.msg.TwistWithCovariance();
      }
      if (initObj.hasOwnProperty('distance_left')) {
        this.distance_left = initObj.distance_left
      }
      else {
        this.distance_left = 0.0;
      }
      if (initObj.hasOwnProperty('distance_right')) {
        this.distance_right = initObj.distance_right
      }
      else {
        this.distance_right = 0.0;
      }
      if (initObj.hasOwnProperty('linear_velocity')) {
        this.linear_velocity = initObj.linear_velocity
      }
      else {
        this.linear_velocity = 0.0;
      }
      if (initObj.hasOwnProperty('angular_velocity')) {
        this.angular_velocity = initObj.angular_velocity
      }
      else {
        this.angular_velocity = 0.0;
      }
      if (initObj.hasOwnProperty('imu_yaw')) {
        this.imu_yaw = initObj.imu_yaw
      }
      else {
        this.imu_yaw = 0.0;
      }
      if (initObj.hasOwnProperty('fused_yaw')) {
        this.fused_yaw = initObj.fused_yaw
      }
      else {
        this.fused_yaw = 0.0;
      }
      if (initObj.hasOwnProperty('yaw_confidence')) {
        this.yaw_confidence = initObj.yaw_confidence
      }
      else {
        this.yaw_confidence = 0.0;
      }
      if (initObj.hasOwnProperty('is_slipping')) {
        this.is_slipping = initObj.is_slipping
      }
      else {
        this.is_slipping = false;
      }
      if (initObj.hasOwnProperty('slip_ratio')) {
        this.slip_ratio = initObj.slip_ratio
      }
      else {
        this.slip_ratio = 0.0;
      }
      if (initObj.hasOwnProperty('encoder_errors')) {
        this.encoder_errors = initObj.encoder_errors
      }
      else {
        this.encoder_errors = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OdometryProcessed
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [child_frame_id]
    bufferOffset = _serializer.string(obj.child_frame_id, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.PoseWithCovariance.serialize(obj.pose, buffer, bufferOffset);
    // Serialize message field [twist]
    bufferOffset = geometry_msgs.msg.TwistWithCovariance.serialize(obj.twist, buffer, bufferOffset);
    // Serialize message field [distance_left]
    bufferOffset = _serializer.float64(obj.distance_left, buffer, bufferOffset);
    // Serialize message field [distance_right]
    bufferOffset = _serializer.float64(obj.distance_right, buffer, bufferOffset);
    // Serialize message field [linear_velocity]
    bufferOffset = _serializer.float64(obj.linear_velocity, buffer, bufferOffset);
    // Serialize message field [angular_velocity]
    bufferOffset = _serializer.float64(obj.angular_velocity, buffer, bufferOffset);
    // Serialize message field [imu_yaw]
    bufferOffset = _serializer.float64(obj.imu_yaw, buffer, bufferOffset);
    // Serialize message field [fused_yaw]
    bufferOffset = _serializer.float64(obj.fused_yaw, buffer, bufferOffset);
    // Serialize message field [yaw_confidence]
    bufferOffset = _serializer.float64(obj.yaw_confidence, buffer, bufferOffset);
    // Serialize message field [is_slipping]
    bufferOffset = _serializer.bool(obj.is_slipping, buffer, bufferOffset);
    // Serialize message field [slip_ratio]
    bufferOffset = _serializer.float64(obj.slip_ratio, buffer, bufferOffset);
    // Serialize message field [encoder_errors]
    bufferOffset = _serializer.uint32(obj.encoder_errors, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OdometryProcessed
    let len;
    let data = new OdometryProcessed(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [child_frame_id]
    data.child_frame_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.PoseWithCovariance.deserialize(buffer, bufferOffset);
    // Deserialize message field [twist]
    data.twist = geometry_msgs.msg.TwistWithCovariance.deserialize(buffer, bufferOffset);
    // Deserialize message field [distance_left]
    data.distance_left = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [distance_right]
    data.distance_right = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [linear_velocity]
    data.linear_velocity = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [angular_velocity]
    data.angular_velocity = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [imu_yaw]
    data.imu_yaw = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [fused_yaw]
    data.fused_yaw = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [yaw_confidence]
    data.yaw_confidence = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [is_slipping]
    data.is_slipping = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [slip_ratio]
    data.slip_ratio = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [encoder_errors]
    data.encoder_errors = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.child_frame_id);
    return length + 753;
  }

  static datatype() {
    // Returns string type for a message object
    return 'my_car_yolo/OdometryProcessed';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0a93869a001531267c92fa7909c7c4ce';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 处理后的里程计数据消息
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new OdometryProcessed(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.child_frame_id !== undefined) {
      resolved.child_frame_id = msg.child_frame_id;
    }
    else {
      resolved.child_frame_id = ''
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.PoseWithCovariance.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.PoseWithCovariance()
    }

    if (msg.twist !== undefined) {
      resolved.twist = geometry_msgs.msg.TwistWithCovariance.Resolve(msg.twist)
    }
    else {
      resolved.twist = new geometry_msgs.msg.TwistWithCovariance()
    }

    if (msg.distance_left !== undefined) {
      resolved.distance_left = msg.distance_left;
    }
    else {
      resolved.distance_left = 0.0
    }

    if (msg.distance_right !== undefined) {
      resolved.distance_right = msg.distance_right;
    }
    else {
      resolved.distance_right = 0.0
    }

    if (msg.linear_velocity !== undefined) {
      resolved.linear_velocity = msg.linear_velocity;
    }
    else {
      resolved.linear_velocity = 0.0
    }

    if (msg.angular_velocity !== undefined) {
      resolved.angular_velocity = msg.angular_velocity;
    }
    else {
      resolved.angular_velocity = 0.0
    }

    if (msg.imu_yaw !== undefined) {
      resolved.imu_yaw = msg.imu_yaw;
    }
    else {
      resolved.imu_yaw = 0.0
    }

    if (msg.fused_yaw !== undefined) {
      resolved.fused_yaw = msg.fused_yaw;
    }
    else {
      resolved.fused_yaw = 0.0
    }

    if (msg.yaw_confidence !== undefined) {
      resolved.yaw_confidence = msg.yaw_confidence;
    }
    else {
      resolved.yaw_confidence = 0.0
    }

    if (msg.is_slipping !== undefined) {
      resolved.is_slipping = msg.is_slipping;
    }
    else {
      resolved.is_slipping = false
    }

    if (msg.slip_ratio !== undefined) {
      resolved.slip_ratio = msg.slip_ratio;
    }
    else {
      resolved.slip_ratio = 0.0
    }

    if (msg.encoder_errors !== undefined) {
      resolved.encoder_errors = msg.encoder_errors;
    }
    else {
      resolved.encoder_errors = 0
    }

    return resolved;
    }
};

module.exports = OdometryProcessed;
