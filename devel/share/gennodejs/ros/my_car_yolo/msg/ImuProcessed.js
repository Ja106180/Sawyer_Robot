// Auto-generated. Do not edit!

// (in-package my_car_yolo.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ImuProcessed {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.yaw = null;
      this.yaw_filtered = null;
      this.yaw_rate = null;
      this.gyro_bias_x = null;
      this.gyro_bias_y = null;
      this.gyro_bias_z = null;
      this.accel_x = null;
      this.accel_y = null;
      this.accel_z = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
      if (initObj.hasOwnProperty('yaw_filtered')) {
        this.yaw_filtered = initObj.yaw_filtered
      }
      else {
        this.yaw_filtered = 0.0;
      }
      if (initObj.hasOwnProperty('yaw_rate')) {
        this.yaw_rate = initObj.yaw_rate
      }
      else {
        this.yaw_rate = 0.0;
      }
      if (initObj.hasOwnProperty('gyro_bias_x')) {
        this.gyro_bias_x = initObj.gyro_bias_x
      }
      else {
        this.gyro_bias_x = 0.0;
      }
      if (initObj.hasOwnProperty('gyro_bias_y')) {
        this.gyro_bias_y = initObj.gyro_bias_y
      }
      else {
        this.gyro_bias_y = 0.0;
      }
      if (initObj.hasOwnProperty('gyro_bias_z')) {
        this.gyro_bias_z = initObj.gyro_bias_z
      }
      else {
        this.gyro_bias_z = 0.0;
      }
      if (initObj.hasOwnProperty('accel_x')) {
        this.accel_x = initObj.accel_x
      }
      else {
        this.accel_x = 0.0;
      }
      if (initObj.hasOwnProperty('accel_y')) {
        this.accel_y = initObj.accel_y
      }
      else {
        this.accel_y = 0.0;
      }
      if (initObj.hasOwnProperty('accel_z')) {
        this.accel_z = initObj.accel_z
      }
      else {
        this.accel_z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ImuProcessed
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float64(obj.yaw, buffer, bufferOffset);
    // Serialize message field [yaw_filtered]
    bufferOffset = _serializer.float64(obj.yaw_filtered, buffer, bufferOffset);
    // Serialize message field [yaw_rate]
    bufferOffset = _serializer.float64(obj.yaw_rate, buffer, bufferOffset);
    // Serialize message field [gyro_bias_x]
    bufferOffset = _serializer.float64(obj.gyro_bias_x, buffer, bufferOffset);
    // Serialize message field [gyro_bias_y]
    bufferOffset = _serializer.float64(obj.gyro_bias_y, buffer, bufferOffset);
    // Serialize message field [gyro_bias_z]
    bufferOffset = _serializer.float64(obj.gyro_bias_z, buffer, bufferOffset);
    // Serialize message field [accel_x]
    bufferOffset = _serializer.float64(obj.accel_x, buffer, bufferOffset);
    // Serialize message field [accel_y]
    bufferOffset = _serializer.float64(obj.accel_y, buffer, bufferOffset);
    // Serialize message field [accel_z]
    bufferOffset = _serializer.float64(obj.accel_z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ImuProcessed
    let len;
    let data = new ImuProcessed(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [yaw_filtered]
    data.yaw_filtered = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [yaw_rate]
    data.yaw_rate = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [gyro_bias_x]
    data.gyro_bias_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [gyro_bias_y]
    data.gyro_bias_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [gyro_bias_z]
    data.gyro_bias_z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [accel_x]
    data.accel_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [accel_y]
    data.accel_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [accel_z]
    data.accel_z = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 72;
  }

  static datatype() {
    // Returns string type for a message object
    return 'my_car_yolo/ImuProcessed';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f8eb09331f7dc4e2c7e0240f0df9da14';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ImuProcessed.msg
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ImuProcessed(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    if (msg.yaw_filtered !== undefined) {
      resolved.yaw_filtered = msg.yaw_filtered;
    }
    else {
      resolved.yaw_filtered = 0.0
    }

    if (msg.yaw_rate !== undefined) {
      resolved.yaw_rate = msg.yaw_rate;
    }
    else {
      resolved.yaw_rate = 0.0
    }

    if (msg.gyro_bias_x !== undefined) {
      resolved.gyro_bias_x = msg.gyro_bias_x;
    }
    else {
      resolved.gyro_bias_x = 0.0
    }

    if (msg.gyro_bias_y !== undefined) {
      resolved.gyro_bias_y = msg.gyro_bias_y;
    }
    else {
      resolved.gyro_bias_y = 0.0
    }

    if (msg.gyro_bias_z !== undefined) {
      resolved.gyro_bias_z = msg.gyro_bias_z;
    }
    else {
      resolved.gyro_bias_z = 0.0
    }

    if (msg.accel_x !== undefined) {
      resolved.accel_x = msg.accel_x;
    }
    else {
      resolved.accel_x = 0.0
    }

    if (msg.accel_y !== undefined) {
      resolved.accel_y = msg.accel_y;
    }
    else {
      resolved.accel_y = 0.0
    }

    if (msg.accel_z !== undefined) {
      resolved.accel_z = msg.accel_z;
    }
    else {
      resolved.accel_z = 0.0
    }

    return resolved;
    }
};

module.exports = ImuProcessed;
