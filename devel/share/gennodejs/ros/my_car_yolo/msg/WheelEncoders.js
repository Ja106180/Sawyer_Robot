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

class WheelEncoders {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.left_pulses = null;
      this.right_pulses = null;
      this.left_velocity = null;
      this.right_velocity = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('left_pulses')) {
        this.left_pulses = initObj.left_pulses
      }
      else {
        this.left_pulses = 0;
      }
      if (initObj.hasOwnProperty('right_pulses')) {
        this.right_pulses = initObj.right_pulses
      }
      else {
        this.right_pulses = 0;
      }
      if (initObj.hasOwnProperty('left_velocity')) {
        this.left_velocity = initObj.left_velocity
      }
      else {
        this.left_velocity = 0.0;
      }
      if (initObj.hasOwnProperty('right_velocity')) {
        this.right_velocity = initObj.right_velocity
      }
      else {
        this.right_velocity = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WheelEncoders
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [left_pulses]
    bufferOffset = _serializer.int32(obj.left_pulses, buffer, bufferOffset);
    // Serialize message field [right_pulses]
    bufferOffset = _serializer.int32(obj.right_pulses, buffer, bufferOffset);
    // Serialize message field [left_velocity]
    bufferOffset = _serializer.float64(obj.left_velocity, buffer, bufferOffset);
    // Serialize message field [right_velocity]
    bufferOffset = _serializer.float64(obj.right_velocity, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WheelEncoders
    let len;
    let data = new WheelEncoders(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [left_pulses]
    data.left_pulses = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [right_pulses]
    data.right_pulses = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [left_velocity]
    data.left_velocity = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [right_velocity]
    data.right_velocity = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'my_car_yolo/WheelEncoders';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '04b36400bf0a0e74c73173ddcaa70f42';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # WheelEncoders.msg
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WheelEncoders(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.left_pulses !== undefined) {
      resolved.left_pulses = msg.left_pulses;
    }
    else {
      resolved.left_pulses = 0
    }

    if (msg.right_pulses !== undefined) {
      resolved.right_pulses = msg.right_pulses;
    }
    else {
      resolved.right_pulses = 0
    }

    if (msg.left_velocity !== undefined) {
      resolved.left_velocity = msg.left_velocity;
    }
    else {
      resolved.left_velocity = 0.0
    }

    if (msg.right_velocity !== undefined) {
      resolved.right_velocity = msg.right_velocity;
    }
    else {
      resolved.right_velocity = 0.0
    }

    return resolved;
    }
};

module.exports = WheelEncoders;
