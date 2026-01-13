// Auto-generated. Do not edit!

// (in-package arm_follow.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let PersonKeypoints = require('./PersonKeypoints.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class PersonsKeypoints {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.persons = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('persons')) {
        this.persons = initObj.persons
      }
      else {
        this.persons = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PersonsKeypoints
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [persons]
    // Serialize the length for message field [persons]
    bufferOffset = _serializer.uint32(obj.persons.length, buffer, bufferOffset);
    obj.persons.forEach((val) => {
      bufferOffset = PersonKeypoints.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PersonsKeypoints
    let len;
    let data = new PersonsKeypoints(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [persons]
    // Deserialize array length for message field [persons]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.persons = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.persons[i] = PersonKeypoints.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.persons.forEach((val) => {
      length += PersonKeypoints.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'arm_follow/PersonsKeypoints';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2cc6b7f8f80dc708d5b607449f5f648f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # PersonsKeypoints.msg
    # 多个人体的关键点数据列表
    
    std_msgs/Header header
    PersonKeypoints[] persons  # 所有人体的关键点列表
    
    
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
    MSG: arm_follow/PersonKeypoints
    # PersonKeypoints.msg
    # 一个人体的关键点数据
    
    std_msgs/Header header
    uint32 person_id          # 人员ID
    float32 bbox_x            # 检测框左上角x
    float32 bbox_y            # 检测框左上角y
    float32 bbox_width        # 检测框宽度
    float32 bbox_height       # 检测框高度
    float32 bbox_area         # 检测框面积
    float32 bbox_confidence   # 检测框置信度（YOLO显示的置信度）
    
    # 右手关键点（COCO格式中通常是右手腕，索引10）
    float32 right_wrist_x     # 右手腕x坐标
    float32 right_wrist_y     # 右手腕y坐标
    float32 right_wrist_conf  # 右手腕置信度
    
    # 右手肘（索引8）
    float32 right_elbow_x
    float32 right_elbow_y
    float32 right_elbow_conf
    
    # 右肩膀（用于判断右手是否伸出）
    float32 right_shoulder_x
    float32 right_shoulder_y
    float32 right_shoulder_conf
    
    # 图像尺寸（用于计算角度）
    uint32 image_width
    uint32 image_height
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PersonsKeypoints(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.persons !== undefined) {
      resolved.persons = new Array(msg.persons.length);
      for (let i = 0; i < resolved.persons.length; ++i) {
        resolved.persons[i] = PersonKeypoints.Resolve(msg.persons[i]);
      }
    }
    else {
      resolved.persons = []
    }

    return resolved;
    }
};

module.exports = PersonsKeypoints;
