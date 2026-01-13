// Auto-generated. Do not edit!

// (in-package arm_follow.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class PersonKeypoints {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.person_id = null;
      this.bbox_x = null;
      this.bbox_y = null;
      this.bbox_width = null;
      this.bbox_height = null;
      this.bbox_area = null;
      this.bbox_confidence = null;
      this.right_wrist_x = null;
      this.right_wrist_y = null;
      this.right_wrist_conf = null;
      this.right_elbow_x = null;
      this.right_elbow_y = null;
      this.right_elbow_conf = null;
      this.right_shoulder_x = null;
      this.right_shoulder_y = null;
      this.right_shoulder_conf = null;
      this.image_width = null;
      this.image_height = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('person_id')) {
        this.person_id = initObj.person_id
      }
      else {
        this.person_id = 0;
      }
      if (initObj.hasOwnProperty('bbox_x')) {
        this.bbox_x = initObj.bbox_x
      }
      else {
        this.bbox_x = 0.0;
      }
      if (initObj.hasOwnProperty('bbox_y')) {
        this.bbox_y = initObj.bbox_y
      }
      else {
        this.bbox_y = 0.0;
      }
      if (initObj.hasOwnProperty('bbox_width')) {
        this.bbox_width = initObj.bbox_width
      }
      else {
        this.bbox_width = 0.0;
      }
      if (initObj.hasOwnProperty('bbox_height')) {
        this.bbox_height = initObj.bbox_height
      }
      else {
        this.bbox_height = 0.0;
      }
      if (initObj.hasOwnProperty('bbox_area')) {
        this.bbox_area = initObj.bbox_area
      }
      else {
        this.bbox_area = 0.0;
      }
      if (initObj.hasOwnProperty('bbox_confidence')) {
        this.bbox_confidence = initObj.bbox_confidence
      }
      else {
        this.bbox_confidence = 0.0;
      }
      if (initObj.hasOwnProperty('right_wrist_x')) {
        this.right_wrist_x = initObj.right_wrist_x
      }
      else {
        this.right_wrist_x = 0.0;
      }
      if (initObj.hasOwnProperty('right_wrist_y')) {
        this.right_wrist_y = initObj.right_wrist_y
      }
      else {
        this.right_wrist_y = 0.0;
      }
      if (initObj.hasOwnProperty('right_wrist_conf')) {
        this.right_wrist_conf = initObj.right_wrist_conf
      }
      else {
        this.right_wrist_conf = 0.0;
      }
      if (initObj.hasOwnProperty('right_elbow_x')) {
        this.right_elbow_x = initObj.right_elbow_x
      }
      else {
        this.right_elbow_x = 0.0;
      }
      if (initObj.hasOwnProperty('right_elbow_y')) {
        this.right_elbow_y = initObj.right_elbow_y
      }
      else {
        this.right_elbow_y = 0.0;
      }
      if (initObj.hasOwnProperty('right_elbow_conf')) {
        this.right_elbow_conf = initObj.right_elbow_conf
      }
      else {
        this.right_elbow_conf = 0.0;
      }
      if (initObj.hasOwnProperty('right_shoulder_x')) {
        this.right_shoulder_x = initObj.right_shoulder_x
      }
      else {
        this.right_shoulder_x = 0.0;
      }
      if (initObj.hasOwnProperty('right_shoulder_y')) {
        this.right_shoulder_y = initObj.right_shoulder_y
      }
      else {
        this.right_shoulder_y = 0.0;
      }
      if (initObj.hasOwnProperty('right_shoulder_conf')) {
        this.right_shoulder_conf = initObj.right_shoulder_conf
      }
      else {
        this.right_shoulder_conf = 0.0;
      }
      if (initObj.hasOwnProperty('image_width')) {
        this.image_width = initObj.image_width
      }
      else {
        this.image_width = 0;
      }
      if (initObj.hasOwnProperty('image_height')) {
        this.image_height = initObj.image_height
      }
      else {
        this.image_height = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PersonKeypoints
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [person_id]
    bufferOffset = _serializer.uint32(obj.person_id, buffer, bufferOffset);
    // Serialize message field [bbox_x]
    bufferOffset = _serializer.float32(obj.bbox_x, buffer, bufferOffset);
    // Serialize message field [bbox_y]
    bufferOffset = _serializer.float32(obj.bbox_y, buffer, bufferOffset);
    // Serialize message field [bbox_width]
    bufferOffset = _serializer.float32(obj.bbox_width, buffer, bufferOffset);
    // Serialize message field [bbox_height]
    bufferOffset = _serializer.float32(obj.bbox_height, buffer, bufferOffset);
    // Serialize message field [bbox_area]
    bufferOffset = _serializer.float32(obj.bbox_area, buffer, bufferOffset);
    // Serialize message field [bbox_confidence]
    bufferOffset = _serializer.float32(obj.bbox_confidence, buffer, bufferOffset);
    // Serialize message field [right_wrist_x]
    bufferOffset = _serializer.float32(obj.right_wrist_x, buffer, bufferOffset);
    // Serialize message field [right_wrist_y]
    bufferOffset = _serializer.float32(obj.right_wrist_y, buffer, bufferOffset);
    // Serialize message field [right_wrist_conf]
    bufferOffset = _serializer.float32(obj.right_wrist_conf, buffer, bufferOffset);
    // Serialize message field [right_elbow_x]
    bufferOffset = _serializer.float32(obj.right_elbow_x, buffer, bufferOffset);
    // Serialize message field [right_elbow_y]
    bufferOffset = _serializer.float32(obj.right_elbow_y, buffer, bufferOffset);
    // Serialize message field [right_elbow_conf]
    bufferOffset = _serializer.float32(obj.right_elbow_conf, buffer, bufferOffset);
    // Serialize message field [right_shoulder_x]
    bufferOffset = _serializer.float32(obj.right_shoulder_x, buffer, bufferOffset);
    // Serialize message field [right_shoulder_y]
    bufferOffset = _serializer.float32(obj.right_shoulder_y, buffer, bufferOffset);
    // Serialize message field [right_shoulder_conf]
    bufferOffset = _serializer.float32(obj.right_shoulder_conf, buffer, bufferOffset);
    // Serialize message field [image_width]
    bufferOffset = _serializer.uint32(obj.image_width, buffer, bufferOffset);
    // Serialize message field [image_height]
    bufferOffset = _serializer.uint32(obj.image_height, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PersonKeypoints
    let len;
    let data = new PersonKeypoints(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [person_id]
    data.person_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [bbox_x]
    data.bbox_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bbox_y]
    data.bbox_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bbox_width]
    data.bbox_width = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bbox_height]
    data.bbox_height = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bbox_area]
    data.bbox_area = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bbox_confidence]
    data.bbox_confidence = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_wrist_x]
    data.right_wrist_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_wrist_y]
    data.right_wrist_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_wrist_conf]
    data.right_wrist_conf = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_elbow_x]
    data.right_elbow_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_elbow_y]
    data.right_elbow_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_elbow_conf]
    data.right_elbow_conf = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_shoulder_x]
    data.right_shoulder_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_shoulder_y]
    data.right_shoulder_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_shoulder_conf]
    data.right_shoulder_conf = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [image_width]
    data.image_width = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [image_height]
    data.image_height = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 72;
  }

  static datatype() {
    // Returns string type for a message object
    return 'arm_follow/PersonKeypoints';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b8aef0b6f60577e67360824121ee7a78';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new PersonKeypoints(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.person_id !== undefined) {
      resolved.person_id = msg.person_id;
    }
    else {
      resolved.person_id = 0
    }

    if (msg.bbox_x !== undefined) {
      resolved.bbox_x = msg.bbox_x;
    }
    else {
      resolved.bbox_x = 0.0
    }

    if (msg.bbox_y !== undefined) {
      resolved.bbox_y = msg.bbox_y;
    }
    else {
      resolved.bbox_y = 0.0
    }

    if (msg.bbox_width !== undefined) {
      resolved.bbox_width = msg.bbox_width;
    }
    else {
      resolved.bbox_width = 0.0
    }

    if (msg.bbox_height !== undefined) {
      resolved.bbox_height = msg.bbox_height;
    }
    else {
      resolved.bbox_height = 0.0
    }

    if (msg.bbox_area !== undefined) {
      resolved.bbox_area = msg.bbox_area;
    }
    else {
      resolved.bbox_area = 0.0
    }

    if (msg.bbox_confidence !== undefined) {
      resolved.bbox_confidence = msg.bbox_confidence;
    }
    else {
      resolved.bbox_confidence = 0.0
    }

    if (msg.right_wrist_x !== undefined) {
      resolved.right_wrist_x = msg.right_wrist_x;
    }
    else {
      resolved.right_wrist_x = 0.0
    }

    if (msg.right_wrist_y !== undefined) {
      resolved.right_wrist_y = msg.right_wrist_y;
    }
    else {
      resolved.right_wrist_y = 0.0
    }

    if (msg.right_wrist_conf !== undefined) {
      resolved.right_wrist_conf = msg.right_wrist_conf;
    }
    else {
      resolved.right_wrist_conf = 0.0
    }

    if (msg.right_elbow_x !== undefined) {
      resolved.right_elbow_x = msg.right_elbow_x;
    }
    else {
      resolved.right_elbow_x = 0.0
    }

    if (msg.right_elbow_y !== undefined) {
      resolved.right_elbow_y = msg.right_elbow_y;
    }
    else {
      resolved.right_elbow_y = 0.0
    }

    if (msg.right_elbow_conf !== undefined) {
      resolved.right_elbow_conf = msg.right_elbow_conf;
    }
    else {
      resolved.right_elbow_conf = 0.0
    }

    if (msg.right_shoulder_x !== undefined) {
      resolved.right_shoulder_x = msg.right_shoulder_x;
    }
    else {
      resolved.right_shoulder_x = 0.0
    }

    if (msg.right_shoulder_y !== undefined) {
      resolved.right_shoulder_y = msg.right_shoulder_y;
    }
    else {
      resolved.right_shoulder_y = 0.0
    }

    if (msg.right_shoulder_conf !== undefined) {
      resolved.right_shoulder_conf = msg.right_shoulder_conf;
    }
    else {
      resolved.right_shoulder_conf = 0.0
    }

    if (msg.image_width !== undefined) {
      resolved.image_width = msg.image_width;
    }
    else {
      resolved.image_width = 0
    }

    if (msg.image_height !== undefined) {
      resolved.image_height = msg.image_height;
    }
    else {
      resolved.image_height = 0
    }

    return resolved;
    }
};

module.exports = PersonKeypoints;
