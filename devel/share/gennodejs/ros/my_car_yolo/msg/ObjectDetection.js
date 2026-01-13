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

class ObjectDetection {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.object_id = null;
      this.center_x = null;
      this.center_y = null;
      this.confidence = null;
      this.bbox_x = null;
      this.bbox_y = null;
      this.bbox_width = null;
      this.bbox_height = null;
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
      if (initObj.hasOwnProperty('object_id')) {
        this.object_id = initObj.object_id
      }
      else {
        this.object_id = 0;
      }
      if (initObj.hasOwnProperty('center_x')) {
        this.center_x = initObj.center_x
      }
      else {
        this.center_x = 0.0;
      }
      if (initObj.hasOwnProperty('center_y')) {
        this.center_y = initObj.center_y
      }
      else {
        this.center_y = 0.0;
      }
      if (initObj.hasOwnProperty('confidence')) {
        this.confidence = initObj.confidence
      }
      else {
        this.confidence = 0.0;
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
    // Serializes a message object of type ObjectDetection
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [object_id]
    bufferOffset = _serializer.uint32(obj.object_id, buffer, bufferOffset);
    // Serialize message field [center_x]
    bufferOffset = _serializer.float32(obj.center_x, buffer, bufferOffset);
    // Serialize message field [center_y]
    bufferOffset = _serializer.float32(obj.center_y, buffer, bufferOffset);
    // Serialize message field [confidence]
    bufferOffset = _serializer.float32(obj.confidence, buffer, bufferOffset);
    // Serialize message field [bbox_x]
    bufferOffset = _serializer.float32(obj.bbox_x, buffer, bufferOffset);
    // Serialize message field [bbox_y]
    bufferOffset = _serializer.float32(obj.bbox_y, buffer, bufferOffset);
    // Serialize message field [bbox_width]
    bufferOffset = _serializer.float32(obj.bbox_width, buffer, bufferOffset);
    // Serialize message field [bbox_height]
    bufferOffset = _serializer.float32(obj.bbox_height, buffer, bufferOffset);
    // Serialize message field [image_width]
    bufferOffset = _serializer.uint32(obj.image_width, buffer, bufferOffset);
    // Serialize message field [image_height]
    bufferOffset = _serializer.uint32(obj.image_height, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ObjectDetection
    let len;
    let data = new ObjectDetection(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [object_id]
    data.object_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [center_x]
    data.center_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [center_y]
    data.center_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [confidence]
    data.confidence = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bbox_x]
    data.bbox_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bbox_y]
    data.bbox_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bbox_width]
    data.bbox_width = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bbox_height]
    data.bbox_height = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [image_width]
    data.image_width = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [image_height]
    data.image_height = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'my_car_yolo/ObjectDetection';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '74b64be95b23c1ba8f1d1679b6ef3c59';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ObjectDetection.msg
    # 单个物体的检测结果
    
    std_msgs/Header header
    uint32 object_id          # 物体ID (1, 2, 3...)
    float32 center_x          # 中心点x坐标（像素）
    float32 center_y          # 中心点y坐标（像素）
    float32 confidence        # 检测置信度
    float32 bbox_x            # 边界框左上角x
    float32 bbox_y            # 边界框左上角y
    float32 bbox_width        # 边界框宽度
    float32 bbox_height        # 边界框高度
    uint32 image_width        # 图像宽度
    uint32 image_height       # 图像高度
    
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
    const resolved = new ObjectDetection(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.object_id !== undefined) {
      resolved.object_id = msg.object_id;
    }
    else {
      resolved.object_id = 0
    }

    if (msg.center_x !== undefined) {
      resolved.center_x = msg.center_x;
    }
    else {
      resolved.center_x = 0.0
    }

    if (msg.center_y !== undefined) {
      resolved.center_y = msg.center_y;
    }
    else {
      resolved.center_y = 0.0
    }

    if (msg.confidence !== undefined) {
      resolved.confidence = msg.confidence;
    }
    else {
      resolved.confidence = 0.0
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

module.exports = ObjectDetection;
