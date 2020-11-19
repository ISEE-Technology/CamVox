// Auto-generated. Do not edit!

// (in-package isee_synchronize.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let CustomPoint = require('./CustomPoint.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class CustomMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.timebase = null;
      this.point_num = null;
      this.lidar_id = null;
      this.rsvd = null;
      this.points = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('timebase')) {
        this.timebase = initObj.timebase
      }
      else {
        this.timebase = 0;
      }
      if (initObj.hasOwnProperty('point_num')) {
        this.point_num = initObj.point_num
      }
      else {
        this.point_num = 0;
      }
      if (initObj.hasOwnProperty('lidar_id')) {
        this.lidar_id = initObj.lidar_id
      }
      else {
        this.lidar_id = 0;
      }
      if (initObj.hasOwnProperty('rsvd')) {
        this.rsvd = initObj.rsvd
      }
      else {
        this.rsvd = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('points')) {
        this.points = initObj.points
      }
      else {
        this.points = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CustomMsg
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [timebase]
    bufferOffset = _serializer.uint64(obj.timebase, buffer, bufferOffset);
    // Serialize message field [point_num]
    bufferOffset = _serializer.uint32(obj.point_num, buffer, bufferOffset);
    // Serialize message field [lidar_id]
    bufferOffset = _serializer.uint8(obj.lidar_id, buffer, bufferOffset);
    // Check that the constant length array field [rsvd] has the right length
    if (obj.rsvd.length !== 3) {
      throw new Error('Unable to serialize array field rsvd - length must be 3')
    }
    // Serialize message field [rsvd]
    bufferOffset = _arraySerializer.uint8(obj.rsvd, buffer, bufferOffset, 3);
    // Serialize message field [points]
    // Serialize the length for message field [points]
    bufferOffset = _serializer.uint32(obj.points.length, buffer, bufferOffset);
    obj.points.forEach((val) => {
      bufferOffset = CustomPoint.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CustomMsg
    let len;
    let data = new CustomMsg(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [timebase]
    data.timebase = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [point_num]
    data.point_num = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [lidar_id]
    data.lidar_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rsvd]
    data.rsvd = _arrayDeserializer.uint8(buffer, bufferOffset, 3)
    // Deserialize message field [points]
    // Deserialize array length for message field [points]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.points = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.points[i] = CustomPoint.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 18 * object.points.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'isee_synchronize/CustomMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'aa196d9cdf97226174bd64be4d696f75';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Livox publish pointcloud msg format.
    
    Header header             # ROS standard message header
    uint64 timebase           # The time of first point
    uint32 point_num          # Total number of pointclouds
    uint8  lidar_id           # Lidar device id number
    uint8[3]  rsvd            # Reserved use
    CustomPoint[] points      # Pointcloud data
    
    
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
    # 0: no frame
    # 1: global frame
    string frame_id
    
    ================================================================================
    MSG: isee_synchronize/CustomPoint
    # Livox costom pointcloud format.
    
    uint32 offset_time      # offset time relative to the base time
    float32 x               # X axis, unit:m
    float32 y               # Y axis, unit:m
    float32 z               # Z axis, unit:m
    uint8 reflectivity      # reflectivity, 0~255
    uint8 line              # laser number in lidar
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CustomMsg(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.timebase !== undefined) {
      resolved.timebase = msg.timebase;
    }
    else {
      resolved.timebase = 0
    }

    if (msg.point_num !== undefined) {
      resolved.point_num = msg.point_num;
    }
    else {
      resolved.point_num = 0
    }

    if (msg.lidar_id !== undefined) {
      resolved.lidar_id = msg.lidar_id;
    }
    else {
      resolved.lidar_id = 0
    }

    if (msg.rsvd !== undefined) {
      resolved.rsvd = msg.rsvd;
    }
    else {
      resolved.rsvd = new Array(3).fill(0)
    }

    if (msg.points !== undefined) {
      resolved.points = new Array(msg.points.length);
      for (let i = 0; i < resolved.points.length; ++i) {
        resolved.points[i] = CustomPoint.Resolve(msg.points[i]);
      }
    }
    else {
      resolved.points = []
    }

    return resolved;
    }
};

module.exports = CustomMsg;
