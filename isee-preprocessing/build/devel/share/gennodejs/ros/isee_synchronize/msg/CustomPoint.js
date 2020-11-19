// Auto-generated. Do not edit!

// (in-package isee_synchronize.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class CustomPoint {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.offset_time = null;
      this.x = null;
      this.y = null;
      this.z = null;
      this.reflectivity = null;
      this.line = null;
    }
    else {
      if (initObj.hasOwnProperty('offset_time')) {
        this.offset_time = initObj.offset_time
      }
      else {
        this.offset_time = 0;
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('z')) {
        this.z = initObj.z
      }
      else {
        this.z = 0.0;
      }
      if (initObj.hasOwnProperty('reflectivity')) {
        this.reflectivity = initObj.reflectivity
      }
      else {
        this.reflectivity = 0;
      }
      if (initObj.hasOwnProperty('line')) {
        this.line = initObj.line
      }
      else {
        this.line = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CustomPoint
    // Serialize message field [offset_time]
    bufferOffset = _serializer.uint32(obj.offset_time, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.float32(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float32(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.float32(obj.z, buffer, bufferOffset);
    // Serialize message field [reflectivity]
    bufferOffset = _serializer.uint8(obj.reflectivity, buffer, bufferOffset);
    // Serialize message field [line]
    bufferOffset = _serializer.uint8(obj.line, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CustomPoint
    let len;
    let data = new CustomPoint(null);
    // Deserialize message field [offset_time]
    data.offset_time = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [reflectivity]
    data.reflectivity = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [line]
    data.line = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 18;
  }

  static datatype() {
    // Returns string type for a message object
    return 'isee_synchronize/CustomPoint';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd852cd86aa9d2b697229026ff07cf452';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new CustomPoint(null);
    if (msg.offset_time !== undefined) {
      resolved.offset_time = msg.offset_time;
    }
    else {
      resolved.offset_time = 0
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.z !== undefined) {
      resolved.z = msg.z;
    }
    else {
      resolved.z = 0.0
    }

    if (msg.reflectivity !== undefined) {
      resolved.reflectivity = msg.reflectivity;
    }
    else {
      resolved.reflectivity = 0
    }

    if (msg.line !== undefined) {
      resolved.line = msg.line;
    }
    else {
      resolved.line = 0
    }

    return resolved;
    }
};

module.exports = CustomPoint;
