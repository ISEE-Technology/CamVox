// Auto-generated. Do not edit!

// (in-package inertial_sense.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class SatInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sat_id = null;
      this.cno = null;
    }
    else {
      if (initObj.hasOwnProperty('sat_id')) {
        this.sat_id = initObj.sat_id
      }
      else {
        this.sat_id = 0;
      }
      if (initObj.hasOwnProperty('cno')) {
        this.cno = initObj.cno
      }
      else {
        this.cno = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SatInfo
    // Serialize message field [sat_id]
    bufferOffset = _serializer.uint32(obj.sat_id, buffer, bufferOffset);
    // Serialize message field [cno]
    bufferOffset = _serializer.uint32(obj.cno, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SatInfo
    let len;
    let data = new SatInfo(null);
    // Deserialize message field [sat_id]
    data.sat_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [cno]
    data.cno = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'inertial_sense/SatInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f1fb6b174b603bb921293910a7f10d63';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 sat_id # sattelite id
    uint32 cno    # Carrier to noise ratio
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SatInfo(null);
    if (msg.sat_id !== undefined) {
      resolved.sat_id = msg.sat_id;
    }
    else {
      resolved.sat_id = 0
    }

    if (msg.cno !== undefined) {
      resolved.cno = msg.cno;
    }
    else {
      resolved.cno = 0
    }

    return resolved;
    }
};

module.exports = SatInfo;
