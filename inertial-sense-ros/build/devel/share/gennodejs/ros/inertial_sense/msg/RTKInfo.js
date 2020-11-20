// Auto-generated. Do not edit!

// (in-package inertial_sense.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class RTKInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.BaseLLA = null;
      this.cycle_slip_count = null;
      this.roverObs = null;
      this.baseObs = null;
      this.roverEph = null;
      this.baseEph = null;
      this.baseAntcount = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('BaseLLA')) {
        this.BaseLLA = initObj.BaseLLA
      }
      else {
        this.BaseLLA = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('cycle_slip_count')) {
        this.cycle_slip_count = initObj.cycle_slip_count
      }
      else {
        this.cycle_slip_count = 0;
      }
      if (initObj.hasOwnProperty('roverObs')) {
        this.roverObs = initObj.roverObs
      }
      else {
        this.roverObs = 0;
      }
      if (initObj.hasOwnProperty('baseObs')) {
        this.baseObs = initObj.baseObs
      }
      else {
        this.baseObs = 0;
      }
      if (initObj.hasOwnProperty('roverEph')) {
        this.roverEph = initObj.roverEph
      }
      else {
        this.roverEph = 0;
      }
      if (initObj.hasOwnProperty('baseEph')) {
        this.baseEph = initObj.baseEph
      }
      else {
        this.baseEph = 0;
      }
      if (initObj.hasOwnProperty('baseAntcount')) {
        this.baseAntcount = initObj.baseAntcount
      }
      else {
        this.baseAntcount = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RTKInfo
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [BaseLLA] has the right length
    if (obj.BaseLLA.length !== 3) {
      throw new Error('Unable to serialize array field BaseLLA - length must be 3')
    }
    // Serialize message field [BaseLLA]
    bufferOffset = _arraySerializer.float32(obj.BaseLLA, buffer, bufferOffset, 3);
    // Serialize message field [cycle_slip_count]
    bufferOffset = _serializer.uint32(obj.cycle_slip_count, buffer, bufferOffset);
    // Serialize message field [roverObs]
    bufferOffset = _serializer.uint32(obj.roverObs, buffer, bufferOffset);
    // Serialize message field [baseObs]
    bufferOffset = _serializer.uint32(obj.baseObs, buffer, bufferOffset);
    // Serialize message field [roverEph]
    bufferOffset = _serializer.uint32(obj.roverEph, buffer, bufferOffset);
    // Serialize message field [baseEph]
    bufferOffset = _serializer.uint32(obj.baseEph, buffer, bufferOffset);
    // Serialize message field [baseAntcount]
    bufferOffset = _serializer.uint32(obj.baseAntcount, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RTKInfo
    let len;
    let data = new RTKInfo(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [BaseLLA]
    data.BaseLLA = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [cycle_slip_count]
    data.cycle_slip_count = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [roverObs]
    data.roverObs = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [baseObs]
    data.baseObs = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [roverEph]
    data.roverEph = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [baseEph]
    data.baseEph = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [baseAntcount]
    data.baseAntcount = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 36;
  }

  static datatype() {
    // Returns string type for a message object
    return 'inertial_sense/RTKInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0f06cd1205181677917f42a40817ccb4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    float32[3] BaseLLA 			# base position in lat-lon-altitude (deg, deg, m)
    uint32 cycle_slip_count 	# number of cycle slips detected
    uint32 roverObs				# number of observations from rover (GPS, Glonass, Gallileo, Beidou, Qzs)
    uint32 baseObs				# number of observations from base (GPS, Glonass, Gallileo, Beidou, Qzs)
    uint32 roverEph				# number of ephemeris messages from rover (GPS, Glonass, Gallileo, Beidou, Qzs)
    uint32 baseEph				# number of ephemeris messages from rover (GPS, Glonass, Gallileo, Beidou, Qzs)
    uint32 baseAntcount			# number of base station antenna position measurements
    
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RTKInfo(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.BaseLLA !== undefined) {
      resolved.BaseLLA = msg.BaseLLA;
    }
    else {
      resolved.BaseLLA = new Array(3).fill(0)
    }

    if (msg.cycle_slip_count !== undefined) {
      resolved.cycle_slip_count = msg.cycle_slip_count;
    }
    else {
      resolved.cycle_slip_count = 0
    }

    if (msg.roverObs !== undefined) {
      resolved.roverObs = msg.roverObs;
    }
    else {
      resolved.roverObs = 0
    }

    if (msg.baseObs !== undefined) {
      resolved.baseObs = msg.baseObs;
    }
    else {
      resolved.baseObs = 0
    }

    if (msg.roverEph !== undefined) {
      resolved.roverEph = msg.roverEph;
    }
    else {
      resolved.roverEph = 0
    }

    if (msg.baseEph !== undefined) {
      resolved.baseEph = msg.baseEph;
    }
    else {
      resolved.baseEph = 0
    }

    if (msg.baseAntcount !== undefined) {
      resolved.baseAntcount = msg.baseAntcount;
    }
    else {
      resolved.baseAntcount = 0
    }

    return resolved;
    }
};

module.exports = RTKInfo;
