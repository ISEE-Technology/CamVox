// Auto-generated. Do not edit!

// (in-package inertial_sense.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let GTime = require('./GTime.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class GNSSObservation {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.time = null;
      this.sat = null;
      this.rcv = null;
      this.SNR = null;
      this.LLI = null;
      this.code = null;
      this.qualL = null;
      this.qualP = null;
      this.L = null;
      this.P = null;
      this.D = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('time')) {
        this.time = initObj.time
      }
      else {
        this.time = new GTime();
      }
      if (initObj.hasOwnProperty('sat')) {
        this.sat = initObj.sat
      }
      else {
        this.sat = 0;
      }
      if (initObj.hasOwnProperty('rcv')) {
        this.rcv = initObj.rcv
      }
      else {
        this.rcv = 0;
      }
      if (initObj.hasOwnProperty('SNR')) {
        this.SNR = initObj.SNR
      }
      else {
        this.SNR = 0;
      }
      if (initObj.hasOwnProperty('LLI')) {
        this.LLI = initObj.LLI
      }
      else {
        this.LLI = 0;
      }
      if (initObj.hasOwnProperty('code')) {
        this.code = initObj.code
      }
      else {
        this.code = 0;
      }
      if (initObj.hasOwnProperty('qualL')) {
        this.qualL = initObj.qualL
      }
      else {
        this.qualL = 0;
      }
      if (initObj.hasOwnProperty('qualP')) {
        this.qualP = initObj.qualP
      }
      else {
        this.qualP = 0;
      }
      if (initObj.hasOwnProperty('L')) {
        this.L = initObj.L
      }
      else {
        this.L = 0.0;
      }
      if (initObj.hasOwnProperty('P')) {
        this.P = initObj.P
      }
      else {
        this.P = 0.0;
      }
      if (initObj.hasOwnProperty('D')) {
        this.D = initObj.D
      }
      else {
        this.D = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GNSSObservation
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [time]
    bufferOffset = GTime.serialize(obj.time, buffer, bufferOffset);
    // Serialize message field [sat]
    bufferOffset = _serializer.uint8(obj.sat, buffer, bufferOffset);
    // Serialize message field [rcv]
    bufferOffset = _serializer.uint8(obj.rcv, buffer, bufferOffset);
    // Serialize message field [SNR]
    bufferOffset = _serializer.uint8(obj.SNR, buffer, bufferOffset);
    // Serialize message field [LLI]
    bufferOffset = _serializer.uint8(obj.LLI, buffer, bufferOffset);
    // Serialize message field [code]
    bufferOffset = _serializer.uint8(obj.code, buffer, bufferOffset);
    // Serialize message field [qualL]
    bufferOffset = _serializer.uint8(obj.qualL, buffer, bufferOffset);
    // Serialize message field [qualP]
    bufferOffset = _serializer.uint8(obj.qualP, buffer, bufferOffset);
    // Serialize message field [L]
    bufferOffset = _serializer.float64(obj.L, buffer, bufferOffset);
    // Serialize message field [P]
    bufferOffset = _serializer.float64(obj.P, buffer, bufferOffset);
    // Serialize message field [D]
    bufferOffset = _serializer.float32(obj.D, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GNSSObservation
    let len;
    let data = new GNSSObservation(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [time]
    data.time = GTime.deserialize(buffer, bufferOffset);
    // Deserialize message field [sat]
    data.sat = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rcv]
    data.rcv = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [SNR]
    data.SNR = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [LLI]
    data.LLI = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [code]
    data.code = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [qualL]
    data.qualL = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [qualP]
    data.qualP = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [L]
    data.L = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [P]
    data.P = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [D]
    data.D = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 43;
  }

  static datatype() {
    // Returns string type for a message object
    return 'inertial_sense/GNSSObservation';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f652831660ce8b4781ba3cf83655ca76';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    GTime time              # time of observation
    uint8 sat 		# satellite number
    uint8 rcv 		# receiver number
    uint8 SNR 		# Signal Strength (0.25 dBHz)
    uint8 LLI 		# Loss-of-Lock Indicator (bit1=loss-of-lock, bit2=half-cycle-invalid)
    uint8 code 		# code indicator (BeiDou: CODE_L1I, Other: CODE_L1C )
    uint8 qualL 	# Estimated carrier phase measurement standard deviation (0.004 cycles)
    uint8 qualP 	# Estimated pseudorange measurement standard deviation (0.01 m)
    float64 L      	# observation data carrier-phase (cycle)
    float64 P      	# observation data pseudorange (m)
    float32 D      	# observation data doppler frequency (0.002 Hz)
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
    MSG: inertial_sense/GTime
    int64 time
    float64 sec
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GNSSObservation(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.time !== undefined) {
      resolved.time = GTime.Resolve(msg.time)
    }
    else {
      resolved.time = new GTime()
    }

    if (msg.sat !== undefined) {
      resolved.sat = msg.sat;
    }
    else {
      resolved.sat = 0
    }

    if (msg.rcv !== undefined) {
      resolved.rcv = msg.rcv;
    }
    else {
      resolved.rcv = 0
    }

    if (msg.SNR !== undefined) {
      resolved.SNR = msg.SNR;
    }
    else {
      resolved.SNR = 0
    }

    if (msg.LLI !== undefined) {
      resolved.LLI = msg.LLI;
    }
    else {
      resolved.LLI = 0
    }

    if (msg.code !== undefined) {
      resolved.code = msg.code;
    }
    else {
      resolved.code = 0
    }

    if (msg.qualL !== undefined) {
      resolved.qualL = msg.qualL;
    }
    else {
      resolved.qualL = 0
    }

    if (msg.qualP !== undefined) {
      resolved.qualP = msg.qualP;
    }
    else {
      resolved.qualP = 0
    }

    if (msg.L !== undefined) {
      resolved.L = msg.L;
    }
    else {
      resolved.L = 0.0
    }

    if (msg.P !== undefined) {
      resolved.P = msg.P;
    }
    else {
      resolved.P = 0.0
    }

    if (msg.D !== undefined) {
      resolved.D = msg.D;
    }
    else {
      resolved.D = 0.0
    }

    return resolved;
    }
};

module.exports = GNSSObservation;
