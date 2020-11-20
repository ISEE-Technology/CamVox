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
let GNSSObservation = require('./GNSSObservation.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class GNSSObsVec {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.time = null;
      this.obs = null;
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
      if (initObj.hasOwnProperty('obs')) {
        this.obs = initObj.obs
      }
      else {
        this.obs = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GNSSObsVec
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [time]
    bufferOffset = GTime.serialize(obj.time, buffer, bufferOffset);
    // Serialize message field [obs]
    // Serialize the length for message field [obs]
    bufferOffset = _serializer.uint32(obj.obs.length, buffer, bufferOffset);
    obj.obs.forEach((val) => {
      bufferOffset = GNSSObservation.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GNSSObsVec
    let len;
    let data = new GNSSObsVec(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [time]
    data.time = GTime.deserialize(buffer, bufferOffset);
    // Deserialize message field [obs]
    // Deserialize array length for message field [obs]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.obs = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.obs[i] = GNSSObservation.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.obs.forEach((val) => {
      length += GNSSObservation.getMessageSize(val);
    });
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'inertial_sense/GNSSObsVec';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd228e847dabfc151b595c858b8d03b94';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    GTime time              # time of all contained observations (UTC Time w/o Leap Seconds)
    GNSSObservation[] obs   # Vector of observations
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
    ================================================================================
    MSG: inertial_sense/GNSSObservation
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GNSSObsVec(null);
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

    if (msg.obs !== undefined) {
      resolved.obs = new Array(msg.obs.length);
      for (let i = 0; i < resolved.obs.length; ++i) {
        resolved.obs[i] = GNSSObservation.Resolve(msg.obs[i]);
      }
    }
    else {
      resolved.obs = []
    }

    return resolved;
    }
};

module.exports = GNSSObsVec;
