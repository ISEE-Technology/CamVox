// Auto-generated. Do not edit!

// (in-package inertial_sense.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SatInfo = require('./SatInfo.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class GPSInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.num_sats = null;
      this.sattelite_info = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('num_sats')) {
        this.num_sats = initObj.num_sats
      }
      else {
        this.num_sats = 0;
      }
      if (initObj.hasOwnProperty('sattelite_info')) {
        this.sattelite_info = initObj.sattelite_info
      }
      else {
        this.sattelite_info = new Array(50).fill(new SatInfo());
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GPSInfo
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [num_sats]
    bufferOffset = _serializer.uint32(obj.num_sats, buffer, bufferOffset);
    // Check that the constant length array field [sattelite_info] has the right length
    if (obj.sattelite_info.length !== 50) {
      throw new Error('Unable to serialize array field sattelite_info - length must be 50')
    }
    // Serialize message field [sattelite_info]
    obj.sattelite_info.forEach((val) => {
      bufferOffset = SatInfo.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GPSInfo
    let len;
    let data = new GPSInfo(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [num_sats]
    data.num_sats = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [sattelite_info]
    len = 50;
    data.sattelite_info = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.sattelite_info[i] = SatInfo.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 404;
  }

  static datatype() {
    // Returns string type for a message object
    return 'inertial_sense/GPSInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a60e054a01708e390d6fe69c6b4a8303';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    Header header
    uint32 num_sats            		# number of sattelites in the sky
    SatInfo[50] sattelite_info	 	# information about sattelites
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
    MSG: inertial_sense/SatInfo
    uint32 sat_id # sattelite id
    uint32 cno    # Carrier to noise ratio
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GPSInfo(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.num_sats !== undefined) {
      resolved.num_sats = msg.num_sats;
    }
    else {
      resolved.num_sats = 0
    }

    if (msg.sattelite_info !== undefined) {
      resolved.sattelite_info = new Array(50)
      for (let i = 0; i < resolved.sattelite_info.length; ++i) {
        if (msg.sattelite_info.length > i) {
          resolved.sattelite_info[i] = SatInfo.Resolve(msg.sattelite_info[i]);
        }
        else {
          resolved.sattelite_info[i] = new SatInfo();
        }
      }
    }
    else {
      resolved.sattelite_info = new Array(50).fill(new SatInfo())
    }

    return resolved;
    }
};

module.exports = GPSInfo;
