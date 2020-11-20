// Auto-generated. Do not edit!

// (in-package inertial_sense.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class INL2States {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.quatEcef = null;
      this.velEcef = null;
      this.posEcef = null;
      this.gyroBias = null;
      this.accelBias = null;
      this.baroBias = null;
      this.magDec = null;
      this.magInc = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('quatEcef')) {
        this.quatEcef = initObj.quatEcef
      }
      else {
        this.quatEcef = new geometry_msgs.msg.Quaternion();
      }
      if (initObj.hasOwnProperty('velEcef')) {
        this.velEcef = initObj.velEcef
      }
      else {
        this.velEcef = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('posEcef')) {
        this.posEcef = initObj.posEcef
      }
      else {
        this.posEcef = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('gyroBias')) {
        this.gyroBias = initObj.gyroBias
      }
      else {
        this.gyroBias = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('accelBias')) {
        this.accelBias = initObj.accelBias
      }
      else {
        this.accelBias = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('baroBias')) {
        this.baroBias = initObj.baroBias
      }
      else {
        this.baroBias = 0.0;
      }
      if (initObj.hasOwnProperty('magDec')) {
        this.magDec = initObj.magDec
      }
      else {
        this.magDec = 0.0;
      }
      if (initObj.hasOwnProperty('magInc')) {
        this.magInc = initObj.magInc
      }
      else {
        this.magInc = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type INL2States
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [quatEcef]
    bufferOffset = geometry_msgs.msg.Quaternion.serialize(obj.quatEcef, buffer, bufferOffset);
    // Serialize message field [velEcef]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.velEcef, buffer, bufferOffset);
    // Serialize message field [posEcef]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.posEcef, buffer, bufferOffset);
    // Serialize message field [gyroBias]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.gyroBias, buffer, bufferOffset);
    // Serialize message field [accelBias]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.accelBias, buffer, bufferOffset);
    // Serialize message field [baroBias]
    bufferOffset = _serializer.float32(obj.baroBias, buffer, bufferOffset);
    // Serialize message field [magDec]
    bufferOffset = _serializer.float32(obj.magDec, buffer, bufferOffset);
    // Serialize message field [magInc]
    bufferOffset = _serializer.float32(obj.magInc, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type INL2States
    let len;
    let data = new INL2States(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [quatEcef]
    data.quatEcef = geometry_msgs.msg.Quaternion.deserialize(buffer, bufferOffset);
    // Deserialize message field [velEcef]
    data.velEcef = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [posEcef]
    data.posEcef = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [gyroBias]
    data.gyroBias = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [accelBias]
    data.accelBias = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [baroBias]
    data.baroBias = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [magDec]
    data.magDec = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [magInc]
    data.magInc = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 140;
  }

  static datatype() {
    // Returns string type for a message object
    return 'inertial_sense/INL2States';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '06de6b8d1957718b78007390d5c6fc67';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header                       # GPS time of week (since Sunday morning) in seconds
    geometry_msgs/Quaternion quatEcef   # Quaternion body rotation with respect to ECEF
    geometry_msgs/Vector3 velEcef       # (m/s) Velocity in ECEF frame
    geometry_msgs/Vector3 posEcef       # (m) Position in ECEF frame
    geometry_msgs/Vector3 gyroBias      # (rad/s) Gyro bias
    geometry_msgs/Vector3 accelBias     # (m/s^2) Accelerometer bias
    float32 baroBias                    # (m) Barometer bias
    float32 magDec                      # (rad) Magnetic declination
    float32 magInc                      # (rad) Magnetic inclination
    
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
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new INL2States(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.quatEcef !== undefined) {
      resolved.quatEcef = geometry_msgs.msg.Quaternion.Resolve(msg.quatEcef)
    }
    else {
      resolved.quatEcef = new geometry_msgs.msg.Quaternion()
    }

    if (msg.velEcef !== undefined) {
      resolved.velEcef = geometry_msgs.msg.Vector3.Resolve(msg.velEcef)
    }
    else {
      resolved.velEcef = new geometry_msgs.msg.Vector3()
    }

    if (msg.posEcef !== undefined) {
      resolved.posEcef = geometry_msgs.msg.Vector3.Resolve(msg.posEcef)
    }
    else {
      resolved.posEcef = new geometry_msgs.msg.Vector3()
    }

    if (msg.gyroBias !== undefined) {
      resolved.gyroBias = geometry_msgs.msg.Vector3.Resolve(msg.gyroBias)
    }
    else {
      resolved.gyroBias = new geometry_msgs.msg.Vector3()
    }

    if (msg.accelBias !== undefined) {
      resolved.accelBias = geometry_msgs.msg.Vector3.Resolve(msg.accelBias)
    }
    else {
      resolved.accelBias = new geometry_msgs.msg.Vector3()
    }

    if (msg.baroBias !== undefined) {
      resolved.baroBias = msg.baroBias;
    }
    else {
      resolved.baroBias = 0.0
    }

    if (msg.magDec !== undefined) {
      resolved.magDec = msg.magDec;
    }
    else {
      resolved.magDec = 0.0
    }

    if (msg.magInc !== undefined) {
      resolved.magInc = msg.magInc;
    }
    else {
      resolved.magInc = 0.0
    }

    return resolved;
    }
};

module.exports = INL2States;
