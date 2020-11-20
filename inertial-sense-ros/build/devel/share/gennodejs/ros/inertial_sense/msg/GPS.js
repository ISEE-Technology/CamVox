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

class GPS {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.num_sat = null;
      this.fix_type = null;
      this.cno = null;
      this.latitude = null;
      this.longitude = null;
      this.altitude = null;
      this.posEcef = null;
      this.velEcef = null;
      this.hMSL = null;
      this.hAcc = null;
      this.vAcc = null;
      this.sAcc = null;
      this.pDop = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('num_sat')) {
        this.num_sat = initObj.num_sat
      }
      else {
        this.num_sat = 0;
      }
      if (initObj.hasOwnProperty('fix_type')) {
        this.fix_type = initObj.fix_type
      }
      else {
        this.fix_type = 0;
      }
      if (initObj.hasOwnProperty('cno')) {
        this.cno = initObj.cno
      }
      else {
        this.cno = 0;
      }
      if (initObj.hasOwnProperty('latitude')) {
        this.latitude = initObj.latitude
      }
      else {
        this.latitude = 0.0;
      }
      if (initObj.hasOwnProperty('longitude')) {
        this.longitude = initObj.longitude
      }
      else {
        this.longitude = 0.0;
      }
      if (initObj.hasOwnProperty('altitude')) {
        this.altitude = initObj.altitude
      }
      else {
        this.altitude = 0.0;
      }
      if (initObj.hasOwnProperty('posEcef')) {
        this.posEcef = initObj.posEcef
      }
      else {
        this.posEcef = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('velEcef')) {
        this.velEcef = initObj.velEcef
      }
      else {
        this.velEcef = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('hMSL')) {
        this.hMSL = initObj.hMSL
      }
      else {
        this.hMSL = 0.0;
      }
      if (initObj.hasOwnProperty('hAcc')) {
        this.hAcc = initObj.hAcc
      }
      else {
        this.hAcc = 0.0;
      }
      if (initObj.hasOwnProperty('vAcc')) {
        this.vAcc = initObj.vAcc
      }
      else {
        this.vAcc = 0.0;
      }
      if (initObj.hasOwnProperty('sAcc')) {
        this.sAcc = initObj.sAcc
      }
      else {
        this.sAcc = 0.0;
      }
      if (initObj.hasOwnProperty('pDop')) {
        this.pDop = initObj.pDop
      }
      else {
        this.pDop = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GPS
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [num_sat]
    bufferOffset = _serializer.int8(obj.num_sat, buffer, bufferOffset);
    // Serialize message field [fix_type]
    bufferOffset = _serializer.uint32(obj.fix_type, buffer, bufferOffset);
    // Serialize message field [cno]
    bufferOffset = _serializer.int32(obj.cno, buffer, bufferOffset);
    // Serialize message field [latitude]
    bufferOffset = _serializer.float64(obj.latitude, buffer, bufferOffset);
    // Serialize message field [longitude]
    bufferOffset = _serializer.float64(obj.longitude, buffer, bufferOffset);
    // Serialize message field [altitude]
    bufferOffset = _serializer.float64(obj.altitude, buffer, bufferOffset);
    // Serialize message field [posEcef]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.posEcef, buffer, bufferOffset);
    // Serialize message field [velEcef]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.velEcef, buffer, bufferOffset);
    // Serialize message field [hMSL]
    bufferOffset = _serializer.float32(obj.hMSL, buffer, bufferOffset);
    // Serialize message field [hAcc]
    bufferOffset = _serializer.float32(obj.hAcc, buffer, bufferOffset);
    // Serialize message field [vAcc]
    bufferOffset = _serializer.float32(obj.vAcc, buffer, bufferOffset);
    // Serialize message field [sAcc]
    bufferOffset = _serializer.float32(obj.sAcc, buffer, bufferOffset);
    // Serialize message field [pDop]
    bufferOffset = _serializer.float32(obj.pDop, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GPS
    let len;
    let data = new GPS(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [num_sat]
    data.num_sat = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [fix_type]
    data.fix_type = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [cno]
    data.cno = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [latitude]
    data.latitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [longitude]
    data.longitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [altitude]
    data.altitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [posEcef]
    data.posEcef = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [velEcef]
    data.velEcef = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [hMSL]
    data.hMSL = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [hAcc]
    data.hAcc = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vAcc]
    data.vAcc = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [sAcc]
    data.sAcc = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pDop]
    data.pDop = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 101;
  }

  static datatype() {
    // Returns string type for a message object
    return 'inertial_sense/GPS';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6aa847d654b817ff4bb5ba8c773b2a17';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # GPS status flags
    uint32 GPS_STATUS_FIX_TYPE_NO_FIX               = 0
    uint32 GPS_STATUS_FIX_TYPE_DEAD_RECKONING_ONLY  = 256
    uint32 GPS_STATUS_FIX_TYPE_2D_FIX               = 512
    uint32 GPS_STATUS_FIX_TYPE_3D_FIX               = 768
    uint32 GPS_STATUS_FIX_TYPE_GPS_PLUS_DEAD_RECK   = 1024
    uint32 GPS_STATUS_FIX_TYPE_TIME_ONLY_FIX        = 1280
    uint32 GPS_STATUS_FIX_TYPE_RESERVED1            = 1536
    uint32 GPS_STATUS_FIX_TYPE_RESERVED2            = 1792
    
    uint32 GPS_STATUS_FIX_STATUS_FIX_OK             = 65536
    
    Header header
    int8 num_sat 							# Number of satellites used in solution
    uint32 fix_type 						# Fix type, one of STATUS_FIX_TYPE flags
    int32 cno 								# mean carrier noise ratio (dBHz)
    float64 latitude 						# latitude (degrees) 
    float64 longitude						# longitude (degrees)
    float64 altitude						# height above ellipsoid (not MSL) (m)
    geometry_msgs/Vector3 posEcef           # Position (m) in ECEF
    geometry_msgs/Vector3 velEcef       	# Velocity (m/s) in ECEF
    float32 hMSL							# height above MSL
    float32 hAcc							# horizontal accuracy
    float32 vAcc							# vertical accuracy
    float32 sAcc							# speed accuracy (m/s)
    float32 pDop							# Position Dilution of Precision (m)	
    
    
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
    const resolved = new GPS(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.num_sat !== undefined) {
      resolved.num_sat = msg.num_sat;
    }
    else {
      resolved.num_sat = 0
    }

    if (msg.fix_type !== undefined) {
      resolved.fix_type = msg.fix_type;
    }
    else {
      resolved.fix_type = 0
    }

    if (msg.cno !== undefined) {
      resolved.cno = msg.cno;
    }
    else {
      resolved.cno = 0
    }

    if (msg.latitude !== undefined) {
      resolved.latitude = msg.latitude;
    }
    else {
      resolved.latitude = 0.0
    }

    if (msg.longitude !== undefined) {
      resolved.longitude = msg.longitude;
    }
    else {
      resolved.longitude = 0.0
    }

    if (msg.altitude !== undefined) {
      resolved.altitude = msg.altitude;
    }
    else {
      resolved.altitude = 0.0
    }

    if (msg.posEcef !== undefined) {
      resolved.posEcef = geometry_msgs.msg.Vector3.Resolve(msg.posEcef)
    }
    else {
      resolved.posEcef = new geometry_msgs.msg.Vector3()
    }

    if (msg.velEcef !== undefined) {
      resolved.velEcef = geometry_msgs.msg.Vector3.Resolve(msg.velEcef)
    }
    else {
      resolved.velEcef = new geometry_msgs.msg.Vector3()
    }

    if (msg.hMSL !== undefined) {
      resolved.hMSL = msg.hMSL;
    }
    else {
      resolved.hMSL = 0.0
    }

    if (msg.hAcc !== undefined) {
      resolved.hAcc = msg.hAcc;
    }
    else {
      resolved.hAcc = 0.0
    }

    if (msg.vAcc !== undefined) {
      resolved.vAcc = msg.vAcc;
    }
    else {
      resolved.vAcc = 0.0
    }

    if (msg.sAcc !== undefined) {
      resolved.sAcc = msg.sAcc;
    }
    else {
      resolved.sAcc = 0.0
    }

    if (msg.pDop !== undefined) {
      resolved.pDop = msg.pDop;
    }
    else {
      resolved.pDop = 0.0
    }

    return resolved;
    }
};

// Constants for message
GPS.Constants = {
  GPS_STATUS_FIX_TYPE_NO_FIX: 0,
  GPS_STATUS_FIX_TYPE_DEAD_RECKONING_ONLY: 256,
  GPS_STATUS_FIX_TYPE_2D_FIX: 512,
  GPS_STATUS_FIX_TYPE_3D_FIX: 768,
  GPS_STATUS_FIX_TYPE_GPS_PLUS_DEAD_RECK: 1024,
  GPS_STATUS_FIX_TYPE_TIME_ONLY_FIX: 1280,
  GPS_STATUS_FIX_TYPE_RESERVED1: 1536,
  GPS_STATUS_FIX_TYPE_RESERVED2: 1792,
  GPS_STATUS_FIX_STATUS_FIX_OK: 65536,
}

module.exports = GPS;
