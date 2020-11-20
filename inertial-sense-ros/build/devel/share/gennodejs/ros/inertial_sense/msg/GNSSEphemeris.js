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

class GNSSEphemeris {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.sat = null;
      this.iode = null;
      this.iodc = null;
      this.sva = null;
      this.svh = null;
      this.week = null;
      this.code = null;
      this.flag = null;
      this.toe = null;
      this.toc = null;
      this.ttr = null;
      this.A = null;
      this.e = null;
      this.i0 = null;
      this.OMG0 = null;
      this.omg = null;
      this.M0 = null;
      this.deln = null;
      this.OMGd = null;
      this.idot = null;
      this.crc = null;
      this.crs = null;
      this.cuc = null;
      this.cus = null;
      this.cic = null;
      this.cis = null;
      this.toes = null;
      this.fit = null;
      this.f0 = null;
      this.f1 = null;
      this.f2 = null;
      this.tgd = null;
      this.Adot = null;
      this.ndot = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('sat')) {
        this.sat = initObj.sat
      }
      else {
        this.sat = 0;
      }
      if (initObj.hasOwnProperty('iode')) {
        this.iode = initObj.iode
      }
      else {
        this.iode = 0;
      }
      if (initObj.hasOwnProperty('iodc')) {
        this.iodc = initObj.iodc
      }
      else {
        this.iodc = 0;
      }
      if (initObj.hasOwnProperty('sva')) {
        this.sva = initObj.sva
      }
      else {
        this.sva = 0;
      }
      if (initObj.hasOwnProperty('svh')) {
        this.svh = initObj.svh
      }
      else {
        this.svh = 0;
      }
      if (initObj.hasOwnProperty('week')) {
        this.week = initObj.week
      }
      else {
        this.week = 0;
      }
      if (initObj.hasOwnProperty('code')) {
        this.code = initObj.code
      }
      else {
        this.code = 0;
      }
      if (initObj.hasOwnProperty('flag')) {
        this.flag = initObj.flag
      }
      else {
        this.flag = 0;
      }
      if (initObj.hasOwnProperty('toe')) {
        this.toe = initObj.toe
      }
      else {
        this.toe = new GTime();
      }
      if (initObj.hasOwnProperty('toc')) {
        this.toc = initObj.toc
      }
      else {
        this.toc = new GTime();
      }
      if (initObj.hasOwnProperty('ttr')) {
        this.ttr = initObj.ttr
      }
      else {
        this.ttr = new GTime();
      }
      if (initObj.hasOwnProperty('A')) {
        this.A = initObj.A
      }
      else {
        this.A = 0.0;
      }
      if (initObj.hasOwnProperty('e')) {
        this.e = initObj.e
      }
      else {
        this.e = 0.0;
      }
      if (initObj.hasOwnProperty('i0')) {
        this.i0 = initObj.i0
      }
      else {
        this.i0 = 0.0;
      }
      if (initObj.hasOwnProperty('OMG0')) {
        this.OMG0 = initObj.OMG0
      }
      else {
        this.OMG0 = 0.0;
      }
      if (initObj.hasOwnProperty('omg')) {
        this.omg = initObj.omg
      }
      else {
        this.omg = 0.0;
      }
      if (initObj.hasOwnProperty('M0')) {
        this.M0 = initObj.M0
      }
      else {
        this.M0 = 0.0;
      }
      if (initObj.hasOwnProperty('deln')) {
        this.deln = initObj.deln
      }
      else {
        this.deln = 0.0;
      }
      if (initObj.hasOwnProperty('OMGd')) {
        this.OMGd = initObj.OMGd
      }
      else {
        this.OMGd = 0.0;
      }
      if (initObj.hasOwnProperty('idot')) {
        this.idot = initObj.idot
      }
      else {
        this.idot = 0.0;
      }
      if (initObj.hasOwnProperty('crc')) {
        this.crc = initObj.crc
      }
      else {
        this.crc = 0.0;
      }
      if (initObj.hasOwnProperty('crs')) {
        this.crs = initObj.crs
      }
      else {
        this.crs = 0.0;
      }
      if (initObj.hasOwnProperty('cuc')) {
        this.cuc = initObj.cuc
      }
      else {
        this.cuc = 0.0;
      }
      if (initObj.hasOwnProperty('cus')) {
        this.cus = initObj.cus
      }
      else {
        this.cus = 0.0;
      }
      if (initObj.hasOwnProperty('cic')) {
        this.cic = initObj.cic
      }
      else {
        this.cic = 0.0;
      }
      if (initObj.hasOwnProperty('cis')) {
        this.cis = initObj.cis
      }
      else {
        this.cis = 0.0;
      }
      if (initObj.hasOwnProperty('toes')) {
        this.toes = initObj.toes
      }
      else {
        this.toes = 0.0;
      }
      if (initObj.hasOwnProperty('fit')) {
        this.fit = initObj.fit
      }
      else {
        this.fit = 0.0;
      }
      if (initObj.hasOwnProperty('f0')) {
        this.f0 = initObj.f0
      }
      else {
        this.f0 = 0.0;
      }
      if (initObj.hasOwnProperty('f1')) {
        this.f1 = initObj.f1
      }
      else {
        this.f1 = 0.0;
      }
      if (initObj.hasOwnProperty('f2')) {
        this.f2 = initObj.f2
      }
      else {
        this.f2 = 0.0;
      }
      if (initObj.hasOwnProperty('tgd')) {
        this.tgd = initObj.tgd
      }
      else {
        this.tgd = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('Adot')) {
        this.Adot = initObj.Adot
      }
      else {
        this.Adot = 0.0;
      }
      if (initObj.hasOwnProperty('ndot')) {
        this.ndot = initObj.ndot
      }
      else {
        this.ndot = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GNSSEphemeris
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [sat]
    bufferOffset = _serializer.int32(obj.sat, buffer, bufferOffset);
    // Serialize message field [iode]
    bufferOffset = _serializer.int32(obj.iode, buffer, bufferOffset);
    // Serialize message field [iodc]
    bufferOffset = _serializer.int32(obj.iodc, buffer, bufferOffset);
    // Serialize message field [sva]
    bufferOffset = _serializer.int32(obj.sva, buffer, bufferOffset);
    // Serialize message field [svh]
    bufferOffset = _serializer.int32(obj.svh, buffer, bufferOffset);
    // Serialize message field [week]
    bufferOffset = _serializer.int32(obj.week, buffer, bufferOffset);
    // Serialize message field [code]
    bufferOffset = _serializer.int32(obj.code, buffer, bufferOffset);
    // Serialize message field [flag]
    bufferOffset = _serializer.int32(obj.flag, buffer, bufferOffset);
    // Serialize message field [toe]
    bufferOffset = GTime.serialize(obj.toe, buffer, bufferOffset);
    // Serialize message field [toc]
    bufferOffset = GTime.serialize(obj.toc, buffer, bufferOffset);
    // Serialize message field [ttr]
    bufferOffset = GTime.serialize(obj.ttr, buffer, bufferOffset);
    // Serialize message field [A]
    bufferOffset = _serializer.float64(obj.A, buffer, bufferOffset);
    // Serialize message field [e]
    bufferOffset = _serializer.float64(obj.e, buffer, bufferOffset);
    // Serialize message field [i0]
    bufferOffset = _serializer.float64(obj.i0, buffer, bufferOffset);
    // Serialize message field [OMG0]
    bufferOffset = _serializer.float64(obj.OMG0, buffer, bufferOffset);
    // Serialize message field [omg]
    bufferOffset = _serializer.float64(obj.omg, buffer, bufferOffset);
    // Serialize message field [M0]
    bufferOffset = _serializer.float64(obj.M0, buffer, bufferOffset);
    // Serialize message field [deln]
    bufferOffset = _serializer.float64(obj.deln, buffer, bufferOffset);
    // Serialize message field [OMGd]
    bufferOffset = _serializer.float64(obj.OMGd, buffer, bufferOffset);
    // Serialize message field [idot]
    bufferOffset = _serializer.float64(obj.idot, buffer, bufferOffset);
    // Serialize message field [crc]
    bufferOffset = _serializer.float64(obj.crc, buffer, bufferOffset);
    // Serialize message field [crs]
    bufferOffset = _serializer.float64(obj.crs, buffer, bufferOffset);
    // Serialize message field [cuc]
    bufferOffset = _serializer.float64(obj.cuc, buffer, bufferOffset);
    // Serialize message field [cus]
    bufferOffset = _serializer.float64(obj.cus, buffer, bufferOffset);
    // Serialize message field [cic]
    bufferOffset = _serializer.float64(obj.cic, buffer, bufferOffset);
    // Serialize message field [cis]
    bufferOffset = _serializer.float64(obj.cis, buffer, bufferOffset);
    // Serialize message field [toes]
    bufferOffset = _serializer.float64(obj.toes, buffer, bufferOffset);
    // Serialize message field [fit]
    bufferOffset = _serializer.float64(obj.fit, buffer, bufferOffset);
    // Serialize message field [f0]
    bufferOffset = _serializer.float64(obj.f0, buffer, bufferOffset);
    // Serialize message field [f1]
    bufferOffset = _serializer.float64(obj.f1, buffer, bufferOffset);
    // Serialize message field [f2]
    bufferOffset = _serializer.float64(obj.f2, buffer, bufferOffset);
    // Check that the constant length array field [tgd] has the right length
    if (obj.tgd.length !== 4) {
      throw new Error('Unable to serialize array field tgd - length must be 4')
    }
    // Serialize message field [tgd]
    bufferOffset = _arraySerializer.float64(obj.tgd, buffer, bufferOffset, 4);
    // Serialize message field [Adot]
    bufferOffset = _serializer.float64(obj.Adot, buffer, bufferOffset);
    // Serialize message field [ndot]
    bufferOffset = _serializer.float64(obj.ndot, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GNSSEphemeris
    let len;
    let data = new GNSSEphemeris(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [sat]
    data.sat = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [iode]
    data.iode = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [iodc]
    data.iodc = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [sva]
    data.sva = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [svh]
    data.svh = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [week]
    data.week = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [code]
    data.code = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [flag]
    data.flag = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [toe]
    data.toe = GTime.deserialize(buffer, bufferOffset);
    // Deserialize message field [toc]
    data.toc = GTime.deserialize(buffer, bufferOffset);
    // Deserialize message field [ttr]
    data.ttr = GTime.deserialize(buffer, bufferOffset);
    // Deserialize message field [A]
    data.A = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [e]
    data.e = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [i0]
    data.i0 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [OMG0]
    data.OMG0 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [omg]
    data.omg = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [M0]
    data.M0 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [deln]
    data.deln = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [OMGd]
    data.OMGd = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [idot]
    data.idot = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [crc]
    data.crc = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [crs]
    data.crs = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [cuc]
    data.cuc = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [cus]
    data.cus = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [cic]
    data.cic = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [cis]
    data.cis = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [toes]
    data.toes = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [fit]
    data.fit = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [f0]
    data.f0 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [f1]
    data.f1 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [f2]
    data.f2 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [tgd]
    data.tgd = _arrayDeserializer.float64(buffer, bufferOffset, 4)
    // Deserialize message field [Adot]
    data.Adot = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ndot]
    data.ndot = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 288;
  }

  static datatype() {
    // Returns string type for a message object
    return 'inertial_sense/GNSSEphemeris';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b950a03f405d085580c4de95aeee72ef';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    int32 sat 		# satellite number
    int32 iode 		# IODE Issue of Data, Ephemeris (ephemeris version)
    int32 iodc 		# IODC Issue of Data, Clock (clock version)
    int32 sva 		# SV accuracy (URA index) IRN-IS-200H p.97            
    int32 svh 		# SV health GPS/QZS (0:ok)            
    int32 week 		# GPS/QZS: gps week, GAL: galileo week
    int32 code 		# GPS/QZS: code on L2 * (00=Invalid, 01 = P Code ON, 11 = C/A code ON, 11 = Invalid) * GAL/CMP: data sources
    int32 flag 		# GPS/QZS: L2 P data flag (indicates that the NAV data stream was commanded OFF on the P-code of the in-phase component of the L2 channel) *  CMP: nav type
    GTime toe 		# Toe
    GTime toc 		# clock data reference time (s) (20.3.4.5)
    GTime ttr 		# T_trans
    float64 A 		# Semi-Major Axis m
    float64 e 		# Eccentricity (no units) 
    float64 i0 		# Inclination Angle at Reference Time (rad)
    float64 OMG0 	# Longitude of Ascending Node of Orbit Plane at Weekly Epoch (rad)
    float64 omg 	# Argument of Perigee (rad)
    float64 M0 		# Mean Anomaly at Reference Time (rad)
    float64 deln 	# Mean Motion Difference From Computed Value (rad)
    float64 OMGd 	# Rate of Right Ascension (rad/s)
    float64 idot 	# Rate of Inclination Angle (rad/s)
    float64 crc 	# Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius
    float64 crs 	# Amplitude of the Sine Harmonic Correction Term to the Orbit Radius (m)
    float64 cuc 	# Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude (rad) 
    float64 cus 	# Amplitude of the Sine Harmonic Correction Term to the Argument of Latitude (rad)
    float64 cic 	# Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination (rad)
    float64 cis 	# Amplitude of the Sine Harmonic Correction Term to the Angle of Inclination (rad)
    float64 toes 	# Reference Time Ephemeris in week (s)
    float64 fit 	# fit interval (h) (0: 4 hours, 1:greater than 4 hours)
    float64 f0 		# SV clock parameters - af0
    float64 f1 		# SV clock parameters - af1
    float64 f2 		# SV clock parameters - af2
    float64[4] tgd 	# group delay parameters: GPS/QZS:tgd[0]=TGD (IRN-IS-200H p.103) * GAL:tgd[0]=BGD E5a/E1,tgd[1]=BGD E5b/E1	* CMP :tgd[0]=BGD1,tgd[1]=BGD2
    float64 Adot 	# Adot for CNAV
    float64 ndot 	# ndot for CNAV
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
    const resolved = new GNSSEphemeris(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.sat !== undefined) {
      resolved.sat = msg.sat;
    }
    else {
      resolved.sat = 0
    }

    if (msg.iode !== undefined) {
      resolved.iode = msg.iode;
    }
    else {
      resolved.iode = 0
    }

    if (msg.iodc !== undefined) {
      resolved.iodc = msg.iodc;
    }
    else {
      resolved.iodc = 0
    }

    if (msg.sva !== undefined) {
      resolved.sva = msg.sva;
    }
    else {
      resolved.sva = 0
    }

    if (msg.svh !== undefined) {
      resolved.svh = msg.svh;
    }
    else {
      resolved.svh = 0
    }

    if (msg.week !== undefined) {
      resolved.week = msg.week;
    }
    else {
      resolved.week = 0
    }

    if (msg.code !== undefined) {
      resolved.code = msg.code;
    }
    else {
      resolved.code = 0
    }

    if (msg.flag !== undefined) {
      resolved.flag = msg.flag;
    }
    else {
      resolved.flag = 0
    }

    if (msg.toe !== undefined) {
      resolved.toe = GTime.Resolve(msg.toe)
    }
    else {
      resolved.toe = new GTime()
    }

    if (msg.toc !== undefined) {
      resolved.toc = GTime.Resolve(msg.toc)
    }
    else {
      resolved.toc = new GTime()
    }

    if (msg.ttr !== undefined) {
      resolved.ttr = GTime.Resolve(msg.ttr)
    }
    else {
      resolved.ttr = new GTime()
    }

    if (msg.A !== undefined) {
      resolved.A = msg.A;
    }
    else {
      resolved.A = 0.0
    }

    if (msg.e !== undefined) {
      resolved.e = msg.e;
    }
    else {
      resolved.e = 0.0
    }

    if (msg.i0 !== undefined) {
      resolved.i0 = msg.i0;
    }
    else {
      resolved.i0 = 0.0
    }

    if (msg.OMG0 !== undefined) {
      resolved.OMG0 = msg.OMG0;
    }
    else {
      resolved.OMG0 = 0.0
    }

    if (msg.omg !== undefined) {
      resolved.omg = msg.omg;
    }
    else {
      resolved.omg = 0.0
    }

    if (msg.M0 !== undefined) {
      resolved.M0 = msg.M0;
    }
    else {
      resolved.M0 = 0.0
    }

    if (msg.deln !== undefined) {
      resolved.deln = msg.deln;
    }
    else {
      resolved.deln = 0.0
    }

    if (msg.OMGd !== undefined) {
      resolved.OMGd = msg.OMGd;
    }
    else {
      resolved.OMGd = 0.0
    }

    if (msg.idot !== undefined) {
      resolved.idot = msg.idot;
    }
    else {
      resolved.idot = 0.0
    }

    if (msg.crc !== undefined) {
      resolved.crc = msg.crc;
    }
    else {
      resolved.crc = 0.0
    }

    if (msg.crs !== undefined) {
      resolved.crs = msg.crs;
    }
    else {
      resolved.crs = 0.0
    }

    if (msg.cuc !== undefined) {
      resolved.cuc = msg.cuc;
    }
    else {
      resolved.cuc = 0.0
    }

    if (msg.cus !== undefined) {
      resolved.cus = msg.cus;
    }
    else {
      resolved.cus = 0.0
    }

    if (msg.cic !== undefined) {
      resolved.cic = msg.cic;
    }
    else {
      resolved.cic = 0.0
    }

    if (msg.cis !== undefined) {
      resolved.cis = msg.cis;
    }
    else {
      resolved.cis = 0.0
    }

    if (msg.toes !== undefined) {
      resolved.toes = msg.toes;
    }
    else {
      resolved.toes = 0.0
    }

    if (msg.fit !== undefined) {
      resolved.fit = msg.fit;
    }
    else {
      resolved.fit = 0.0
    }

    if (msg.f0 !== undefined) {
      resolved.f0 = msg.f0;
    }
    else {
      resolved.f0 = 0.0
    }

    if (msg.f1 !== undefined) {
      resolved.f1 = msg.f1;
    }
    else {
      resolved.f1 = 0.0
    }

    if (msg.f2 !== undefined) {
      resolved.f2 = msg.f2;
    }
    else {
      resolved.f2 = 0.0
    }

    if (msg.tgd !== undefined) {
      resolved.tgd = msg.tgd;
    }
    else {
      resolved.tgd = new Array(4).fill(0)
    }

    if (msg.Adot !== undefined) {
      resolved.Adot = msg.Adot;
    }
    else {
      resolved.Adot = 0.0
    }

    if (msg.ndot !== undefined) {
      resolved.ndot = msg.ndot;
    }
    else {
      resolved.ndot = 0.0
    }

    return resolved;
    }
};

module.exports = GNSSEphemeris;
