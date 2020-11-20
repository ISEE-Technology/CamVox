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

//-----------------------------------------------------------

class GlonassEphemeris {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sat = null;
      this.iode = null;
      this.frq = null;
      this.svh = null;
      this.sva = null;
      this.age = null;
      this.toe = null;
      this.tof = null;
      this.pos = null;
      this.vel = null;
      this.acc = null;
      this.taun = null;
      this.gamn = null;
      this.dtaun = null;
    }
    else {
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
      if (initObj.hasOwnProperty('frq')) {
        this.frq = initObj.frq
      }
      else {
        this.frq = 0;
      }
      if (initObj.hasOwnProperty('svh')) {
        this.svh = initObj.svh
      }
      else {
        this.svh = 0;
      }
      if (initObj.hasOwnProperty('sva')) {
        this.sva = initObj.sva
      }
      else {
        this.sva = 0;
      }
      if (initObj.hasOwnProperty('age')) {
        this.age = initObj.age
      }
      else {
        this.age = 0;
      }
      if (initObj.hasOwnProperty('toe')) {
        this.toe = initObj.toe
      }
      else {
        this.toe = new GTime();
      }
      if (initObj.hasOwnProperty('tof')) {
        this.tof = initObj.tof
      }
      else {
        this.tof = new GTime();
      }
      if (initObj.hasOwnProperty('pos')) {
        this.pos = initObj.pos
      }
      else {
        this.pos = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('vel')) {
        this.vel = initObj.vel
      }
      else {
        this.vel = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('acc')) {
        this.acc = initObj.acc
      }
      else {
        this.acc = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('taun')) {
        this.taun = initObj.taun
      }
      else {
        this.taun = 0.0;
      }
      if (initObj.hasOwnProperty('gamn')) {
        this.gamn = initObj.gamn
      }
      else {
        this.gamn = 0.0;
      }
      if (initObj.hasOwnProperty('dtaun')) {
        this.dtaun = initObj.dtaun
      }
      else {
        this.dtaun = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GlonassEphemeris
    // Serialize message field [sat]
    bufferOffset = _serializer.int32(obj.sat, buffer, bufferOffset);
    // Serialize message field [iode]
    bufferOffset = _serializer.int32(obj.iode, buffer, bufferOffset);
    // Serialize message field [frq]
    bufferOffset = _serializer.int32(obj.frq, buffer, bufferOffset);
    // Serialize message field [svh]
    bufferOffset = _serializer.int32(obj.svh, buffer, bufferOffset);
    // Serialize message field [sva]
    bufferOffset = _serializer.int32(obj.sva, buffer, bufferOffset);
    // Serialize message field [age]
    bufferOffset = _serializer.int32(obj.age, buffer, bufferOffset);
    // Serialize message field [toe]
    bufferOffset = GTime.serialize(obj.toe, buffer, bufferOffset);
    // Serialize message field [tof]
    bufferOffset = GTime.serialize(obj.tof, buffer, bufferOffset);
    // Check that the constant length array field [pos] has the right length
    if (obj.pos.length !== 3) {
      throw new Error('Unable to serialize array field pos - length must be 3')
    }
    // Serialize message field [pos]
    bufferOffset = _arraySerializer.float64(obj.pos, buffer, bufferOffset, 3);
    // Check that the constant length array field [vel] has the right length
    if (obj.vel.length !== 3) {
      throw new Error('Unable to serialize array field vel - length must be 3')
    }
    // Serialize message field [vel]
    bufferOffset = _arraySerializer.float64(obj.vel, buffer, bufferOffset, 3);
    // Check that the constant length array field [acc] has the right length
    if (obj.acc.length !== 3) {
      throw new Error('Unable to serialize array field acc - length must be 3')
    }
    // Serialize message field [acc]
    bufferOffset = _arraySerializer.float64(obj.acc, buffer, bufferOffset, 3);
    // Serialize message field [taun]
    bufferOffset = _serializer.float64(obj.taun, buffer, bufferOffset);
    // Serialize message field [gamn]
    bufferOffset = _serializer.float64(obj.gamn, buffer, bufferOffset);
    // Serialize message field [dtaun]
    bufferOffset = _serializer.float64(obj.dtaun, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GlonassEphemeris
    let len;
    let data = new GlonassEphemeris(null);
    // Deserialize message field [sat]
    data.sat = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [iode]
    data.iode = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [frq]
    data.frq = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [svh]
    data.svh = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [sva]
    data.sva = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [age]
    data.age = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [toe]
    data.toe = GTime.deserialize(buffer, bufferOffset);
    // Deserialize message field [tof]
    data.tof = GTime.deserialize(buffer, bufferOffset);
    // Deserialize message field [pos]
    data.pos = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [vel]
    data.vel = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [acc]
    data.acc = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [taun]
    data.taun = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [gamn]
    data.gamn = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [dtaun]
    data.dtaun = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 152;
  }

  static datatype() {
    // Returns string type for a message object
    return 'inertial_sense/GlonassEphemeris';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd6e50080404485ddd899caa0ddef8be5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 sat 	# satellite number 
    int32 iode 	# IODE (0-6 bit of tb field) 
    int32 frq 	# satellite frequency number 
    int32 svh 	# satellite health 
    int32 sva 	# satellite accuracy 
    int32 age 	# satellite age of operation 
    GTime toe 	# epoch of epherides (gpst) 
    GTime tof 	# message frame time (gpst) 
    float64[3] pos 	# satellite position (ecef) (m) 
    float64[3] vel 	# satellite velocity (ecef) (m/s) 
    float64[3] acc 	# satellite acceleration (ecef) (m/s^2) 
    float64 taun 	# SV clock bias (s) 
    float64 gamn 	# relative freq bias 
    float64 dtaun 	# delay between L1 and L2 (s) 
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
    const resolved = new GlonassEphemeris(null);
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

    if (msg.frq !== undefined) {
      resolved.frq = msg.frq;
    }
    else {
      resolved.frq = 0
    }

    if (msg.svh !== undefined) {
      resolved.svh = msg.svh;
    }
    else {
      resolved.svh = 0
    }

    if (msg.sva !== undefined) {
      resolved.sva = msg.sva;
    }
    else {
      resolved.sva = 0
    }

    if (msg.age !== undefined) {
      resolved.age = msg.age;
    }
    else {
      resolved.age = 0
    }

    if (msg.toe !== undefined) {
      resolved.toe = GTime.Resolve(msg.toe)
    }
    else {
      resolved.toe = new GTime()
    }

    if (msg.tof !== undefined) {
      resolved.tof = GTime.Resolve(msg.tof)
    }
    else {
      resolved.tof = new GTime()
    }

    if (msg.pos !== undefined) {
      resolved.pos = msg.pos;
    }
    else {
      resolved.pos = new Array(3).fill(0)
    }

    if (msg.vel !== undefined) {
      resolved.vel = msg.vel;
    }
    else {
      resolved.vel = new Array(3).fill(0)
    }

    if (msg.acc !== undefined) {
      resolved.acc = msg.acc;
    }
    else {
      resolved.acc = new Array(3).fill(0)
    }

    if (msg.taun !== undefined) {
      resolved.taun = msg.taun;
    }
    else {
      resolved.taun = 0.0
    }

    if (msg.gamn !== undefined) {
      resolved.gamn = msg.gamn;
    }
    else {
      resolved.gamn = 0.0
    }

    if (msg.dtaun !== undefined) {
      resolved.dtaun = msg.dtaun;
    }
    else {
      resolved.dtaun = 0.0
    }

    return resolved;
    }
};

module.exports = GlonassEphemeris;
