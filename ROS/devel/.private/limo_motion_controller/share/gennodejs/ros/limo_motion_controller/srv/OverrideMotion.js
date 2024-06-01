// Auto-generated. Do not edit!

// (in-package limo_motion_controller.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class OverrideMotionRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.speed = null;
      this.angle = null;
      this.duration = null;
      this.sameSpeedStart = null;
    }
    else {
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0.0;
      }
      if (initObj.hasOwnProperty('angle')) {
        this.angle = initObj.angle
      }
      else {
        this.angle = 0.0;
      }
      if (initObj.hasOwnProperty('duration')) {
        this.duration = initObj.duration
      }
      else {
        this.duration = 0.0;
      }
      if (initObj.hasOwnProperty('sameSpeedStart')) {
        this.sameSpeedStart = initObj.sameSpeedStart
      }
      else {
        this.sameSpeedStart = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OverrideMotionRequest
    // Serialize message field [speed]
    bufferOffset = _serializer.float32(obj.speed, buffer, bufferOffset);
    // Serialize message field [angle]
    bufferOffset = _serializer.float32(obj.angle, buffer, bufferOffset);
    // Serialize message field [duration]
    bufferOffset = _serializer.float32(obj.duration, buffer, bufferOffset);
    // Serialize message field [sameSpeedStart]
    bufferOffset = _serializer.bool(obj.sameSpeedStart, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OverrideMotionRequest
    let len;
    let data = new OverrideMotionRequest(null);
    // Deserialize message field [speed]
    data.speed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angle]
    data.angle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [duration]
    data.duration = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [sameSpeedStart]
    data.sameSpeedStart = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 13;
  }

  static datatype() {
    // Returns string type for a service object
    return 'limo_motion_controller/OverrideMotionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bbdf834105919beb2a921bd852a73b0a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 speed
    float32 angle
    float32 duration
    bool sameSpeedStart
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new OverrideMotionRequest(null);
    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0.0
    }

    if (msg.angle !== undefined) {
      resolved.angle = msg.angle;
    }
    else {
      resolved.angle = 0.0
    }

    if (msg.duration !== undefined) {
      resolved.duration = msg.duration;
    }
    else {
      resolved.duration = 0.0
    }

    if (msg.sameSpeedStart !== undefined) {
      resolved.sameSpeedStart = msg.sameSpeedStart;
    }
    else {
      resolved.sameSpeedStart = false
    }

    return resolved;
    }
};

class OverrideMotionResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.alreadyRunning = null;
    }
    else {
      if (initObj.hasOwnProperty('alreadyRunning')) {
        this.alreadyRunning = initObj.alreadyRunning
      }
      else {
        this.alreadyRunning = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OverrideMotionResponse
    // Serialize message field [alreadyRunning]
    bufferOffset = _serializer.bool(obj.alreadyRunning, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OverrideMotionResponse
    let len;
    let data = new OverrideMotionResponse(null);
    // Deserialize message field [alreadyRunning]
    data.alreadyRunning = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'limo_motion_controller/OverrideMotionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '874749314d3cdd5b8d70590d992e9aad';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool alreadyRunning
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new OverrideMotionResponse(null);
    if (msg.alreadyRunning !== undefined) {
      resolved.alreadyRunning = msg.alreadyRunning;
    }
    else {
      resolved.alreadyRunning = false
    }

    return resolved;
    }
};

module.exports = {
  Request: OverrideMotionRequest,
  Response: OverrideMotionResponse,
  md5sum() { return '4955e86101808a9dfce48b4756da7f72'; },
  datatype() { return 'limo_motion_controller/OverrideMotion'; }
};
