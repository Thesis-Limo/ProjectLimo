// Auto-generated. Do not edit!

// (in-package limo_motion_controller.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let MovementController = require('./MovementController.js');

//-----------------------------------------------------------

class MotionPlan {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sequence = null;
    }
    else {
      if (initObj.hasOwnProperty('sequence')) {
        this.sequence = initObj.sequence
      }
      else {
        this.sequence = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotionPlan
    // Serialize message field [sequence]
    // Serialize the length for message field [sequence]
    bufferOffset = _serializer.uint32(obj.sequence.length, buffer, bufferOffset);
    obj.sequence.forEach((val) => {
      bufferOffset = MovementController.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotionPlan
    let len;
    let data = new MotionPlan(null);
    // Deserialize message field [sequence]
    // Deserialize array length for message field [sequence]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.sequence = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.sequence[i] = MovementController.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 12 * object.sequence.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'limo_motion_controller/MotionPlan';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '862f364494b8552b6422445d2907f57e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    limo_motion_controller/MovementController[] sequence
    ================================================================================
    MSG: limo_motion_controller/MovementController
    float32 speed
    float32 angle
    float32 duration
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MotionPlan(null);
    if (msg.sequence !== undefined) {
      resolved.sequence = new Array(msg.sequence.length);
      for (let i = 0; i < resolved.sequence.length; ++i) {
        resolved.sequence[i] = MovementController.Resolve(msg.sequence[i]);
      }
    }
    else {
      resolved.sequence = []
    }

    return resolved;
    }
};

module.exports = MotionPlan;
