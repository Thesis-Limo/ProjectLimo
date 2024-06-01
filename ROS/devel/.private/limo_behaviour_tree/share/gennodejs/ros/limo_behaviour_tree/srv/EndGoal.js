// Auto-generated. Do not edit!

// (in-package limo_behaviour_tree.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class EndGoalRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.goalPos = null;
    }
    else {
      if (initObj.hasOwnProperty('goalPos')) {
        this.goalPos = initObj.goalPos
      }
      else {
        this.goalPos = new geometry_msgs.msg.Point();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EndGoalRequest
    // Serialize message field [goalPos]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.goalPos, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EndGoalRequest
    let len;
    let data = new EndGoalRequest(null);
    // Deserialize message field [goalPos]
    data.goalPos = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'limo_behaviour_tree/EndGoalRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f43f04c866179f13e0ff9145477de5ab';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Point goalPos
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
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
    const resolved = new EndGoalRequest(null);
    if (msg.goalPos !== undefined) {
      resolved.goalPos = geometry_msgs.msg.Point.Resolve(msg.goalPos)
    }
    else {
      resolved.goalPos = new geometry_msgs.msg.Point()
    }

    return resolved;
    }
};

class EndGoalResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EndGoalResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EndGoalResponse
    let len;
    let data = new EndGoalResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'limo_behaviour_tree/EndGoalResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new EndGoalResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: EndGoalRequest,
  Response: EndGoalResponse,
  md5sum() { return 'f43f04c866179f13e0ff9145477de5ab'; },
  datatype() { return 'limo_behaviour_tree/EndGoal'; }
};
