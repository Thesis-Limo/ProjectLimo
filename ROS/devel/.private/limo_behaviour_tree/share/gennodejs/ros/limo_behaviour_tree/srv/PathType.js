// Auto-generated. Do not edit!

// (in-package limo_behaviour_tree.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class PathTypeRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pathType = null;
    }
    else {
      if (initObj.hasOwnProperty('pathType')) {
        this.pathType = initObj.pathType
      }
      else {
        this.pathType = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PathTypeRequest
    // Serialize message field [pathType]
    bufferOffset = _serializer.int32(obj.pathType, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PathTypeRequest
    let len;
    let data = new PathTypeRequest(null);
    // Deserialize message field [pathType]
    data.pathType = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'limo_behaviour_tree/PathTypeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '11e5c53def65876fc1708cc2295e05a4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 pathType
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PathTypeRequest(null);
    if (msg.pathType !== undefined) {
      resolved.pathType = msg.pathType;
    }
    else {
      resolved.pathType = 0
    }

    return resolved;
    }
};

class PathTypeResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PathTypeResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PathTypeResponse
    let len;
    let data = new PathTypeResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'limo_behaviour_tree/PathTypeResponse';
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
    const resolved = new PathTypeResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: PathTypeRequest,
  Response: PathTypeResponse,
  md5sum() { return '11e5c53def65876fc1708cc2295e05a4'; },
  datatype() { return 'limo_behaviour_tree/PathType'; }
};
