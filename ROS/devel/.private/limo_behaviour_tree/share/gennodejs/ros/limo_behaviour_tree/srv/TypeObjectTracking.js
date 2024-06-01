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

class TypeObjectTrackingRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.objectID = null;
    }
    else {
      if (initObj.hasOwnProperty('objectID')) {
        this.objectID = initObj.objectID
      }
      else {
        this.objectID = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TypeObjectTrackingRequest
    // Serialize message field [objectID]
    bufferOffset = _serializer.int32(obj.objectID, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TypeObjectTrackingRequest
    let len;
    let data = new TypeObjectTrackingRequest(null);
    // Deserialize message field [objectID]
    data.objectID = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'limo_behaviour_tree/TypeObjectTrackingRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0b08c3191adc4004289244e57c20d2f6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 objectID
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TypeObjectTrackingRequest(null);
    if (msg.objectID !== undefined) {
      resolved.objectID = msg.objectID;
    }
    else {
      resolved.objectID = 0
    }

    return resolved;
    }
};

class TypeObjectTrackingResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TypeObjectTrackingResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TypeObjectTrackingResponse
    let len;
    let data = new TypeObjectTrackingResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'limo_behaviour_tree/TypeObjectTrackingResponse';
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
    const resolved = new TypeObjectTrackingResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: TypeObjectTrackingRequest,
  Response: TypeObjectTrackingResponse,
  md5sum() { return '0b08c3191adc4004289244e57c20d2f6'; },
  datatype() { return 'limo_behaviour_tree/TypeObjectTracking'; }
};
