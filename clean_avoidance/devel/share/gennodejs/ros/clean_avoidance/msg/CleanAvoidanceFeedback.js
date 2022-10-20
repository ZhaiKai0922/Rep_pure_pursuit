// Auto-generated. Do not edit!

// (in-package clean_avoidance.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class CleanAvoidanceFeedback {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.feedback_x = null;
      this.feedback_y = null;
    }
    else {
      if (initObj.hasOwnProperty('feedback_x')) {
        this.feedback_x = initObj.feedback_x
      }
      else {
        this.feedback_x = 0.0;
      }
      if (initObj.hasOwnProperty('feedback_y')) {
        this.feedback_y = initObj.feedback_y
      }
      else {
        this.feedback_y = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CleanAvoidanceFeedback
    // Serialize message field [feedback_x]
    bufferOffset = _serializer.float64(obj.feedback_x, buffer, bufferOffset);
    // Serialize message field [feedback_y]
    bufferOffset = _serializer.float64(obj.feedback_y, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CleanAvoidanceFeedback
    let len;
    let data = new CleanAvoidanceFeedback(null);
    // Deserialize message field [feedback_x]
    data.feedback_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [feedback_y]
    data.feedback_y = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'clean_avoidance/CleanAvoidanceFeedback';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '28b2f4c6bff997642d6c7dab1fe99c89';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    float64 feedback_x
    float64 feedback_y
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CleanAvoidanceFeedback(null);
    if (msg.feedback_x !== undefined) {
      resolved.feedback_x = msg.feedback_x;
    }
    else {
      resolved.feedback_x = 0.0
    }

    if (msg.feedback_y !== undefined) {
      resolved.feedback_y = msg.feedback_y;
    }
    else {
      resolved.feedback_y = 0.0
    }

    return resolved;
    }
};

module.exports = CleanAvoidanceFeedback;
