// Auto-generated. Do not edit!

// (in-package syropod_highlevel_controller.msg)


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

class TipState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.name = null;
      this.wrench = null;
      this.step_plane = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = [];
      }
      if (initObj.hasOwnProperty('wrench')) {
        this.wrench = initObj.wrench
      }
      else {
        this.wrench = [];
      }
      if (initObj.hasOwnProperty('step_plane')) {
        this.step_plane = initObj.step_plane
      }
      else {
        this.step_plane = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TipState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [name]
    bufferOffset = _arraySerializer.string(obj.name, buffer, bufferOffset, null);
    // Serialize message field [wrench]
    // Serialize the length for message field [wrench]
    bufferOffset = _serializer.uint32(obj.wrench.length, buffer, bufferOffset);
    obj.wrench.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Wrench.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [step_plane]
    // Serialize the length for message field [step_plane]
    bufferOffset = _serializer.uint32(obj.step_plane.length, buffer, bufferOffset);
    obj.step_plane.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Vector3.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TipState
    let len;
    let data = new TipState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [name]
    data.name = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [wrench]
    // Deserialize array length for message field [wrench]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.wrench = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.wrench[i] = geometry_msgs.msg.Wrench.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [step_plane]
    // Deserialize array length for message field [step_plane]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.step_plane = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.step_plane[i] = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.name.forEach((val) => {
      length += 4 + val.length;
    });
    length += 48 * object.wrench.length;
    length += 24 * object.step_plane.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'syropod_highlevel_controller/TipState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '24a3486efb85ea2d52231aaad58ea97c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    string[] name
    geometry_msgs/Wrench[] wrench 
    geometry_msgs/Vector3[] step_plane
    
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
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Wrench
    # This represents force in free space, separated into
    # its linear and angular parts.
    Vector3  force
    Vector3  torque
    
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
    const resolved = new TipState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = []
    }

    if (msg.wrench !== undefined) {
      resolved.wrench = new Array(msg.wrench.length);
      for (let i = 0; i < resolved.wrench.length; ++i) {
        resolved.wrench[i] = geometry_msgs.msg.Wrench.Resolve(msg.wrench[i]);
      }
    }
    else {
      resolved.wrench = []
    }

    if (msg.step_plane !== undefined) {
      resolved.step_plane = new Array(msg.step_plane.length);
      for (let i = 0; i < resolved.step_plane.length; ++i) {
        resolved.step_plane[i] = geometry_msgs.msg.Vector3.Resolve(msg.step_plane[i]);
      }
    }
    else {
      resolved.step_plane = []
    }

    return resolved;
    }
};

module.exports = TipState;
