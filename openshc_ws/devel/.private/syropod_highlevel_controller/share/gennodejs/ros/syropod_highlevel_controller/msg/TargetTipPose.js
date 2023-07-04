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

class TargetTipPose {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.name = null;
      this.target = null;
      this.stance = null;
      this.swing_clearance = null;
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
      if (initObj.hasOwnProperty('target')) {
        this.target = initObj.target
      }
      else {
        this.target = [];
      }
      if (initObj.hasOwnProperty('stance')) {
        this.stance = initObj.stance
      }
      else {
        this.stance = [];
      }
      if (initObj.hasOwnProperty('swing_clearance')) {
        this.swing_clearance = initObj.swing_clearance
      }
      else {
        this.swing_clearance = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TargetTipPose
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [name]
    bufferOffset = _arraySerializer.string(obj.name, buffer, bufferOffset, null);
    // Serialize message field [target]
    // Serialize the length for message field [target]
    bufferOffset = _serializer.uint32(obj.target.length, buffer, bufferOffset);
    obj.target.forEach((val) => {
      bufferOffset = geometry_msgs.msg.PoseStamped.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [stance]
    // Serialize the length for message field [stance]
    bufferOffset = _serializer.uint32(obj.stance.length, buffer, bufferOffset);
    obj.stance.forEach((val) => {
      bufferOffset = geometry_msgs.msg.PoseStamped.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [swing_clearance]
    bufferOffset = _arraySerializer.float64(obj.swing_clearance, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TargetTipPose
    let len;
    let data = new TargetTipPose(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [name]
    data.name = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [target]
    // Deserialize array length for message field [target]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.target = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.target[i] = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [stance]
    // Deserialize array length for message field [stance]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.stance = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.stance[i] = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [swing_clearance]
    data.swing_clearance = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.name.forEach((val) => {
      length += 4 + val.length;
    });
    object.target.forEach((val) => {
      length += geometry_msgs.msg.PoseStamped.getMessageSize(val);
    });
    object.stance.forEach((val) => {
      length += geometry_msgs.msg.PoseStamped.getMessageSize(val);
    });
    length += 8 * object.swing_clearance.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'syropod_highlevel_controller/TargetTipPose';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '05c537cc093e23228ee76fae952f2b3e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    string[] name
    geometry_msgs/PoseStamped[] target
    geometry_msgs/PoseStamped[] stance
    float64[] swing_clearance
    
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
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TargetTipPose(null);
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

    if (msg.target !== undefined) {
      resolved.target = new Array(msg.target.length);
      for (let i = 0; i < resolved.target.length; ++i) {
        resolved.target[i] = geometry_msgs.msg.PoseStamped.Resolve(msg.target[i]);
      }
    }
    else {
      resolved.target = []
    }

    if (msg.stance !== undefined) {
      resolved.stance = new Array(msg.stance.length);
      for (let i = 0; i < resolved.stance.length; ++i) {
        resolved.stance[i] = geometry_msgs.msg.PoseStamped.Resolve(msg.stance[i]);
      }
    }
    else {
      resolved.stance = []
    }

    if (msg.swing_clearance !== undefined) {
      resolved.swing_clearance = msg.swing_clearance;
    }
    else {
      resolved.swing_clearance = []
    }

    return resolved;
    }
};

module.exports = TargetTipPose;
