// Auto-generated. Do not edit!

// (in-package syropod_remote.msg)


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

class AndroidSensor {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.id_name = null;
      this.override_priority_interface = null;
      this.orientation = null;
      this.relative_compass = null;
      this.robot_state = null;
      this.posing_mode = null;
      this.control_axis = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('id_name')) {
        this.id_name = initObj.id_name
      }
      else {
        this.id_name = new std_msgs.msg.String();
      }
      if (initObj.hasOwnProperty('override_priority_interface')) {
        this.override_priority_interface = initObj.override_priority_interface
      }
      else {
        this.override_priority_interface = new std_msgs.msg.Bool();
      }
      if (initObj.hasOwnProperty('orientation')) {
        this.orientation = initObj.orientation
      }
      else {
        this.orientation = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('relative_compass')) {
        this.relative_compass = initObj.relative_compass
      }
      else {
        this.relative_compass = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('robot_state')) {
        this.robot_state = initObj.robot_state
      }
      else {
        this.robot_state = new std_msgs.msg.Int8();
      }
      if (initObj.hasOwnProperty('posing_mode')) {
        this.posing_mode = initObj.posing_mode
      }
      else {
        this.posing_mode = new std_msgs.msg.Int8();
      }
      if (initObj.hasOwnProperty('control_axis')) {
        this.control_axis = initObj.control_axis
      }
      else {
        this.control_axis = new geometry_msgs.msg.Point();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AndroidSensor
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [id_name]
    bufferOffset = std_msgs.msg.String.serialize(obj.id_name, buffer, bufferOffset);
    // Serialize message field [override_priority_interface]
    bufferOffset = std_msgs.msg.Bool.serialize(obj.override_priority_interface, buffer, bufferOffset);
    // Serialize message field [orientation]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.orientation, buffer, bufferOffset);
    // Serialize message field [relative_compass]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.relative_compass, buffer, bufferOffset);
    // Serialize message field [robot_state]
    bufferOffset = std_msgs.msg.Int8.serialize(obj.robot_state, buffer, bufferOffset);
    // Serialize message field [posing_mode]
    bufferOffset = std_msgs.msg.Int8.serialize(obj.posing_mode, buffer, bufferOffset);
    // Serialize message field [control_axis]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.control_axis, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AndroidSensor
    let len;
    let data = new AndroidSensor(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [id_name]
    data.id_name = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    // Deserialize message field [override_priority_interface]
    data.override_priority_interface = std_msgs.msg.Bool.deserialize(buffer, bufferOffset);
    // Deserialize message field [orientation]
    data.orientation = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [relative_compass]
    data.relative_compass = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [robot_state]
    data.robot_state = std_msgs.msg.Int8.deserialize(buffer, bufferOffset);
    // Deserialize message field [posing_mode]
    data.posing_mode = std_msgs.msg.Int8.deserialize(buffer, bufferOffset);
    // Deserialize message field [control_axis]
    data.control_axis = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += std_msgs.msg.String.getMessageSize(object.id_name);
    return length + 59;
  }

  static datatype() {
    // Returns string type for a message object
    return 'syropod_remote/AndroidSensor';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '05a22ff6b1072fe74d2077e0d442d058';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    std_msgs/String id_name
    std_msgs/Bool override_priority_interface
    geometry_msgs/Point orientation
    std_msgs/Float64 relative_compass
    std_msgs/Int8 robot_state
    std_msgs/Int8 posing_mode
    geometry_msgs/Point control_axis
    
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
    MSG: std_msgs/String
    string data
    
    ================================================================================
    MSG: std_msgs/Bool
    bool data
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: std_msgs/Float64
    float64 data
    ================================================================================
    MSG: std_msgs/Int8
    int8 data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AndroidSensor(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.id_name !== undefined) {
      resolved.id_name = std_msgs.msg.String.Resolve(msg.id_name)
    }
    else {
      resolved.id_name = new std_msgs.msg.String()
    }

    if (msg.override_priority_interface !== undefined) {
      resolved.override_priority_interface = std_msgs.msg.Bool.Resolve(msg.override_priority_interface)
    }
    else {
      resolved.override_priority_interface = new std_msgs.msg.Bool()
    }

    if (msg.orientation !== undefined) {
      resolved.orientation = geometry_msgs.msg.Point.Resolve(msg.orientation)
    }
    else {
      resolved.orientation = new geometry_msgs.msg.Point()
    }

    if (msg.relative_compass !== undefined) {
      resolved.relative_compass = std_msgs.msg.Float64.Resolve(msg.relative_compass)
    }
    else {
      resolved.relative_compass = new std_msgs.msg.Float64()
    }

    if (msg.robot_state !== undefined) {
      resolved.robot_state = std_msgs.msg.Int8.Resolve(msg.robot_state)
    }
    else {
      resolved.robot_state = new std_msgs.msg.Int8()
    }

    if (msg.posing_mode !== undefined) {
      resolved.posing_mode = std_msgs.msg.Int8.Resolve(msg.posing_mode)
    }
    else {
      resolved.posing_mode = new std_msgs.msg.Int8()
    }

    if (msg.control_axis !== undefined) {
      resolved.control_axis = geometry_msgs.msg.Point.Resolve(msg.control_axis)
    }
    else {
      resolved.control_axis = new geometry_msgs.msg.Point()
    }

    return resolved;
    }
};

module.exports = AndroidSensor;
