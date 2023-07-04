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

class AndroidJoy {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.id_name = null;
      this.override_priority_interface = null;
      this.primary_control_axis = null;
      this.secondary_control_axis = null;
      this.system_state = null;
      this.robot_state = null;
      this.gait_selection = null;
      this.cruise_control_mode = null;
      this.auto_navigation_mode = null;
      this.posing_mode = null;
      this.pose_reset_mode = null;
      this.primary_leg_selection = null;
      this.secondary_leg_selection = null;
      this.primary_leg_state = null;
      this.secondary_leg_state = null;
      this.parameter_selection = null;
      this.parameter_adjustment = null;
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
      if (initObj.hasOwnProperty('primary_control_axis')) {
        this.primary_control_axis = initObj.primary_control_axis
      }
      else {
        this.primary_control_axis = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('secondary_control_axis')) {
        this.secondary_control_axis = initObj.secondary_control_axis
      }
      else {
        this.secondary_control_axis = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('system_state')) {
        this.system_state = initObj.system_state
      }
      else {
        this.system_state = new std_msgs.msg.Int8();
      }
      if (initObj.hasOwnProperty('robot_state')) {
        this.robot_state = initObj.robot_state
      }
      else {
        this.robot_state = new std_msgs.msg.Int8();
      }
      if (initObj.hasOwnProperty('gait_selection')) {
        this.gait_selection = initObj.gait_selection
      }
      else {
        this.gait_selection = new std_msgs.msg.Int8();
      }
      if (initObj.hasOwnProperty('cruise_control_mode')) {
        this.cruise_control_mode = initObj.cruise_control_mode
      }
      else {
        this.cruise_control_mode = new std_msgs.msg.Int8();
      }
      if (initObj.hasOwnProperty('auto_navigation_mode')) {
        this.auto_navigation_mode = initObj.auto_navigation_mode
      }
      else {
        this.auto_navigation_mode = new std_msgs.msg.Int8();
      }
      if (initObj.hasOwnProperty('posing_mode')) {
        this.posing_mode = initObj.posing_mode
      }
      else {
        this.posing_mode = new std_msgs.msg.Int8();
      }
      if (initObj.hasOwnProperty('pose_reset_mode')) {
        this.pose_reset_mode = initObj.pose_reset_mode
      }
      else {
        this.pose_reset_mode = new std_msgs.msg.Int8();
      }
      if (initObj.hasOwnProperty('primary_leg_selection')) {
        this.primary_leg_selection = initObj.primary_leg_selection
      }
      else {
        this.primary_leg_selection = new std_msgs.msg.Int8();
      }
      if (initObj.hasOwnProperty('secondary_leg_selection')) {
        this.secondary_leg_selection = initObj.secondary_leg_selection
      }
      else {
        this.secondary_leg_selection = new std_msgs.msg.Int8();
      }
      if (initObj.hasOwnProperty('primary_leg_state')) {
        this.primary_leg_state = initObj.primary_leg_state
      }
      else {
        this.primary_leg_state = new std_msgs.msg.Int8();
      }
      if (initObj.hasOwnProperty('secondary_leg_state')) {
        this.secondary_leg_state = initObj.secondary_leg_state
      }
      else {
        this.secondary_leg_state = new std_msgs.msg.Int8();
      }
      if (initObj.hasOwnProperty('parameter_selection')) {
        this.parameter_selection = initObj.parameter_selection
      }
      else {
        this.parameter_selection = new std_msgs.msg.Int8();
      }
      if (initObj.hasOwnProperty('parameter_adjustment')) {
        this.parameter_adjustment = initObj.parameter_adjustment
      }
      else {
        this.parameter_adjustment = new std_msgs.msg.Int8();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AndroidJoy
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [id_name]
    bufferOffset = std_msgs.msg.String.serialize(obj.id_name, buffer, bufferOffset);
    // Serialize message field [override_priority_interface]
    bufferOffset = std_msgs.msg.Bool.serialize(obj.override_priority_interface, buffer, bufferOffset);
    // Serialize message field [primary_control_axis]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.primary_control_axis, buffer, bufferOffset);
    // Serialize message field [secondary_control_axis]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.secondary_control_axis, buffer, bufferOffset);
    // Serialize message field [system_state]
    bufferOffset = std_msgs.msg.Int8.serialize(obj.system_state, buffer, bufferOffset);
    // Serialize message field [robot_state]
    bufferOffset = std_msgs.msg.Int8.serialize(obj.robot_state, buffer, bufferOffset);
    // Serialize message field [gait_selection]
    bufferOffset = std_msgs.msg.Int8.serialize(obj.gait_selection, buffer, bufferOffset);
    // Serialize message field [cruise_control_mode]
    bufferOffset = std_msgs.msg.Int8.serialize(obj.cruise_control_mode, buffer, bufferOffset);
    // Serialize message field [auto_navigation_mode]
    bufferOffset = std_msgs.msg.Int8.serialize(obj.auto_navigation_mode, buffer, bufferOffset);
    // Serialize message field [posing_mode]
    bufferOffset = std_msgs.msg.Int8.serialize(obj.posing_mode, buffer, bufferOffset);
    // Serialize message field [pose_reset_mode]
    bufferOffset = std_msgs.msg.Int8.serialize(obj.pose_reset_mode, buffer, bufferOffset);
    // Serialize message field [primary_leg_selection]
    bufferOffset = std_msgs.msg.Int8.serialize(obj.primary_leg_selection, buffer, bufferOffset);
    // Serialize message field [secondary_leg_selection]
    bufferOffset = std_msgs.msg.Int8.serialize(obj.secondary_leg_selection, buffer, bufferOffset);
    // Serialize message field [primary_leg_state]
    bufferOffset = std_msgs.msg.Int8.serialize(obj.primary_leg_state, buffer, bufferOffset);
    // Serialize message field [secondary_leg_state]
    bufferOffset = std_msgs.msg.Int8.serialize(obj.secondary_leg_state, buffer, bufferOffset);
    // Serialize message field [parameter_selection]
    bufferOffset = std_msgs.msg.Int8.serialize(obj.parameter_selection, buffer, bufferOffset);
    // Serialize message field [parameter_adjustment]
    bufferOffset = std_msgs.msg.Int8.serialize(obj.parameter_adjustment, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AndroidJoy
    let len;
    let data = new AndroidJoy(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [id_name]
    data.id_name = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    // Deserialize message field [override_priority_interface]
    data.override_priority_interface = std_msgs.msg.Bool.deserialize(buffer, bufferOffset);
    // Deserialize message field [primary_control_axis]
    data.primary_control_axis = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [secondary_control_axis]
    data.secondary_control_axis = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [system_state]
    data.system_state = std_msgs.msg.Int8.deserialize(buffer, bufferOffset);
    // Deserialize message field [robot_state]
    data.robot_state = std_msgs.msg.Int8.deserialize(buffer, bufferOffset);
    // Deserialize message field [gait_selection]
    data.gait_selection = std_msgs.msg.Int8.deserialize(buffer, bufferOffset);
    // Deserialize message field [cruise_control_mode]
    data.cruise_control_mode = std_msgs.msg.Int8.deserialize(buffer, bufferOffset);
    // Deserialize message field [auto_navigation_mode]
    data.auto_navigation_mode = std_msgs.msg.Int8.deserialize(buffer, bufferOffset);
    // Deserialize message field [posing_mode]
    data.posing_mode = std_msgs.msg.Int8.deserialize(buffer, bufferOffset);
    // Deserialize message field [pose_reset_mode]
    data.pose_reset_mode = std_msgs.msg.Int8.deserialize(buffer, bufferOffset);
    // Deserialize message field [primary_leg_selection]
    data.primary_leg_selection = std_msgs.msg.Int8.deserialize(buffer, bufferOffset);
    // Deserialize message field [secondary_leg_selection]
    data.secondary_leg_selection = std_msgs.msg.Int8.deserialize(buffer, bufferOffset);
    // Deserialize message field [primary_leg_state]
    data.primary_leg_state = std_msgs.msg.Int8.deserialize(buffer, bufferOffset);
    // Deserialize message field [secondary_leg_state]
    data.secondary_leg_state = std_msgs.msg.Int8.deserialize(buffer, bufferOffset);
    // Deserialize message field [parameter_selection]
    data.parameter_selection = std_msgs.msg.Int8.deserialize(buffer, bufferOffset);
    // Deserialize message field [parameter_adjustment]
    data.parameter_adjustment = std_msgs.msg.Int8.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += std_msgs.msg.String.getMessageSize(object.id_name);
    return length + 62;
  }

  static datatype() {
    // Returns string type for a message object
    return 'syropod_remote/AndroidJoy';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f798248626a520efb6e3973bbe95d25a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    std_msgs/String id_name
    std_msgs/Bool override_priority_interface
    geometry_msgs/Point primary_control_axis
    geometry_msgs/Point secondary_control_axis
    std_msgs/Int8 system_state
    std_msgs/Int8 robot_state
    std_msgs/Int8 gait_selection
    std_msgs/Int8 cruise_control_mode
    std_msgs/Int8 auto_navigation_mode
    std_msgs/Int8 posing_mode
    std_msgs/Int8 pose_reset_mode
    std_msgs/Int8 primary_leg_selection
    std_msgs/Int8 secondary_leg_selection
    std_msgs/Int8 primary_leg_state
    std_msgs/Int8 secondary_leg_state
    std_msgs/Int8 parameter_selection
    std_msgs/Int8 parameter_adjustment
    
    
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
    MSG: std_msgs/Int8
    int8 data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AndroidJoy(null);
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

    if (msg.primary_control_axis !== undefined) {
      resolved.primary_control_axis = geometry_msgs.msg.Point.Resolve(msg.primary_control_axis)
    }
    else {
      resolved.primary_control_axis = new geometry_msgs.msg.Point()
    }

    if (msg.secondary_control_axis !== undefined) {
      resolved.secondary_control_axis = geometry_msgs.msg.Point.Resolve(msg.secondary_control_axis)
    }
    else {
      resolved.secondary_control_axis = new geometry_msgs.msg.Point()
    }

    if (msg.system_state !== undefined) {
      resolved.system_state = std_msgs.msg.Int8.Resolve(msg.system_state)
    }
    else {
      resolved.system_state = new std_msgs.msg.Int8()
    }

    if (msg.robot_state !== undefined) {
      resolved.robot_state = std_msgs.msg.Int8.Resolve(msg.robot_state)
    }
    else {
      resolved.robot_state = new std_msgs.msg.Int8()
    }

    if (msg.gait_selection !== undefined) {
      resolved.gait_selection = std_msgs.msg.Int8.Resolve(msg.gait_selection)
    }
    else {
      resolved.gait_selection = new std_msgs.msg.Int8()
    }

    if (msg.cruise_control_mode !== undefined) {
      resolved.cruise_control_mode = std_msgs.msg.Int8.Resolve(msg.cruise_control_mode)
    }
    else {
      resolved.cruise_control_mode = new std_msgs.msg.Int8()
    }

    if (msg.auto_navigation_mode !== undefined) {
      resolved.auto_navigation_mode = std_msgs.msg.Int8.Resolve(msg.auto_navigation_mode)
    }
    else {
      resolved.auto_navigation_mode = new std_msgs.msg.Int8()
    }

    if (msg.posing_mode !== undefined) {
      resolved.posing_mode = std_msgs.msg.Int8.Resolve(msg.posing_mode)
    }
    else {
      resolved.posing_mode = new std_msgs.msg.Int8()
    }

    if (msg.pose_reset_mode !== undefined) {
      resolved.pose_reset_mode = std_msgs.msg.Int8.Resolve(msg.pose_reset_mode)
    }
    else {
      resolved.pose_reset_mode = new std_msgs.msg.Int8()
    }

    if (msg.primary_leg_selection !== undefined) {
      resolved.primary_leg_selection = std_msgs.msg.Int8.Resolve(msg.primary_leg_selection)
    }
    else {
      resolved.primary_leg_selection = new std_msgs.msg.Int8()
    }

    if (msg.secondary_leg_selection !== undefined) {
      resolved.secondary_leg_selection = std_msgs.msg.Int8.Resolve(msg.secondary_leg_selection)
    }
    else {
      resolved.secondary_leg_selection = new std_msgs.msg.Int8()
    }

    if (msg.primary_leg_state !== undefined) {
      resolved.primary_leg_state = std_msgs.msg.Int8.Resolve(msg.primary_leg_state)
    }
    else {
      resolved.primary_leg_state = new std_msgs.msg.Int8()
    }

    if (msg.secondary_leg_state !== undefined) {
      resolved.secondary_leg_state = std_msgs.msg.Int8.Resolve(msg.secondary_leg_state)
    }
    else {
      resolved.secondary_leg_state = new std_msgs.msg.Int8()
    }

    if (msg.parameter_selection !== undefined) {
      resolved.parameter_selection = std_msgs.msg.Int8.Resolve(msg.parameter_selection)
    }
    else {
      resolved.parameter_selection = new std_msgs.msg.Int8()
    }

    if (msg.parameter_adjustment !== undefined) {
      resolved.parameter_adjustment = std_msgs.msg.Int8.Resolve(msg.parameter_adjustment)
    }
    else {
      resolved.parameter_adjustment = new std_msgs.msg.Int8()
    }

    return resolved;
    }
};

module.exports = AndroidJoy;
