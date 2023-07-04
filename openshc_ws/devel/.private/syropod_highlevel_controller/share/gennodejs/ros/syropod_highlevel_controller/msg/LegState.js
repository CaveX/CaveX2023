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

class LegState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.name = null;
      this.walker_tip_pose = null;
      this.target_tip_pose = null;
      this.poser_tip_pose = null;
      this.model_tip_pose = null;
      this.actual_tip_pose = null;
      this.model_tip_velocity = null;
      this.joint_positions = null;
      this.joint_velocities = null;
      this.joint_efforts = null;
      this.stance_progress = null;
      this.swing_progress = null;
      this.time_to_swing_end = null;
      this.pose_delta = null;
      this.auto_pose = null;
      this.tip_force = null;
      this.admittance_delta = null;
      this.virtual_stiffness = null;
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
        this.name = '';
      }
      if (initObj.hasOwnProperty('walker_tip_pose')) {
        this.walker_tip_pose = initObj.walker_tip_pose
      }
      else {
        this.walker_tip_pose = new geometry_msgs.msg.PoseStamped();
      }
      if (initObj.hasOwnProperty('target_tip_pose')) {
        this.target_tip_pose = initObj.target_tip_pose
      }
      else {
        this.target_tip_pose = new geometry_msgs.msg.PoseStamped();
      }
      if (initObj.hasOwnProperty('poser_tip_pose')) {
        this.poser_tip_pose = initObj.poser_tip_pose
      }
      else {
        this.poser_tip_pose = new geometry_msgs.msg.PoseStamped();
      }
      if (initObj.hasOwnProperty('model_tip_pose')) {
        this.model_tip_pose = initObj.model_tip_pose
      }
      else {
        this.model_tip_pose = new geometry_msgs.msg.PoseStamped();
      }
      if (initObj.hasOwnProperty('actual_tip_pose')) {
        this.actual_tip_pose = initObj.actual_tip_pose
      }
      else {
        this.actual_tip_pose = new geometry_msgs.msg.PoseStamped();
      }
      if (initObj.hasOwnProperty('model_tip_velocity')) {
        this.model_tip_velocity = initObj.model_tip_velocity
      }
      else {
        this.model_tip_velocity = new geometry_msgs.msg.TwistStamped();
      }
      if (initObj.hasOwnProperty('joint_positions')) {
        this.joint_positions = initObj.joint_positions
      }
      else {
        this.joint_positions = [];
      }
      if (initObj.hasOwnProperty('joint_velocities')) {
        this.joint_velocities = initObj.joint_velocities
      }
      else {
        this.joint_velocities = [];
      }
      if (initObj.hasOwnProperty('joint_efforts')) {
        this.joint_efforts = initObj.joint_efforts
      }
      else {
        this.joint_efforts = [];
      }
      if (initObj.hasOwnProperty('stance_progress')) {
        this.stance_progress = initObj.stance_progress
      }
      else {
        this.stance_progress = 0.0;
      }
      if (initObj.hasOwnProperty('swing_progress')) {
        this.swing_progress = initObj.swing_progress
      }
      else {
        this.swing_progress = 0.0;
      }
      if (initObj.hasOwnProperty('time_to_swing_end')) {
        this.time_to_swing_end = initObj.time_to_swing_end
      }
      else {
        this.time_to_swing_end = 0.0;
      }
      if (initObj.hasOwnProperty('pose_delta')) {
        this.pose_delta = initObj.pose_delta
      }
      else {
        this.pose_delta = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('auto_pose')) {
        this.auto_pose = initObj.auto_pose
      }
      else {
        this.auto_pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('tip_force')) {
        this.tip_force = initObj.tip_force
      }
      else {
        this.tip_force = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('admittance_delta')) {
        this.admittance_delta = initObj.admittance_delta
      }
      else {
        this.admittance_delta = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('virtual_stiffness')) {
        this.virtual_stiffness = initObj.virtual_stiffness
      }
      else {
        this.virtual_stiffness = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LegState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [walker_tip_pose]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.walker_tip_pose, buffer, bufferOffset);
    // Serialize message field [target_tip_pose]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.target_tip_pose, buffer, bufferOffset);
    // Serialize message field [poser_tip_pose]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.poser_tip_pose, buffer, bufferOffset);
    // Serialize message field [model_tip_pose]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.model_tip_pose, buffer, bufferOffset);
    // Serialize message field [actual_tip_pose]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.actual_tip_pose, buffer, bufferOffset);
    // Serialize message field [model_tip_velocity]
    bufferOffset = geometry_msgs.msg.TwistStamped.serialize(obj.model_tip_velocity, buffer, bufferOffset);
    // Serialize message field [joint_positions]
    bufferOffset = _arraySerializer.float64(obj.joint_positions, buffer, bufferOffset, null);
    // Serialize message field [joint_velocities]
    bufferOffset = _arraySerializer.float64(obj.joint_velocities, buffer, bufferOffset, null);
    // Serialize message field [joint_efforts]
    bufferOffset = _arraySerializer.float64(obj.joint_efforts, buffer, bufferOffset, null);
    // Serialize message field [stance_progress]
    bufferOffset = _serializer.float64(obj.stance_progress, buffer, bufferOffset);
    // Serialize message field [swing_progress]
    bufferOffset = _serializer.float64(obj.swing_progress, buffer, bufferOffset);
    // Serialize message field [time_to_swing_end]
    bufferOffset = _serializer.float64(obj.time_to_swing_end, buffer, bufferOffset);
    // Serialize message field [pose_delta]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.pose_delta, buffer, bufferOffset);
    // Serialize message field [auto_pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.auto_pose, buffer, bufferOffset);
    // Serialize message field [tip_force]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.tip_force, buffer, bufferOffset);
    // Serialize message field [admittance_delta]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.admittance_delta, buffer, bufferOffset);
    // Serialize message field [virtual_stiffness]
    bufferOffset = _serializer.float64(obj.virtual_stiffness, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LegState
    let len;
    let data = new LegState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [walker_tip_pose]
    data.walker_tip_pose = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [target_tip_pose]
    data.target_tip_pose = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [poser_tip_pose]
    data.poser_tip_pose = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [model_tip_pose]
    data.model_tip_pose = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [actual_tip_pose]
    data.actual_tip_pose = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [model_tip_velocity]
    data.model_tip_velocity = geometry_msgs.msg.TwistStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [joint_positions]
    data.joint_positions = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [joint_velocities]
    data.joint_velocities = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [joint_efforts]
    data.joint_efforts = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [stance_progress]
    data.stance_progress = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [swing_progress]
    data.swing_progress = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [time_to_swing_end]
    data.time_to_swing_end = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pose_delta]
    data.pose_delta = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [auto_pose]
    data.auto_pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [tip_force]
    data.tip_force = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [admittance_delta]
    data.admittance_delta = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [virtual_stiffness]
    data.virtual_stiffness = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.name.length;
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.walker_tip_pose);
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.target_tip_pose);
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.poser_tip_pose);
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.model_tip_pose);
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.actual_tip_pose);
    length += geometry_msgs.msg.TwistStamped.getMessageSize(object.model_tip_velocity);
    length += 8 * object.joint_positions.length;
    length += 8 * object.joint_velocities.length;
    length += 8 * object.joint_efforts.length;
    return length + 208;
  }

  static datatype() {
    // Returns string type for a message object
    return 'syropod_highlevel_controller/LegState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dadda7ca412e345da1ddcca95ddf0ccc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    string name
    
    geometry_msgs/PoseStamped walker_tip_pose
    geometry_msgs/PoseStamped target_tip_pose
    geometry_msgs/PoseStamped poser_tip_pose
    geometry_msgs/PoseStamped model_tip_pose
    geometry_msgs/PoseStamped actual_tip_pose
    
    geometry_msgs/TwistStamped model_tip_velocity
    
    float64[] joint_positions
    float64[] joint_velocities
    float64[] joint_efforts
    
    float64 stance_progress
    float64 swing_progress
    
    float64 time_to_swing_end
    geometry_msgs/Pose pose_delta
    
    geometry_msgs/Pose auto_pose
    
    geometry_msgs/Vector3 tip_force
    geometry_msgs/Vector3 admittance_delta
    float64 virtual_stiffness
    
    
    
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
    
    ================================================================================
    MSG: geometry_msgs/TwistStamped
    # A twist with reference coordinate frame and timestamp
    Header header
    Twist twist
    
    ================================================================================
    MSG: geometry_msgs/Twist
    # This expresses velocity in free space broken into its linear and angular parts.
    Vector3  linear
    Vector3  angular
    
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
    const resolved = new LegState(null);
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
      resolved.name = ''
    }

    if (msg.walker_tip_pose !== undefined) {
      resolved.walker_tip_pose = geometry_msgs.msg.PoseStamped.Resolve(msg.walker_tip_pose)
    }
    else {
      resolved.walker_tip_pose = new geometry_msgs.msg.PoseStamped()
    }

    if (msg.target_tip_pose !== undefined) {
      resolved.target_tip_pose = geometry_msgs.msg.PoseStamped.Resolve(msg.target_tip_pose)
    }
    else {
      resolved.target_tip_pose = new geometry_msgs.msg.PoseStamped()
    }

    if (msg.poser_tip_pose !== undefined) {
      resolved.poser_tip_pose = geometry_msgs.msg.PoseStamped.Resolve(msg.poser_tip_pose)
    }
    else {
      resolved.poser_tip_pose = new geometry_msgs.msg.PoseStamped()
    }

    if (msg.model_tip_pose !== undefined) {
      resolved.model_tip_pose = geometry_msgs.msg.PoseStamped.Resolve(msg.model_tip_pose)
    }
    else {
      resolved.model_tip_pose = new geometry_msgs.msg.PoseStamped()
    }

    if (msg.actual_tip_pose !== undefined) {
      resolved.actual_tip_pose = geometry_msgs.msg.PoseStamped.Resolve(msg.actual_tip_pose)
    }
    else {
      resolved.actual_tip_pose = new geometry_msgs.msg.PoseStamped()
    }

    if (msg.model_tip_velocity !== undefined) {
      resolved.model_tip_velocity = geometry_msgs.msg.TwistStamped.Resolve(msg.model_tip_velocity)
    }
    else {
      resolved.model_tip_velocity = new geometry_msgs.msg.TwistStamped()
    }

    if (msg.joint_positions !== undefined) {
      resolved.joint_positions = msg.joint_positions;
    }
    else {
      resolved.joint_positions = []
    }

    if (msg.joint_velocities !== undefined) {
      resolved.joint_velocities = msg.joint_velocities;
    }
    else {
      resolved.joint_velocities = []
    }

    if (msg.joint_efforts !== undefined) {
      resolved.joint_efforts = msg.joint_efforts;
    }
    else {
      resolved.joint_efforts = []
    }

    if (msg.stance_progress !== undefined) {
      resolved.stance_progress = msg.stance_progress;
    }
    else {
      resolved.stance_progress = 0.0
    }

    if (msg.swing_progress !== undefined) {
      resolved.swing_progress = msg.swing_progress;
    }
    else {
      resolved.swing_progress = 0.0
    }

    if (msg.time_to_swing_end !== undefined) {
      resolved.time_to_swing_end = msg.time_to_swing_end;
    }
    else {
      resolved.time_to_swing_end = 0.0
    }

    if (msg.pose_delta !== undefined) {
      resolved.pose_delta = geometry_msgs.msg.Pose.Resolve(msg.pose_delta)
    }
    else {
      resolved.pose_delta = new geometry_msgs.msg.Pose()
    }

    if (msg.auto_pose !== undefined) {
      resolved.auto_pose = geometry_msgs.msg.Pose.Resolve(msg.auto_pose)
    }
    else {
      resolved.auto_pose = new geometry_msgs.msg.Pose()
    }

    if (msg.tip_force !== undefined) {
      resolved.tip_force = geometry_msgs.msg.Vector3.Resolve(msg.tip_force)
    }
    else {
      resolved.tip_force = new geometry_msgs.msg.Vector3()
    }

    if (msg.admittance_delta !== undefined) {
      resolved.admittance_delta = geometry_msgs.msg.Vector3.Resolve(msg.admittance_delta)
    }
    else {
      resolved.admittance_delta = new geometry_msgs.msg.Vector3()
    }

    if (msg.virtual_stiffness !== undefined) {
      resolved.virtual_stiffness = msg.virtual_stiffness;
    }
    else {
      resolved.virtual_stiffness = 0.0
    }

    return resolved;
    }
};

module.exports = LegState;
