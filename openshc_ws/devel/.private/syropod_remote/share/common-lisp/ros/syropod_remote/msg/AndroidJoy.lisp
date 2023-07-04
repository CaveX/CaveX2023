; Auto-generated. Do not edit!


(cl:in-package syropod_remote-msg)


;//! \htmlinclude AndroidJoy.msg.html

(cl:defclass <AndroidJoy> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (id_name
    :reader id_name
    :initarg :id_name
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String))
   (override_priority_interface
    :reader override_priority_interface
    :initarg :override_priority_interface
    :type std_msgs-msg:Bool
    :initform (cl:make-instance 'std_msgs-msg:Bool))
   (primary_control_axis
    :reader primary_control_axis
    :initarg :primary_control_axis
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (secondary_control_axis
    :reader secondary_control_axis
    :initarg :secondary_control_axis
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (system_state
    :reader system_state
    :initarg :system_state
    :type std_msgs-msg:Int8
    :initform (cl:make-instance 'std_msgs-msg:Int8))
   (robot_state
    :reader robot_state
    :initarg :robot_state
    :type std_msgs-msg:Int8
    :initform (cl:make-instance 'std_msgs-msg:Int8))
   (gait_selection
    :reader gait_selection
    :initarg :gait_selection
    :type std_msgs-msg:Int8
    :initform (cl:make-instance 'std_msgs-msg:Int8))
   (cruise_control_mode
    :reader cruise_control_mode
    :initarg :cruise_control_mode
    :type std_msgs-msg:Int8
    :initform (cl:make-instance 'std_msgs-msg:Int8))
   (auto_navigation_mode
    :reader auto_navigation_mode
    :initarg :auto_navigation_mode
    :type std_msgs-msg:Int8
    :initform (cl:make-instance 'std_msgs-msg:Int8))
   (posing_mode
    :reader posing_mode
    :initarg :posing_mode
    :type std_msgs-msg:Int8
    :initform (cl:make-instance 'std_msgs-msg:Int8))
   (pose_reset_mode
    :reader pose_reset_mode
    :initarg :pose_reset_mode
    :type std_msgs-msg:Int8
    :initform (cl:make-instance 'std_msgs-msg:Int8))
   (primary_leg_selection
    :reader primary_leg_selection
    :initarg :primary_leg_selection
    :type std_msgs-msg:Int8
    :initform (cl:make-instance 'std_msgs-msg:Int8))
   (secondary_leg_selection
    :reader secondary_leg_selection
    :initarg :secondary_leg_selection
    :type std_msgs-msg:Int8
    :initform (cl:make-instance 'std_msgs-msg:Int8))
   (primary_leg_state
    :reader primary_leg_state
    :initarg :primary_leg_state
    :type std_msgs-msg:Int8
    :initform (cl:make-instance 'std_msgs-msg:Int8))
   (secondary_leg_state
    :reader secondary_leg_state
    :initarg :secondary_leg_state
    :type std_msgs-msg:Int8
    :initform (cl:make-instance 'std_msgs-msg:Int8))
   (parameter_selection
    :reader parameter_selection
    :initarg :parameter_selection
    :type std_msgs-msg:Int8
    :initform (cl:make-instance 'std_msgs-msg:Int8))
   (parameter_adjustment
    :reader parameter_adjustment
    :initarg :parameter_adjustment
    :type std_msgs-msg:Int8
    :initform (cl:make-instance 'std_msgs-msg:Int8)))
)

(cl:defclass AndroidJoy (<AndroidJoy>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AndroidJoy>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AndroidJoy)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name syropod_remote-msg:<AndroidJoy> is deprecated: use syropod_remote-msg:AndroidJoy instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AndroidJoy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_remote-msg:header-val is deprecated.  Use syropod_remote-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'id_name-val :lambda-list '(m))
(cl:defmethod id_name-val ((m <AndroidJoy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_remote-msg:id_name-val is deprecated.  Use syropod_remote-msg:id_name instead.")
  (id_name m))

(cl:ensure-generic-function 'override_priority_interface-val :lambda-list '(m))
(cl:defmethod override_priority_interface-val ((m <AndroidJoy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_remote-msg:override_priority_interface-val is deprecated.  Use syropod_remote-msg:override_priority_interface instead.")
  (override_priority_interface m))

(cl:ensure-generic-function 'primary_control_axis-val :lambda-list '(m))
(cl:defmethod primary_control_axis-val ((m <AndroidJoy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_remote-msg:primary_control_axis-val is deprecated.  Use syropod_remote-msg:primary_control_axis instead.")
  (primary_control_axis m))

(cl:ensure-generic-function 'secondary_control_axis-val :lambda-list '(m))
(cl:defmethod secondary_control_axis-val ((m <AndroidJoy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_remote-msg:secondary_control_axis-val is deprecated.  Use syropod_remote-msg:secondary_control_axis instead.")
  (secondary_control_axis m))

(cl:ensure-generic-function 'system_state-val :lambda-list '(m))
(cl:defmethod system_state-val ((m <AndroidJoy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_remote-msg:system_state-val is deprecated.  Use syropod_remote-msg:system_state instead.")
  (system_state m))

(cl:ensure-generic-function 'robot_state-val :lambda-list '(m))
(cl:defmethod robot_state-val ((m <AndroidJoy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_remote-msg:robot_state-val is deprecated.  Use syropod_remote-msg:robot_state instead.")
  (robot_state m))

(cl:ensure-generic-function 'gait_selection-val :lambda-list '(m))
(cl:defmethod gait_selection-val ((m <AndroidJoy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_remote-msg:gait_selection-val is deprecated.  Use syropod_remote-msg:gait_selection instead.")
  (gait_selection m))

(cl:ensure-generic-function 'cruise_control_mode-val :lambda-list '(m))
(cl:defmethod cruise_control_mode-val ((m <AndroidJoy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_remote-msg:cruise_control_mode-val is deprecated.  Use syropod_remote-msg:cruise_control_mode instead.")
  (cruise_control_mode m))

(cl:ensure-generic-function 'auto_navigation_mode-val :lambda-list '(m))
(cl:defmethod auto_navigation_mode-val ((m <AndroidJoy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_remote-msg:auto_navigation_mode-val is deprecated.  Use syropod_remote-msg:auto_navigation_mode instead.")
  (auto_navigation_mode m))

(cl:ensure-generic-function 'posing_mode-val :lambda-list '(m))
(cl:defmethod posing_mode-val ((m <AndroidJoy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_remote-msg:posing_mode-val is deprecated.  Use syropod_remote-msg:posing_mode instead.")
  (posing_mode m))

(cl:ensure-generic-function 'pose_reset_mode-val :lambda-list '(m))
(cl:defmethod pose_reset_mode-val ((m <AndroidJoy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_remote-msg:pose_reset_mode-val is deprecated.  Use syropod_remote-msg:pose_reset_mode instead.")
  (pose_reset_mode m))

(cl:ensure-generic-function 'primary_leg_selection-val :lambda-list '(m))
(cl:defmethod primary_leg_selection-val ((m <AndroidJoy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_remote-msg:primary_leg_selection-val is deprecated.  Use syropod_remote-msg:primary_leg_selection instead.")
  (primary_leg_selection m))

(cl:ensure-generic-function 'secondary_leg_selection-val :lambda-list '(m))
(cl:defmethod secondary_leg_selection-val ((m <AndroidJoy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_remote-msg:secondary_leg_selection-val is deprecated.  Use syropod_remote-msg:secondary_leg_selection instead.")
  (secondary_leg_selection m))

(cl:ensure-generic-function 'primary_leg_state-val :lambda-list '(m))
(cl:defmethod primary_leg_state-val ((m <AndroidJoy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_remote-msg:primary_leg_state-val is deprecated.  Use syropod_remote-msg:primary_leg_state instead.")
  (primary_leg_state m))

(cl:ensure-generic-function 'secondary_leg_state-val :lambda-list '(m))
(cl:defmethod secondary_leg_state-val ((m <AndroidJoy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_remote-msg:secondary_leg_state-val is deprecated.  Use syropod_remote-msg:secondary_leg_state instead.")
  (secondary_leg_state m))

(cl:ensure-generic-function 'parameter_selection-val :lambda-list '(m))
(cl:defmethod parameter_selection-val ((m <AndroidJoy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_remote-msg:parameter_selection-val is deprecated.  Use syropod_remote-msg:parameter_selection instead.")
  (parameter_selection m))

(cl:ensure-generic-function 'parameter_adjustment-val :lambda-list '(m))
(cl:defmethod parameter_adjustment-val ((m <AndroidJoy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_remote-msg:parameter_adjustment-val is deprecated.  Use syropod_remote-msg:parameter_adjustment instead.")
  (parameter_adjustment m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AndroidJoy>) ostream)
  "Serializes a message object of type '<AndroidJoy>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'id_name) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'override_priority_interface) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'primary_control_axis) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'secondary_control_axis) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'system_state) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robot_state) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'gait_selection) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cruise_control_mode) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'auto_navigation_mode) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'posing_mode) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose_reset_mode) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'primary_leg_selection) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'secondary_leg_selection) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'primary_leg_state) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'secondary_leg_state) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'parameter_selection) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'parameter_adjustment) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AndroidJoy>) istream)
  "Deserializes a message object of type '<AndroidJoy>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'id_name) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'override_priority_interface) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'primary_control_axis) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'secondary_control_axis) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'system_state) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robot_state) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'gait_selection) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cruise_control_mode) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'auto_navigation_mode) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'posing_mode) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose_reset_mode) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'primary_leg_selection) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'secondary_leg_selection) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'primary_leg_state) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'secondary_leg_state) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'parameter_selection) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'parameter_adjustment) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AndroidJoy>)))
  "Returns string type for a message object of type '<AndroidJoy>"
  "syropod_remote/AndroidJoy")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AndroidJoy)))
  "Returns string type for a message object of type 'AndroidJoy"
  "syropod_remote/AndroidJoy")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AndroidJoy>)))
  "Returns md5sum for a message object of type '<AndroidJoy>"
  "f798248626a520efb6e3973bbe95d25a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AndroidJoy)))
  "Returns md5sum for a message object of type 'AndroidJoy"
  "f798248626a520efb6e3973bbe95d25a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AndroidJoy>)))
  "Returns full string definition for message of type '<AndroidJoy>"
  (cl:format cl:nil "Header header~%std_msgs/String id_name~%std_msgs/Bool override_priority_interface~%geometry_msgs/Point primary_control_axis~%geometry_msgs/Point secondary_control_axis~%std_msgs/Int8 system_state~%std_msgs/Int8 robot_state~%std_msgs/Int8 gait_selection~%std_msgs/Int8 cruise_control_mode~%std_msgs/Int8 auto_navigation_mode~%std_msgs/Int8 posing_mode~%std_msgs/Int8 pose_reset_mode~%std_msgs/Int8 primary_leg_selection~%std_msgs/Int8 secondary_leg_selection~%std_msgs/Int8 primary_leg_state~%std_msgs/Int8 secondary_leg_state~%std_msgs/Int8 parameter_selection~%std_msgs/Int8 parameter_adjustment~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/Int8~%int8 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AndroidJoy)))
  "Returns full string definition for message of type 'AndroidJoy"
  (cl:format cl:nil "Header header~%std_msgs/String id_name~%std_msgs/Bool override_priority_interface~%geometry_msgs/Point primary_control_axis~%geometry_msgs/Point secondary_control_axis~%std_msgs/Int8 system_state~%std_msgs/Int8 robot_state~%std_msgs/Int8 gait_selection~%std_msgs/Int8 cruise_control_mode~%std_msgs/Int8 auto_navigation_mode~%std_msgs/Int8 posing_mode~%std_msgs/Int8 pose_reset_mode~%std_msgs/Int8 primary_leg_selection~%std_msgs/Int8 secondary_leg_selection~%std_msgs/Int8 primary_leg_state~%std_msgs/Int8 secondary_leg_state~%std_msgs/Int8 parameter_selection~%std_msgs/Int8 parameter_adjustment~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/Int8~%int8 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AndroidJoy>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'id_name))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'override_priority_interface))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'primary_control_axis))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'secondary_control_axis))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'system_state))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robot_state))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'gait_selection))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cruise_control_mode))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'auto_navigation_mode))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'posing_mode))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose_reset_mode))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'primary_leg_selection))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'secondary_leg_selection))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'primary_leg_state))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'secondary_leg_state))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'parameter_selection))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'parameter_adjustment))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AndroidJoy>))
  "Converts a ROS message object to a list"
  (cl:list 'AndroidJoy
    (cl:cons ':header (header msg))
    (cl:cons ':id_name (id_name msg))
    (cl:cons ':override_priority_interface (override_priority_interface msg))
    (cl:cons ':primary_control_axis (primary_control_axis msg))
    (cl:cons ':secondary_control_axis (secondary_control_axis msg))
    (cl:cons ':system_state (system_state msg))
    (cl:cons ':robot_state (robot_state msg))
    (cl:cons ':gait_selection (gait_selection msg))
    (cl:cons ':cruise_control_mode (cruise_control_mode msg))
    (cl:cons ':auto_navigation_mode (auto_navigation_mode msg))
    (cl:cons ':posing_mode (posing_mode msg))
    (cl:cons ':pose_reset_mode (pose_reset_mode msg))
    (cl:cons ':primary_leg_selection (primary_leg_selection msg))
    (cl:cons ':secondary_leg_selection (secondary_leg_selection msg))
    (cl:cons ':primary_leg_state (primary_leg_state msg))
    (cl:cons ':secondary_leg_state (secondary_leg_state msg))
    (cl:cons ':parameter_selection (parameter_selection msg))
    (cl:cons ':parameter_adjustment (parameter_adjustment msg))
))
