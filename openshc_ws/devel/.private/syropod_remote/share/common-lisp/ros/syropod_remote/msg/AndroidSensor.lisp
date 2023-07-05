; Auto-generated. Do not edit!


(cl:in-package syropod_remote-msg)


;//! \htmlinclude AndroidSensor.msg.html

(cl:defclass <AndroidSensor> (roslisp-msg-protocol:ros-message)
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
   (orientation
    :reader orientation
    :initarg :orientation
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (relative_compass
    :reader relative_compass
    :initarg :relative_compass
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (robot_state
    :reader robot_state
    :initarg :robot_state
    :type std_msgs-msg:Int8
    :initform (cl:make-instance 'std_msgs-msg:Int8))
   (posing_mode
    :reader posing_mode
    :initarg :posing_mode
    :type std_msgs-msg:Int8
    :initform (cl:make-instance 'std_msgs-msg:Int8))
   (control_axis
    :reader control_axis
    :initarg :control_axis
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass AndroidSensor (<AndroidSensor>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AndroidSensor>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AndroidSensor)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name syropod_remote-msg:<AndroidSensor> is deprecated: use syropod_remote-msg:AndroidSensor instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AndroidSensor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_remote-msg:header-val is deprecated.  Use syropod_remote-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'id_name-val :lambda-list '(m))
(cl:defmethod id_name-val ((m <AndroidSensor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_remote-msg:id_name-val is deprecated.  Use syropod_remote-msg:id_name instead.")
  (id_name m))

(cl:ensure-generic-function 'override_priority_interface-val :lambda-list '(m))
(cl:defmethod override_priority_interface-val ((m <AndroidSensor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_remote-msg:override_priority_interface-val is deprecated.  Use syropod_remote-msg:override_priority_interface instead.")
  (override_priority_interface m))

(cl:ensure-generic-function 'orientation-val :lambda-list '(m))
(cl:defmethod orientation-val ((m <AndroidSensor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_remote-msg:orientation-val is deprecated.  Use syropod_remote-msg:orientation instead.")
  (orientation m))

(cl:ensure-generic-function 'relative_compass-val :lambda-list '(m))
(cl:defmethod relative_compass-val ((m <AndroidSensor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_remote-msg:relative_compass-val is deprecated.  Use syropod_remote-msg:relative_compass instead.")
  (relative_compass m))

(cl:ensure-generic-function 'robot_state-val :lambda-list '(m))
(cl:defmethod robot_state-val ((m <AndroidSensor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_remote-msg:robot_state-val is deprecated.  Use syropod_remote-msg:robot_state instead.")
  (robot_state m))

(cl:ensure-generic-function 'posing_mode-val :lambda-list '(m))
(cl:defmethod posing_mode-val ((m <AndroidSensor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_remote-msg:posing_mode-val is deprecated.  Use syropod_remote-msg:posing_mode instead.")
  (posing_mode m))

(cl:ensure-generic-function 'control_axis-val :lambda-list '(m))
(cl:defmethod control_axis-val ((m <AndroidSensor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_remote-msg:control_axis-val is deprecated.  Use syropod_remote-msg:control_axis instead.")
  (control_axis m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AndroidSensor>) ostream)
  "Serializes a message object of type '<AndroidSensor>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'id_name) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'override_priority_interface) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'orientation) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'relative_compass) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robot_state) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'posing_mode) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'control_axis) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AndroidSensor>) istream)
  "Deserializes a message object of type '<AndroidSensor>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'id_name) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'override_priority_interface) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'orientation) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'relative_compass) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robot_state) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'posing_mode) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'control_axis) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AndroidSensor>)))
  "Returns string type for a message object of type '<AndroidSensor>"
  "syropod_remote/AndroidSensor")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AndroidSensor)))
  "Returns string type for a message object of type 'AndroidSensor"
  "syropod_remote/AndroidSensor")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AndroidSensor>)))
  "Returns md5sum for a message object of type '<AndroidSensor>"
  "05a22ff6b1072fe74d2077e0d442d058")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AndroidSensor)))
  "Returns md5sum for a message object of type 'AndroidSensor"
  "05a22ff6b1072fe74d2077e0d442d058")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AndroidSensor>)))
  "Returns full string definition for message of type '<AndroidSensor>"
  (cl:format cl:nil "Header header~%std_msgs/String id_name~%std_msgs/Bool override_priority_interface~%geometry_msgs/Point orientation~%std_msgs/Float64 relative_compass~%std_msgs/Int8 robot_state~%std_msgs/Int8 posing_mode~%geometry_msgs/Point control_axis~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/Int8~%int8 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AndroidSensor)))
  "Returns full string definition for message of type 'AndroidSensor"
  (cl:format cl:nil "Header header~%std_msgs/String id_name~%std_msgs/Bool override_priority_interface~%geometry_msgs/Point orientation~%std_msgs/Float64 relative_compass~%std_msgs/Int8 robot_state~%std_msgs/Int8 posing_mode~%geometry_msgs/Point control_axis~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/Int8~%int8 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AndroidSensor>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'id_name))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'override_priority_interface))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'orientation))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'relative_compass))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robot_state))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'posing_mode))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'control_axis))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AndroidSensor>))
  "Converts a ROS message object to a list"
  (cl:list 'AndroidSensor
    (cl:cons ':header (header msg))
    (cl:cons ':id_name (id_name msg))
    (cl:cons ':override_priority_interface (override_priority_interface msg))
    (cl:cons ':orientation (orientation msg))
    (cl:cons ':relative_compass (relative_compass msg))
    (cl:cons ':robot_state (robot_state msg))
    (cl:cons ':posing_mode (posing_mode msg))
    (cl:cons ':control_axis (control_axis msg))
))
