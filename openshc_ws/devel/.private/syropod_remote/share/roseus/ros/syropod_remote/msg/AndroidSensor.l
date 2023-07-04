;; Auto-generated. Do not edit!


(when (boundp 'syropod_remote::AndroidSensor)
  (if (not (find-package "SYROPOD_REMOTE"))
    (make-package "SYROPOD_REMOTE"))
  (shadow 'AndroidSensor (find-package "SYROPOD_REMOTE")))
(unless (find-package "SYROPOD_REMOTE::ANDROIDSENSOR")
  (make-package "SYROPOD_REMOTE::ANDROIDSENSOR"))

(in-package "ROS")
;;//! \htmlinclude AndroidSensor.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass syropod_remote::AndroidSensor
  :super ros::object
  :slots (_header _id_name _override_priority_interface _orientation _relative_compass _robot_state _posing_mode _control_axis ))

(defmethod syropod_remote::AndroidSensor
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:id_name __id_name) (instance std_msgs::String :init))
    ((:override_priority_interface __override_priority_interface) (instance std_msgs::Bool :init))
    ((:orientation __orientation) (instance geometry_msgs::Point :init))
    ((:relative_compass __relative_compass) (instance std_msgs::Float64 :init))
    ((:robot_state __robot_state) (instance std_msgs::Int8 :init))
    ((:posing_mode __posing_mode) (instance std_msgs::Int8 :init))
    ((:control_axis __control_axis) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _id_name __id_name)
   (setq _override_priority_interface __override_priority_interface)
   (setq _orientation __orientation)
   (setq _relative_compass __relative_compass)
   (setq _robot_state __robot_state)
   (setq _posing_mode __posing_mode)
   (setq _control_axis __control_axis)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:id_name
   (&rest __id_name)
   (if (keywordp (car __id_name))
       (send* _id_name __id_name)
     (progn
       (if __id_name (setq _id_name (car __id_name)))
       _id_name)))
  (:override_priority_interface
   (&rest __override_priority_interface)
   (if (keywordp (car __override_priority_interface))
       (send* _override_priority_interface __override_priority_interface)
     (progn
       (if __override_priority_interface (setq _override_priority_interface (car __override_priority_interface)))
       _override_priority_interface)))
  (:orientation
   (&rest __orientation)
   (if (keywordp (car __orientation))
       (send* _orientation __orientation)
     (progn
       (if __orientation (setq _orientation (car __orientation)))
       _orientation)))
  (:relative_compass
   (&rest __relative_compass)
   (if (keywordp (car __relative_compass))
       (send* _relative_compass __relative_compass)
     (progn
       (if __relative_compass (setq _relative_compass (car __relative_compass)))
       _relative_compass)))
  (:robot_state
   (&rest __robot_state)
   (if (keywordp (car __robot_state))
       (send* _robot_state __robot_state)
     (progn
       (if __robot_state (setq _robot_state (car __robot_state)))
       _robot_state)))
  (:posing_mode
   (&rest __posing_mode)
   (if (keywordp (car __posing_mode))
       (send* _posing_mode __posing_mode)
     (progn
       (if __posing_mode (setq _posing_mode (car __posing_mode)))
       _posing_mode)))
  (:control_axis
   (&rest __control_axis)
   (if (keywordp (car __control_axis))
       (send* _control_axis __control_axis)
     (progn
       (if __control_axis (setq _control_axis (car __control_axis)))
       _control_axis)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; std_msgs/String _id_name
    (send _id_name :serialization-length)
    ;; std_msgs/Bool _override_priority_interface
    (send _override_priority_interface :serialization-length)
    ;; geometry_msgs/Point _orientation
    (send _orientation :serialization-length)
    ;; std_msgs/Float64 _relative_compass
    (send _relative_compass :serialization-length)
    ;; std_msgs/Int8 _robot_state
    (send _robot_state :serialization-length)
    ;; std_msgs/Int8 _posing_mode
    (send _posing_mode :serialization-length)
    ;; geometry_msgs/Point _control_axis
    (send _control_axis :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; std_msgs/String _id_name
       (send _id_name :serialize s)
     ;; std_msgs/Bool _override_priority_interface
       (send _override_priority_interface :serialize s)
     ;; geometry_msgs/Point _orientation
       (send _orientation :serialize s)
     ;; std_msgs/Float64 _relative_compass
       (send _relative_compass :serialize s)
     ;; std_msgs/Int8 _robot_state
       (send _robot_state :serialize s)
     ;; std_msgs/Int8 _posing_mode
       (send _posing_mode :serialize s)
     ;; geometry_msgs/Point _control_axis
       (send _control_axis :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; std_msgs/String _id_name
     (send _id_name :deserialize buf ptr-) (incf ptr- (send _id_name :serialization-length))
   ;; std_msgs/Bool _override_priority_interface
     (send _override_priority_interface :deserialize buf ptr-) (incf ptr- (send _override_priority_interface :serialization-length))
   ;; geometry_msgs/Point _orientation
     (send _orientation :deserialize buf ptr-) (incf ptr- (send _orientation :serialization-length))
   ;; std_msgs/Float64 _relative_compass
     (send _relative_compass :deserialize buf ptr-) (incf ptr- (send _relative_compass :serialization-length))
   ;; std_msgs/Int8 _robot_state
     (send _robot_state :deserialize buf ptr-) (incf ptr- (send _robot_state :serialization-length))
   ;; std_msgs/Int8 _posing_mode
     (send _posing_mode :deserialize buf ptr-) (incf ptr- (send _posing_mode :serialization-length))
   ;; geometry_msgs/Point _control_axis
     (send _control_axis :deserialize buf ptr-) (incf ptr- (send _control_axis :serialization-length))
   ;;
   self)
  )

(setf (get syropod_remote::AndroidSensor :md5sum-) "05a22ff6b1072fe74d2077e0d442d058")
(setf (get syropod_remote::AndroidSensor :datatype-) "syropod_remote/AndroidSensor")
(setf (get syropod_remote::AndroidSensor :definition-)
      "Header header
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

")



(provide :syropod_remote/AndroidSensor "05a22ff6b1072fe74d2077e0d442d058")


