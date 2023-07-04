;; Auto-generated. Do not edit!


(when (boundp 'syropod_remote::AndroidJoy)
  (if (not (find-package "SYROPOD_REMOTE"))
    (make-package "SYROPOD_REMOTE"))
  (shadow 'AndroidJoy (find-package "SYROPOD_REMOTE")))
(unless (find-package "SYROPOD_REMOTE::ANDROIDJOY")
  (make-package "SYROPOD_REMOTE::ANDROIDJOY"))

(in-package "ROS")
;;//! \htmlinclude AndroidJoy.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass syropod_remote::AndroidJoy
  :super ros::object
  :slots (_header _id_name _override_priority_interface _primary_control_axis _secondary_control_axis _system_state _robot_state _gait_selection _cruise_control_mode _auto_navigation_mode _posing_mode _pose_reset_mode _primary_leg_selection _secondary_leg_selection _primary_leg_state _secondary_leg_state _parameter_selection _parameter_adjustment ))

(defmethod syropod_remote::AndroidJoy
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:id_name __id_name) (instance std_msgs::String :init))
    ((:override_priority_interface __override_priority_interface) (instance std_msgs::Bool :init))
    ((:primary_control_axis __primary_control_axis) (instance geometry_msgs::Point :init))
    ((:secondary_control_axis __secondary_control_axis) (instance geometry_msgs::Point :init))
    ((:system_state __system_state) (instance std_msgs::Int8 :init))
    ((:robot_state __robot_state) (instance std_msgs::Int8 :init))
    ((:gait_selection __gait_selection) (instance std_msgs::Int8 :init))
    ((:cruise_control_mode __cruise_control_mode) (instance std_msgs::Int8 :init))
    ((:auto_navigation_mode __auto_navigation_mode) (instance std_msgs::Int8 :init))
    ((:posing_mode __posing_mode) (instance std_msgs::Int8 :init))
    ((:pose_reset_mode __pose_reset_mode) (instance std_msgs::Int8 :init))
    ((:primary_leg_selection __primary_leg_selection) (instance std_msgs::Int8 :init))
    ((:secondary_leg_selection __secondary_leg_selection) (instance std_msgs::Int8 :init))
    ((:primary_leg_state __primary_leg_state) (instance std_msgs::Int8 :init))
    ((:secondary_leg_state __secondary_leg_state) (instance std_msgs::Int8 :init))
    ((:parameter_selection __parameter_selection) (instance std_msgs::Int8 :init))
    ((:parameter_adjustment __parameter_adjustment) (instance std_msgs::Int8 :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _id_name __id_name)
   (setq _override_priority_interface __override_priority_interface)
   (setq _primary_control_axis __primary_control_axis)
   (setq _secondary_control_axis __secondary_control_axis)
   (setq _system_state __system_state)
   (setq _robot_state __robot_state)
   (setq _gait_selection __gait_selection)
   (setq _cruise_control_mode __cruise_control_mode)
   (setq _auto_navigation_mode __auto_navigation_mode)
   (setq _posing_mode __posing_mode)
   (setq _pose_reset_mode __pose_reset_mode)
   (setq _primary_leg_selection __primary_leg_selection)
   (setq _secondary_leg_selection __secondary_leg_selection)
   (setq _primary_leg_state __primary_leg_state)
   (setq _secondary_leg_state __secondary_leg_state)
   (setq _parameter_selection __parameter_selection)
   (setq _parameter_adjustment __parameter_adjustment)
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
  (:primary_control_axis
   (&rest __primary_control_axis)
   (if (keywordp (car __primary_control_axis))
       (send* _primary_control_axis __primary_control_axis)
     (progn
       (if __primary_control_axis (setq _primary_control_axis (car __primary_control_axis)))
       _primary_control_axis)))
  (:secondary_control_axis
   (&rest __secondary_control_axis)
   (if (keywordp (car __secondary_control_axis))
       (send* _secondary_control_axis __secondary_control_axis)
     (progn
       (if __secondary_control_axis (setq _secondary_control_axis (car __secondary_control_axis)))
       _secondary_control_axis)))
  (:system_state
   (&rest __system_state)
   (if (keywordp (car __system_state))
       (send* _system_state __system_state)
     (progn
       (if __system_state (setq _system_state (car __system_state)))
       _system_state)))
  (:robot_state
   (&rest __robot_state)
   (if (keywordp (car __robot_state))
       (send* _robot_state __robot_state)
     (progn
       (if __robot_state (setq _robot_state (car __robot_state)))
       _robot_state)))
  (:gait_selection
   (&rest __gait_selection)
   (if (keywordp (car __gait_selection))
       (send* _gait_selection __gait_selection)
     (progn
       (if __gait_selection (setq _gait_selection (car __gait_selection)))
       _gait_selection)))
  (:cruise_control_mode
   (&rest __cruise_control_mode)
   (if (keywordp (car __cruise_control_mode))
       (send* _cruise_control_mode __cruise_control_mode)
     (progn
       (if __cruise_control_mode (setq _cruise_control_mode (car __cruise_control_mode)))
       _cruise_control_mode)))
  (:auto_navigation_mode
   (&rest __auto_navigation_mode)
   (if (keywordp (car __auto_navigation_mode))
       (send* _auto_navigation_mode __auto_navigation_mode)
     (progn
       (if __auto_navigation_mode (setq _auto_navigation_mode (car __auto_navigation_mode)))
       _auto_navigation_mode)))
  (:posing_mode
   (&rest __posing_mode)
   (if (keywordp (car __posing_mode))
       (send* _posing_mode __posing_mode)
     (progn
       (if __posing_mode (setq _posing_mode (car __posing_mode)))
       _posing_mode)))
  (:pose_reset_mode
   (&rest __pose_reset_mode)
   (if (keywordp (car __pose_reset_mode))
       (send* _pose_reset_mode __pose_reset_mode)
     (progn
       (if __pose_reset_mode (setq _pose_reset_mode (car __pose_reset_mode)))
       _pose_reset_mode)))
  (:primary_leg_selection
   (&rest __primary_leg_selection)
   (if (keywordp (car __primary_leg_selection))
       (send* _primary_leg_selection __primary_leg_selection)
     (progn
       (if __primary_leg_selection (setq _primary_leg_selection (car __primary_leg_selection)))
       _primary_leg_selection)))
  (:secondary_leg_selection
   (&rest __secondary_leg_selection)
   (if (keywordp (car __secondary_leg_selection))
       (send* _secondary_leg_selection __secondary_leg_selection)
     (progn
       (if __secondary_leg_selection (setq _secondary_leg_selection (car __secondary_leg_selection)))
       _secondary_leg_selection)))
  (:primary_leg_state
   (&rest __primary_leg_state)
   (if (keywordp (car __primary_leg_state))
       (send* _primary_leg_state __primary_leg_state)
     (progn
       (if __primary_leg_state (setq _primary_leg_state (car __primary_leg_state)))
       _primary_leg_state)))
  (:secondary_leg_state
   (&rest __secondary_leg_state)
   (if (keywordp (car __secondary_leg_state))
       (send* _secondary_leg_state __secondary_leg_state)
     (progn
       (if __secondary_leg_state (setq _secondary_leg_state (car __secondary_leg_state)))
       _secondary_leg_state)))
  (:parameter_selection
   (&rest __parameter_selection)
   (if (keywordp (car __parameter_selection))
       (send* _parameter_selection __parameter_selection)
     (progn
       (if __parameter_selection (setq _parameter_selection (car __parameter_selection)))
       _parameter_selection)))
  (:parameter_adjustment
   (&rest __parameter_adjustment)
   (if (keywordp (car __parameter_adjustment))
       (send* _parameter_adjustment __parameter_adjustment)
     (progn
       (if __parameter_adjustment (setq _parameter_adjustment (car __parameter_adjustment)))
       _parameter_adjustment)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; std_msgs/String _id_name
    (send _id_name :serialization-length)
    ;; std_msgs/Bool _override_priority_interface
    (send _override_priority_interface :serialization-length)
    ;; geometry_msgs/Point _primary_control_axis
    (send _primary_control_axis :serialization-length)
    ;; geometry_msgs/Point _secondary_control_axis
    (send _secondary_control_axis :serialization-length)
    ;; std_msgs/Int8 _system_state
    (send _system_state :serialization-length)
    ;; std_msgs/Int8 _robot_state
    (send _robot_state :serialization-length)
    ;; std_msgs/Int8 _gait_selection
    (send _gait_selection :serialization-length)
    ;; std_msgs/Int8 _cruise_control_mode
    (send _cruise_control_mode :serialization-length)
    ;; std_msgs/Int8 _auto_navigation_mode
    (send _auto_navigation_mode :serialization-length)
    ;; std_msgs/Int8 _posing_mode
    (send _posing_mode :serialization-length)
    ;; std_msgs/Int8 _pose_reset_mode
    (send _pose_reset_mode :serialization-length)
    ;; std_msgs/Int8 _primary_leg_selection
    (send _primary_leg_selection :serialization-length)
    ;; std_msgs/Int8 _secondary_leg_selection
    (send _secondary_leg_selection :serialization-length)
    ;; std_msgs/Int8 _primary_leg_state
    (send _primary_leg_state :serialization-length)
    ;; std_msgs/Int8 _secondary_leg_state
    (send _secondary_leg_state :serialization-length)
    ;; std_msgs/Int8 _parameter_selection
    (send _parameter_selection :serialization-length)
    ;; std_msgs/Int8 _parameter_adjustment
    (send _parameter_adjustment :serialization-length)
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
     ;; geometry_msgs/Point _primary_control_axis
       (send _primary_control_axis :serialize s)
     ;; geometry_msgs/Point _secondary_control_axis
       (send _secondary_control_axis :serialize s)
     ;; std_msgs/Int8 _system_state
       (send _system_state :serialize s)
     ;; std_msgs/Int8 _robot_state
       (send _robot_state :serialize s)
     ;; std_msgs/Int8 _gait_selection
       (send _gait_selection :serialize s)
     ;; std_msgs/Int8 _cruise_control_mode
       (send _cruise_control_mode :serialize s)
     ;; std_msgs/Int8 _auto_navigation_mode
       (send _auto_navigation_mode :serialize s)
     ;; std_msgs/Int8 _posing_mode
       (send _posing_mode :serialize s)
     ;; std_msgs/Int8 _pose_reset_mode
       (send _pose_reset_mode :serialize s)
     ;; std_msgs/Int8 _primary_leg_selection
       (send _primary_leg_selection :serialize s)
     ;; std_msgs/Int8 _secondary_leg_selection
       (send _secondary_leg_selection :serialize s)
     ;; std_msgs/Int8 _primary_leg_state
       (send _primary_leg_state :serialize s)
     ;; std_msgs/Int8 _secondary_leg_state
       (send _secondary_leg_state :serialize s)
     ;; std_msgs/Int8 _parameter_selection
       (send _parameter_selection :serialize s)
     ;; std_msgs/Int8 _parameter_adjustment
       (send _parameter_adjustment :serialize s)
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
   ;; geometry_msgs/Point _primary_control_axis
     (send _primary_control_axis :deserialize buf ptr-) (incf ptr- (send _primary_control_axis :serialization-length))
   ;; geometry_msgs/Point _secondary_control_axis
     (send _secondary_control_axis :deserialize buf ptr-) (incf ptr- (send _secondary_control_axis :serialization-length))
   ;; std_msgs/Int8 _system_state
     (send _system_state :deserialize buf ptr-) (incf ptr- (send _system_state :serialization-length))
   ;; std_msgs/Int8 _robot_state
     (send _robot_state :deserialize buf ptr-) (incf ptr- (send _robot_state :serialization-length))
   ;; std_msgs/Int8 _gait_selection
     (send _gait_selection :deserialize buf ptr-) (incf ptr- (send _gait_selection :serialization-length))
   ;; std_msgs/Int8 _cruise_control_mode
     (send _cruise_control_mode :deserialize buf ptr-) (incf ptr- (send _cruise_control_mode :serialization-length))
   ;; std_msgs/Int8 _auto_navigation_mode
     (send _auto_navigation_mode :deserialize buf ptr-) (incf ptr- (send _auto_navigation_mode :serialization-length))
   ;; std_msgs/Int8 _posing_mode
     (send _posing_mode :deserialize buf ptr-) (incf ptr- (send _posing_mode :serialization-length))
   ;; std_msgs/Int8 _pose_reset_mode
     (send _pose_reset_mode :deserialize buf ptr-) (incf ptr- (send _pose_reset_mode :serialization-length))
   ;; std_msgs/Int8 _primary_leg_selection
     (send _primary_leg_selection :deserialize buf ptr-) (incf ptr- (send _primary_leg_selection :serialization-length))
   ;; std_msgs/Int8 _secondary_leg_selection
     (send _secondary_leg_selection :deserialize buf ptr-) (incf ptr- (send _secondary_leg_selection :serialization-length))
   ;; std_msgs/Int8 _primary_leg_state
     (send _primary_leg_state :deserialize buf ptr-) (incf ptr- (send _primary_leg_state :serialization-length))
   ;; std_msgs/Int8 _secondary_leg_state
     (send _secondary_leg_state :deserialize buf ptr-) (incf ptr- (send _secondary_leg_state :serialization-length))
   ;; std_msgs/Int8 _parameter_selection
     (send _parameter_selection :deserialize buf ptr-) (incf ptr- (send _parameter_selection :serialization-length))
   ;; std_msgs/Int8 _parameter_adjustment
     (send _parameter_adjustment :deserialize buf ptr-) (incf ptr- (send _parameter_adjustment :serialization-length))
   ;;
   self)
  )

(setf (get syropod_remote::AndroidJoy :md5sum-) "f798248626a520efb6e3973bbe95d25a")
(setf (get syropod_remote::AndroidJoy :datatype-) "syropod_remote/AndroidJoy")
(setf (get syropod_remote::AndroidJoy :definition-)
      "Header header
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

")



(provide :syropod_remote/AndroidJoy "f798248626a520efb6e3973bbe95d25a")


