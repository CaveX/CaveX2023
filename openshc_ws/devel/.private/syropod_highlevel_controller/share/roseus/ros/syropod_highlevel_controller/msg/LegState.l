;; Auto-generated. Do not edit!


(when (boundp 'syropod_highlevel_controller::LegState)
  (if (not (find-package "SYROPOD_HIGHLEVEL_CONTROLLER"))
    (make-package "SYROPOD_HIGHLEVEL_CONTROLLER"))
  (shadow 'LegState (find-package "SYROPOD_HIGHLEVEL_CONTROLLER")))
(unless (find-package "SYROPOD_HIGHLEVEL_CONTROLLER::LEGSTATE")
  (make-package "SYROPOD_HIGHLEVEL_CONTROLLER::LEGSTATE"))

(in-package "ROS")
;;//! \htmlinclude LegState.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass syropod_highlevel_controller::LegState
  :super ros::object
  :slots (_header _name _walker_tip_pose _target_tip_pose _poser_tip_pose _model_tip_pose _actual_tip_pose _model_tip_velocity _joint_positions _joint_velocities _joint_efforts _stance_progress _swing_progress _time_to_swing_end _pose_delta _auto_pose _tip_force _admittance_delta _virtual_stiffness ))

(defmethod syropod_highlevel_controller::LegState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:name __name) "")
    ((:walker_tip_pose __walker_tip_pose) (instance geometry_msgs::PoseStamped :init))
    ((:target_tip_pose __target_tip_pose) (instance geometry_msgs::PoseStamped :init))
    ((:poser_tip_pose __poser_tip_pose) (instance geometry_msgs::PoseStamped :init))
    ((:model_tip_pose __model_tip_pose) (instance geometry_msgs::PoseStamped :init))
    ((:actual_tip_pose __actual_tip_pose) (instance geometry_msgs::PoseStamped :init))
    ((:model_tip_velocity __model_tip_velocity) (instance geometry_msgs::TwistStamped :init))
    ((:joint_positions __joint_positions) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:joint_velocities __joint_velocities) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:joint_efforts __joint_efforts) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:stance_progress __stance_progress) 0.0)
    ((:swing_progress __swing_progress) 0.0)
    ((:time_to_swing_end __time_to_swing_end) 0.0)
    ((:pose_delta __pose_delta) (instance geometry_msgs::Pose :init))
    ((:auto_pose __auto_pose) (instance geometry_msgs::Pose :init))
    ((:tip_force __tip_force) (instance geometry_msgs::Vector3 :init))
    ((:admittance_delta __admittance_delta) (instance geometry_msgs::Vector3 :init))
    ((:virtual_stiffness __virtual_stiffness) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _name (string __name))
   (setq _walker_tip_pose __walker_tip_pose)
   (setq _target_tip_pose __target_tip_pose)
   (setq _poser_tip_pose __poser_tip_pose)
   (setq _model_tip_pose __model_tip_pose)
   (setq _actual_tip_pose __actual_tip_pose)
   (setq _model_tip_velocity __model_tip_velocity)
   (setq _joint_positions __joint_positions)
   (setq _joint_velocities __joint_velocities)
   (setq _joint_efforts __joint_efforts)
   (setq _stance_progress (float __stance_progress))
   (setq _swing_progress (float __swing_progress))
   (setq _time_to_swing_end (float __time_to_swing_end))
   (setq _pose_delta __pose_delta)
   (setq _auto_pose __auto_pose)
   (setq _tip_force __tip_force)
   (setq _admittance_delta __admittance_delta)
   (setq _virtual_stiffness (float __virtual_stiffness))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:walker_tip_pose
   (&rest __walker_tip_pose)
   (if (keywordp (car __walker_tip_pose))
       (send* _walker_tip_pose __walker_tip_pose)
     (progn
       (if __walker_tip_pose (setq _walker_tip_pose (car __walker_tip_pose)))
       _walker_tip_pose)))
  (:target_tip_pose
   (&rest __target_tip_pose)
   (if (keywordp (car __target_tip_pose))
       (send* _target_tip_pose __target_tip_pose)
     (progn
       (if __target_tip_pose (setq _target_tip_pose (car __target_tip_pose)))
       _target_tip_pose)))
  (:poser_tip_pose
   (&rest __poser_tip_pose)
   (if (keywordp (car __poser_tip_pose))
       (send* _poser_tip_pose __poser_tip_pose)
     (progn
       (if __poser_tip_pose (setq _poser_tip_pose (car __poser_tip_pose)))
       _poser_tip_pose)))
  (:model_tip_pose
   (&rest __model_tip_pose)
   (if (keywordp (car __model_tip_pose))
       (send* _model_tip_pose __model_tip_pose)
     (progn
       (if __model_tip_pose (setq _model_tip_pose (car __model_tip_pose)))
       _model_tip_pose)))
  (:actual_tip_pose
   (&rest __actual_tip_pose)
   (if (keywordp (car __actual_tip_pose))
       (send* _actual_tip_pose __actual_tip_pose)
     (progn
       (if __actual_tip_pose (setq _actual_tip_pose (car __actual_tip_pose)))
       _actual_tip_pose)))
  (:model_tip_velocity
   (&rest __model_tip_velocity)
   (if (keywordp (car __model_tip_velocity))
       (send* _model_tip_velocity __model_tip_velocity)
     (progn
       (if __model_tip_velocity (setq _model_tip_velocity (car __model_tip_velocity)))
       _model_tip_velocity)))
  (:joint_positions
   (&optional __joint_positions)
   (if __joint_positions (setq _joint_positions __joint_positions)) _joint_positions)
  (:joint_velocities
   (&optional __joint_velocities)
   (if __joint_velocities (setq _joint_velocities __joint_velocities)) _joint_velocities)
  (:joint_efforts
   (&optional __joint_efforts)
   (if __joint_efforts (setq _joint_efforts __joint_efforts)) _joint_efforts)
  (:stance_progress
   (&optional __stance_progress)
   (if __stance_progress (setq _stance_progress __stance_progress)) _stance_progress)
  (:swing_progress
   (&optional __swing_progress)
   (if __swing_progress (setq _swing_progress __swing_progress)) _swing_progress)
  (:time_to_swing_end
   (&optional __time_to_swing_end)
   (if __time_to_swing_end (setq _time_to_swing_end __time_to_swing_end)) _time_to_swing_end)
  (:pose_delta
   (&rest __pose_delta)
   (if (keywordp (car __pose_delta))
       (send* _pose_delta __pose_delta)
     (progn
       (if __pose_delta (setq _pose_delta (car __pose_delta)))
       _pose_delta)))
  (:auto_pose
   (&rest __auto_pose)
   (if (keywordp (car __auto_pose))
       (send* _auto_pose __auto_pose)
     (progn
       (if __auto_pose (setq _auto_pose (car __auto_pose)))
       _auto_pose)))
  (:tip_force
   (&rest __tip_force)
   (if (keywordp (car __tip_force))
       (send* _tip_force __tip_force)
     (progn
       (if __tip_force (setq _tip_force (car __tip_force)))
       _tip_force)))
  (:admittance_delta
   (&rest __admittance_delta)
   (if (keywordp (car __admittance_delta))
       (send* _admittance_delta __admittance_delta)
     (progn
       (if __admittance_delta (setq _admittance_delta (car __admittance_delta)))
       _admittance_delta)))
  (:virtual_stiffness
   (&optional __virtual_stiffness)
   (if __virtual_stiffness (setq _virtual_stiffness __virtual_stiffness)) _virtual_stiffness)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _name
    4 (length _name)
    ;; geometry_msgs/PoseStamped _walker_tip_pose
    (send _walker_tip_pose :serialization-length)
    ;; geometry_msgs/PoseStamped _target_tip_pose
    (send _target_tip_pose :serialization-length)
    ;; geometry_msgs/PoseStamped _poser_tip_pose
    (send _poser_tip_pose :serialization-length)
    ;; geometry_msgs/PoseStamped _model_tip_pose
    (send _model_tip_pose :serialization-length)
    ;; geometry_msgs/PoseStamped _actual_tip_pose
    (send _actual_tip_pose :serialization-length)
    ;; geometry_msgs/TwistStamped _model_tip_velocity
    (send _model_tip_velocity :serialization-length)
    ;; float64[] _joint_positions
    (* 8    (length _joint_positions)) 4
    ;; float64[] _joint_velocities
    (* 8    (length _joint_velocities)) 4
    ;; float64[] _joint_efforts
    (* 8    (length _joint_efforts)) 4
    ;; float64 _stance_progress
    8
    ;; float64 _swing_progress
    8
    ;; float64 _time_to_swing_end
    8
    ;; geometry_msgs/Pose _pose_delta
    (send _pose_delta :serialization-length)
    ;; geometry_msgs/Pose _auto_pose
    (send _auto_pose :serialization-length)
    ;; geometry_msgs/Vector3 _tip_force
    (send _tip_force :serialization-length)
    ;; geometry_msgs/Vector3 _admittance_delta
    (send _admittance_delta :serialization-length)
    ;; float64 _virtual_stiffness
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; geometry_msgs/PoseStamped _walker_tip_pose
       (send _walker_tip_pose :serialize s)
     ;; geometry_msgs/PoseStamped _target_tip_pose
       (send _target_tip_pose :serialize s)
     ;; geometry_msgs/PoseStamped _poser_tip_pose
       (send _poser_tip_pose :serialize s)
     ;; geometry_msgs/PoseStamped _model_tip_pose
       (send _model_tip_pose :serialize s)
     ;; geometry_msgs/PoseStamped _actual_tip_pose
       (send _actual_tip_pose :serialize s)
     ;; geometry_msgs/TwistStamped _model_tip_velocity
       (send _model_tip_velocity :serialize s)
     ;; float64[] _joint_positions
     (write-long (length _joint_positions) s)
     (dotimes (i (length _joint_positions))
       (sys::poke (elt _joint_positions i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _joint_velocities
     (write-long (length _joint_velocities) s)
     (dotimes (i (length _joint_velocities))
       (sys::poke (elt _joint_velocities i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _joint_efforts
     (write-long (length _joint_efforts) s)
     (dotimes (i (length _joint_efforts))
       (sys::poke (elt _joint_efforts i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _stance_progress
       (sys::poke _stance_progress (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _swing_progress
       (sys::poke _swing_progress (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _time_to_swing_end
       (sys::poke _time_to_swing_end (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/Pose _pose_delta
       (send _pose_delta :serialize s)
     ;; geometry_msgs/Pose _auto_pose
       (send _auto_pose :serialize s)
     ;; geometry_msgs/Vector3 _tip_force
       (send _tip_force :serialize s)
     ;; geometry_msgs/Vector3 _admittance_delta
       (send _admittance_delta :serialize s)
     ;; float64 _virtual_stiffness
       (sys::poke _virtual_stiffness (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/PoseStamped _walker_tip_pose
     (send _walker_tip_pose :deserialize buf ptr-) (incf ptr- (send _walker_tip_pose :serialization-length))
   ;; geometry_msgs/PoseStamped _target_tip_pose
     (send _target_tip_pose :deserialize buf ptr-) (incf ptr- (send _target_tip_pose :serialization-length))
   ;; geometry_msgs/PoseStamped _poser_tip_pose
     (send _poser_tip_pose :deserialize buf ptr-) (incf ptr- (send _poser_tip_pose :serialization-length))
   ;; geometry_msgs/PoseStamped _model_tip_pose
     (send _model_tip_pose :deserialize buf ptr-) (incf ptr- (send _model_tip_pose :serialization-length))
   ;; geometry_msgs/PoseStamped _actual_tip_pose
     (send _actual_tip_pose :deserialize buf ptr-) (incf ptr- (send _actual_tip_pose :serialization-length))
   ;; geometry_msgs/TwistStamped _model_tip_velocity
     (send _model_tip_velocity :deserialize buf ptr-) (incf ptr- (send _model_tip_velocity :serialization-length))
   ;; float64[] _joint_positions
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_positions (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_positions i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _joint_velocities
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_velocities (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_velocities i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _joint_efforts
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_efforts (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_efforts i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64 _stance_progress
     (setq _stance_progress (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _swing_progress
     (setq _swing_progress (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _time_to_swing_end
     (setq _time_to_swing_end (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/Pose _pose_delta
     (send _pose_delta :deserialize buf ptr-) (incf ptr- (send _pose_delta :serialization-length))
   ;; geometry_msgs/Pose _auto_pose
     (send _auto_pose :deserialize buf ptr-) (incf ptr- (send _auto_pose :serialization-length))
   ;; geometry_msgs/Vector3 _tip_force
     (send _tip_force :deserialize buf ptr-) (incf ptr- (send _tip_force :serialization-length))
   ;; geometry_msgs/Vector3 _admittance_delta
     (send _admittance_delta :deserialize buf ptr-) (incf ptr- (send _admittance_delta :serialization-length))
   ;; float64 _virtual_stiffness
     (setq _virtual_stiffness (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get syropod_highlevel_controller::LegState :md5sum-) "dadda7ca412e345da1ddcca95ddf0ccc")
(setf (get syropod_highlevel_controller::LegState :datatype-) "syropod_highlevel_controller/LegState")
(setf (get syropod_highlevel_controller::LegState :definition-)
      "Header header
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
")



(provide :syropod_highlevel_controller/LegState "dadda7ca412e345da1ddcca95ddf0ccc")


