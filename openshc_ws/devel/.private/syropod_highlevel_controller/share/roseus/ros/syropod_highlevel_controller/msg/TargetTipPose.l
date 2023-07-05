;; Auto-generated. Do not edit!


(when (boundp 'syropod_highlevel_controller::TargetTipPose)
  (if (not (find-package "SYROPOD_HIGHLEVEL_CONTROLLER"))
    (make-package "SYROPOD_HIGHLEVEL_CONTROLLER"))
  (shadow 'TargetTipPose (find-package "SYROPOD_HIGHLEVEL_CONTROLLER")))
(unless (find-package "SYROPOD_HIGHLEVEL_CONTROLLER::TARGETTIPPOSE")
  (make-package "SYROPOD_HIGHLEVEL_CONTROLLER::TARGETTIPPOSE"))

(in-package "ROS")
;;//! \htmlinclude TargetTipPose.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass syropod_highlevel_controller::TargetTipPose
  :super ros::object
  :slots (_header _name _target _stance _swing_clearance ))

(defmethod syropod_highlevel_controller::TargetTipPose
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:name __name) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:target __target) (let (r) (dotimes (i 0) (push (instance geometry_msgs::PoseStamped :init) r)) r))
    ((:stance __stance) (let (r) (dotimes (i 0) (push (instance geometry_msgs::PoseStamped :init) r)) r))
    ((:swing_clearance __swing_clearance) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _name __name)
   (setq _target __target)
   (setq _stance __stance)
   (setq _swing_clearance __swing_clearance)
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
  (:target
   (&rest __target)
   (if (keywordp (car __target))
       (send* _target __target)
     (progn
       (if __target (setq _target (car __target)))
       _target)))
  (:stance
   (&rest __stance)
   (if (keywordp (car __stance))
       (send* _stance __stance)
     (progn
       (if __stance (setq _stance (car __stance)))
       _stance)))
  (:swing_clearance
   (&optional __swing_clearance)
   (if __swing_clearance (setq _swing_clearance __swing_clearance)) _swing_clearance)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string[] _name
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _name)) 4
    ;; geometry_msgs/PoseStamped[] _target
    (apply #'+ (send-all _target :serialization-length)) 4
    ;; geometry_msgs/PoseStamped[] _stance
    (apply #'+ (send-all _stance :serialization-length)) 4
    ;; float64[] _swing_clearance
    (* 8    (length _swing_clearance)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string[] _name
     (write-long (length _name) s)
     (dolist (elem _name)
       (write-long (length elem) s) (princ elem s)
       )
     ;; geometry_msgs/PoseStamped[] _target
     (write-long (length _target) s)
     (dolist (elem _target)
       (send elem :serialize s)
       )
     ;; geometry_msgs/PoseStamped[] _stance
     (write-long (length _stance) s)
     (dolist (elem _stance)
       (send elem :serialize s)
       )
     ;; float64[] _swing_clearance
     (write-long (length _swing_clearance) s)
     (dotimes (i (length _swing_clearance))
       (sys::poke (elt _swing_clearance i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string[] _name
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _name (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _name i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; geometry_msgs/PoseStamped[] _target
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _target (let (r) (dotimes (i n) (push (instance geometry_msgs::PoseStamped :init) r)) r))
     (dolist (elem- _target)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/PoseStamped[] _stance
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _stance (let (r) (dotimes (i n) (push (instance geometry_msgs::PoseStamped :init) r)) r))
     (dolist (elem- _stance)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float64[] _swing_clearance
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _swing_clearance (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _swing_clearance i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get syropod_highlevel_controller::TargetTipPose :md5sum-) "05c537cc093e23228ee76fae952f2b3e")
(setf (get syropod_highlevel_controller::TargetTipPose :datatype-) "syropod_highlevel_controller/TargetTipPose")
(setf (get syropod_highlevel_controller::TargetTipPose :definition-)
      "Header header
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

")



(provide :syropod_highlevel_controller/TargetTipPose "05c537cc093e23228ee76fae952f2b3e")


