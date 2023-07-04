;; Auto-generated. Do not edit!


(when (boundp 'syropod_highlevel_controller::TipState)
  (if (not (find-package "SYROPOD_HIGHLEVEL_CONTROLLER"))
    (make-package "SYROPOD_HIGHLEVEL_CONTROLLER"))
  (shadow 'TipState (find-package "SYROPOD_HIGHLEVEL_CONTROLLER")))
(unless (find-package "SYROPOD_HIGHLEVEL_CONTROLLER::TIPSTATE")
  (make-package "SYROPOD_HIGHLEVEL_CONTROLLER::TIPSTATE"))

(in-package "ROS")
;;//! \htmlinclude TipState.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass syropod_highlevel_controller::TipState
  :super ros::object
  :slots (_header _name _wrench _step_plane ))

(defmethod syropod_highlevel_controller::TipState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:name __name) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:wrench __wrench) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Wrench :init) r)) r))
    ((:step_plane __step_plane) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Vector3 :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _name __name)
   (setq _wrench __wrench)
   (setq _step_plane __step_plane)
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
  (:wrench
   (&rest __wrench)
   (if (keywordp (car __wrench))
       (send* _wrench __wrench)
     (progn
       (if __wrench (setq _wrench (car __wrench)))
       _wrench)))
  (:step_plane
   (&rest __step_plane)
   (if (keywordp (car __step_plane))
       (send* _step_plane __step_plane)
     (progn
       (if __step_plane (setq _step_plane (car __step_plane)))
       _step_plane)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string[] _name
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _name)) 4
    ;; geometry_msgs/Wrench[] _wrench
    (apply #'+ (send-all _wrench :serialization-length)) 4
    ;; geometry_msgs/Vector3[] _step_plane
    (apply #'+ (send-all _step_plane :serialization-length)) 4
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
     ;; geometry_msgs/Wrench[] _wrench
     (write-long (length _wrench) s)
     (dolist (elem _wrench)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Vector3[] _step_plane
     (write-long (length _step_plane) s)
     (dolist (elem _step_plane)
       (send elem :serialize s)
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
   ;; geometry_msgs/Wrench[] _wrench
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _wrench (let (r) (dotimes (i n) (push (instance geometry_msgs::Wrench :init) r)) r))
     (dolist (elem- _wrench)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Vector3[] _step_plane
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _step_plane (let (r) (dotimes (i n) (push (instance geometry_msgs::Vector3 :init) r)) r))
     (dolist (elem- _step_plane)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get syropod_highlevel_controller::TipState :md5sum-) "24a3486efb85ea2d52231aaad58ea97c")
(setf (get syropod_highlevel_controller::TipState :datatype-) "syropod_highlevel_controller/TipState")
(setf (get syropod_highlevel_controller::TipState :definition-)
      "Header header
string[] name
geometry_msgs/Wrench[] wrench 
geometry_msgs/Vector3[] step_plane

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
MSG: geometry_msgs/Wrench
# This represents force in free space, separated into
# its linear and angular parts.
Vector3  force
Vector3  torque

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



(provide :syropod_highlevel_controller/TipState "24a3486efb85ea2d52231aaad58ea97c")


