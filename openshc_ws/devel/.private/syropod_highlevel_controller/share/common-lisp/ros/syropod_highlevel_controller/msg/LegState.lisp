; Auto-generated. Do not edit!


(cl:in-package syropod_highlevel_controller-msg)


;//! \htmlinclude LegState.msg.html

(cl:defclass <LegState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (walker_tip_pose
    :reader walker_tip_pose
    :initarg :walker_tip_pose
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped))
   (target_tip_pose
    :reader target_tip_pose
    :initarg :target_tip_pose
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped))
   (poser_tip_pose
    :reader poser_tip_pose
    :initarg :poser_tip_pose
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped))
   (model_tip_pose
    :reader model_tip_pose
    :initarg :model_tip_pose
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped))
   (actual_tip_pose
    :reader actual_tip_pose
    :initarg :actual_tip_pose
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped))
   (model_tip_velocity
    :reader model_tip_velocity
    :initarg :model_tip_velocity
    :type geometry_msgs-msg:TwistStamped
    :initform (cl:make-instance 'geometry_msgs-msg:TwistStamped))
   (joint_positions
    :reader joint_positions
    :initarg :joint_positions
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (joint_velocities
    :reader joint_velocities
    :initarg :joint_velocities
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (joint_efforts
    :reader joint_efforts
    :initarg :joint_efforts
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (stance_progress
    :reader stance_progress
    :initarg :stance_progress
    :type cl:float
    :initform 0.0)
   (swing_progress
    :reader swing_progress
    :initarg :swing_progress
    :type cl:float
    :initform 0.0)
   (time_to_swing_end
    :reader time_to_swing_end
    :initarg :time_to_swing_end
    :type cl:float
    :initform 0.0)
   (pose_delta
    :reader pose_delta
    :initarg :pose_delta
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (auto_pose
    :reader auto_pose
    :initarg :auto_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (tip_force
    :reader tip_force
    :initarg :tip_force
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (admittance_delta
    :reader admittance_delta
    :initarg :admittance_delta
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (virtual_stiffness
    :reader virtual_stiffness
    :initarg :virtual_stiffness
    :type cl:float
    :initform 0.0))
)

(cl:defclass LegState (<LegState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LegState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LegState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name syropod_highlevel_controller-msg:<LegState> is deprecated: use syropod_highlevel_controller-msg:LegState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <LegState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_highlevel_controller-msg:header-val is deprecated.  Use syropod_highlevel_controller-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <LegState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_highlevel_controller-msg:name-val is deprecated.  Use syropod_highlevel_controller-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'walker_tip_pose-val :lambda-list '(m))
(cl:defmethod walker_tip_pose-val ((m <LegState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_highlevel_controller-msg:walker_tip_pose-val is deprecated.  Use syropod_highlevel_controller-msg:walker_tip_pose instead.")
  (walker_tip_pose m))

(cl:ensure-generic-function 'target_tip_pose-val :lambda-list '(m))
(cl:defmethod target_tip_pose-val ((m <LegState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_highlevel_controller-msg:target_tip_pose-val is deprecated.  Use syropod_highlevel_controller-msg:target_tip_pose instead.")
  (target_tip_pose m))

(cl:ensure-generic-function 'poser_tip_pose-val :lambda-list '(m))
(cl:defmethod poser_tip_pose-val ((m <LegState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_highlevel_controller-msg:poser_tip_pose-val is deprecated.  Use syropod_highlevel_controller-msg:poser_tip_pose instead.")
  (poser_tip_pose m))

(cl:ensure-generic-function 'model_tip_pose-val :lambda-list '(m))
(cl:defmethod model_tip_pose-val ((m <LegState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_highlevel_controller-msg:model_tip_pose-val is deprecated.  Use syropod_highlevel_controller-msg:model_tip_pose instead.")
  (model_tip_pose m))

(cl:ensure-generic-function 'actual_tip_pose-val :lambda-list '(m))
(cl:defmethod actual_tip_pose-val ((m <LegState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_highlevel_controller-msg:actual_tip_pose-val is deprecated.  Use syropod_highlevel_controller-msg:actual_tip_pose instead.")
  (actual_tip_pose m))

(cl:ensure-generic-function 'model_tip_velocity-val :lambda-list '(m))
(cl:defmethod model_tip_velocity-val ((m <LegState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_highlevel_controller-msg:model_tip_velocity-val is deprecated.  Use syropod_highlevel_controller-msg:model_tip_velocity instead.")
  (model_tip_velocity m))

(cl:ensure-generic-function 'joint_positions-val :lambda-list '(m))
(cl:defmethod joint_positions-val ((m <LegState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_highlevel_controller-msg:joint_positions-val is deprecated.  Use syropod_highlevel_controller-msg:joint_positions instead.")
  (joint_positions m))

(cl:ensure-generic-function 'joint_velocities-val :lambda-list '(m))
(cl:defmethod joint_velocities-val ((m <LegState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_highlevel_controller-msg:joint_velocities-val is deprecated.  Use syropod_highlevel_controller-msg:joint_velocities instead.")
  (joint_velocities m))

(cl:ensure-generic-function 'joint_efforts-val :lambda-list '(m))
(cl:defmethod joint_efforts-val ((m <LegState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_highlevel_controller-msg:joint_efforts-val is deprecated.  Use syropod_highlevel_controller-msg:joint_efforts instead.")
  (joint_efforts m))

(cl:ensure-generic-function 'stance_progress-val :lambda-list '(m))
(cl:defmethod stance_progress-val ((m <LegState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_highlevel_controller-msg:stance_progress-val is deprecated.  Use syropod_highlevel_controller-msg:stance_progress instead.")
  (stance_progress m))

(cl:ensure-generic-function 'swing_progress-val :lambda-list '(m))
(cl:defmethod swing_progress-val ((m <LegState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_highlevel_controller-msg:swing_progress-val is deprecated.  Use syropod_highlevel_controller-msg:swing_progress instead.")
  (swing_progress m))

(cl:ensure-generic-function 'time_to_swing_end-val :lambda-list '(m))
(cl:defmethod time_to_swing_end-val ((m <LegState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_highlevel_controller-msg:time_to_swing_end-val is deprecated.  Use syropod_highlevel_controller-msg:time_to_swing_end instead.")
  (time_to_swing_end m))

(cl:ensure-generic-function 'pose_delta-val :lambda-list '(m))
(cl:defmethod pose_delta-val ((m <LegState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_highlevel_controller-msg:pose_delta-val is deprecated.  Use syropod_highlevel_controller-msg:pose_delta instead.")
  (pose_delta m))

(cl:ensure-generic-function 'auto_pose-val :lambda-list '(m))
(cl:defmethod auto_pose-val ((m <LegState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_highlevel_controller-msg:auto_pose-val is deprecated.  Use syropod_highlevel_controller-msg:auto_pose instead.")
  (auto_pose m))

(cl:ensure-generic-function 'tip_force-val :lambda-list '(m))
(cl:defmethod tip_force-val ((m <LegState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_highlevel_controller-msg:tip_force-val is deprecated.  Use syropod_highlevel_controller-msg:tip_force instead.")
  (tip_force m))

(cl:ensure-generic-function 'admittance_delta-val :lambda-list '(m))
(cl:defmethod admittance_delta-val ((m <LegState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_highlevel_controller-msg:admittance_delta-val is deprecated.  Use syropod_highlevel_controller-msg:admittance_delta instead.")
  (admittance_delta m))

(cl:ensure-generic-function 'virtual_stiffness-val :lambda-list '(m))
(cl:defmethod virtual_stiffness-val ((m <LegState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syropod_highlevel_controller-msg:virtual_stiffness-val is deprecated.  Use syropod_highlevel_controller-msg:virtual_stiffness instead.")
  (virtual_stiffness m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LegState>) ostream)
  "Serializes a message object of type '<LegState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'walker_tip_pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target_tip_pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'poser_tip_pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'model_tip_pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'actual_tip_pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'model_tip_velocity) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joint_positions))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'joint_positions))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joint_velocities))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'joint_velocities))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joint_efforts))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'joint_efforts))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'stance_progress))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'swing_progress))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'time_to_swing_end))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose_delta) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'auto_pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'tip_force) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'admittance_delta) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'virtual_stiffness))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LegState>) istream)
  "Deserializes a message object of type '<LegState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'walker_tip_pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target_tip_pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'poser_tip_pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'model_tip_pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'actual_tip_pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'model_tip_velocity) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joint_positions) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joint_positions)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joint_velocities) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joint_velocities)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joint_efforts) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joint_efforts)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'stance_progress) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'swing_progress) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'time_to_swing_end) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose_delta) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'auto_pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'tip_force) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'admittance_delta) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'virtual_stiffness) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LegState>)))
  "Returns string type for a message object of type '<LegState>"
  "syropod_highlevel_controller/LegState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LegState)))
  "Returns string type for a message object of type 'LegState"
  "syropod_highlevel_controller/LegState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LegState>)))
  "Returns md5sum for a message object of type '<LegState>"
  "dadda7ca412e345da1ddcca95ddf0ccc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LegState)))
  "Returns md5sum for a message object of type 'LegState"
  "dadda7ca412e345da1ddcca95ddf0ccc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LegState>)))
  "Returns full string definition for message of type '<LegState>"
  (cl:format cl:nil "Header header~%string name~%~%geometry_msgs/PoseStamped walker_tip_pose~%geometry_msgs/PoseStamped target_tip_pose~%geometry_msgs/PoseStamped poser_tip_pose~%geometry_msgs/PoseStamped model_tip_pose~%geometry_msgs/PoseStamped actual_tip_pose~%~%geometry_msgs/TwistStamped model_tip_velocity~%~%float64[] joint_positions~%float64[] joint_velocities~%float64[] joint_efforts~%~%float64 stance_progress~%float64 swing_progress~%~%float64 time_to_swing_end~%geometry_msgs/Pose pose_delta~%~%geometry_msgs/Pose auto_pose~%~%geometry_msgs/Vector3 tip_force~%geometry_msgs/Vector3 admittance_delta~%float64 virtual_stiffness~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistStamped~%# A twist with reference coordinate frame and timestamp~%Header header~%Twist twist~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LegState)))
  "Returns full string definition for message of type 'LegState"
  (cl:format cl:nil "Header header~%string name~%~%geometry_msgs/PoseStamped walker_tip_pose~%geometry_msgs/PoseStamped target_tip_pose~%geometry_msgs/PoseStamped poser_tip_pose~%geometry_msgs/PoseStamped model_tip_pose~%geometry_msgs/PoseStamped actual_tip_pose~%~%geometry_msgs/TwistStamped model_tip_velocity~%~%float64[] joint_positions~%float64[] joint_velocities~%float64[] joint_efforts~%~%float64 stance_progress~%float64 swing_progress~%~%float64 time_to_swing_end~%geometry_msgs/Pose pose_delta~%~%geometry_msgs/Pose auto_pose~%~%geometry_msgs/Vector3 tip_force~%geometry_msgs/Vector3 admittance_delta~%float64 virtual_stiffness~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistStamped~%# A twist with reference coordinate frame and timestamp~%Header header~%Twist twist~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LegState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'name))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'walker_tip_pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target_tip_pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'poser_tip_pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'model_tip_pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'actual_tip_pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'model_tip_velocity))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_positions) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_velocities) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_efforts) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     8
     8
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose_delta))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'auto_pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'tip_force))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'admittance_delta))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LegState>))
  "Converts a ROS message object to a list"
  (cl:list 'LegState
    (cl:cons ':header (header msg))
    (cl:cons ':name (name msg))
    (cl:cons ':walker_tip_pose (walker_tip_pose msg))
    (cl:cons ':target_tip_pose (target_tip_pose msg))
    (cl:cons ':poser_tip_pose (poser_tip_pose msg))
    (cl:cons ':model_tip_pose (model_tip_pose msg))
    (cl:cons ':actual_tip_pose (actual_tip_pose msg))
    (cl:cons ':model_tip_velocity (model_tip_velocity msg))
    (cl:cons ':joint_positions (joint_positions msg))
    (cl:cons ':joint_velocities (joint_velocities msg))
    (cl:cons ':joint_efforts (joint_efforts msg))
    (cl:cons ':stance_progress (stance_progress msg))
    (cl:cons ':swing_progress (swing_progress msg))
    (cl:cons ':time_to_swing_end (time_to_swing_end msg))
    (cl:cons ':pose_delta (pose_delta msg))
    (cl:cons ':auto_pose (auto_pose msg))
    (cl:cons ':tip_force (tip_force msg))
    (cl:cons ':admittance_delta (admittance_delta msg))
    (cl:cons ':virtual_stiffness (virtual_stiffness msg))
))
