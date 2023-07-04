
(cl:in-package :asdf)

(defsystem "syropod_highlevel_controller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "LegState" :depends-on ("_package_LegState"))
    (:file "_package_LegState" :depends-on ("_package"))
    (:file "TargetTipPose" :depends-on ("_package_TargetTipPose"))
    (:file "_package_TargetTipPose" :depends-on ("_package"))
    (:file "TipState" :depends-on ("_package_TipState"))
    (:file "_package_TipState" :depends-on ("_package"))
  ))