
(cl:in-package :asdf)

(defsystem "syropod_remote-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AndroidJoy" :depends-on ("_package_AndroidJoy"))
    (:file "_package_AndroidJoy" :depends-on ("_package"))
    (:file "AndroidSensor" :depends-on ("_package_AndroidSensor"))
    (:file "_package_AndroidSensor" :depends-on ("_package"))
  ))