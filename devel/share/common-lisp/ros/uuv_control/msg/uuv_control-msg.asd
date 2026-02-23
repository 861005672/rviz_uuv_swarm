
(cl:in-package :asdf)

(defsystem "uuv_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "State3D" :depends-on ("_package_State3D"))
    (:file "_package_State3D" :depends-on ("_package"))
  ))