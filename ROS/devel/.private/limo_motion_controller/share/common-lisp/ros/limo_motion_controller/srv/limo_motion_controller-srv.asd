
(cl:in-package :asdf)

(defsystem "limo_motion_controller-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "OverrideMotion" :depends-on ("_package_OverrideMotion"))
    (:file "_package_OverrideMotion" :depends-on ("_package"))
  ))