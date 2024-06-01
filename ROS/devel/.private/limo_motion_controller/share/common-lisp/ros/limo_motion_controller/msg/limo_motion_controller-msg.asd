
(cl:in-package :asdf)

(defsystem "limo_motion_controller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MotionPlan" :depends-on ("_package_MotionPlan"))
    (:file "_package_MotionPlan" :depends-on ("_package"))
    (:file "MovementController" :depends-on ("_package_MovementController"))
    (:file "_package_MovementController" :depends-on ("_package"))
  ))