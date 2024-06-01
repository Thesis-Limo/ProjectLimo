
(cl:in-package :asdf)

(defsystem "limo_yolo-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "map" :depends-on ("_package_map"))
    (:file "_package_map" :depends-on ("_package"))
  ))