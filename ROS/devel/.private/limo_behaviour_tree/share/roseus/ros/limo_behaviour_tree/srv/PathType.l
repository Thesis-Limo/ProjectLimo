;; Auto-generated. Do not edit!


(when (boundp 'limo_behaviour_tree::PathType)
  (if (not (find-package "LIMO_BEHAVIOUR_TREE"))
    (make-package "LIMO_BEHAVIOUR_TREE"))
  (shadow 'PathType (find-package "LIMO_BEHAVIOUR_TREE")))
(unless (find-package "LIMO_BEHAVIOUR_TREE::PATHTYPE")
  (make-package "LIMO_BEHAVIOUR_TREE::PATHTYPE"))
(unless (find-package "LIMO_BEHAVIOUR_TREE::PATHTYPEREQUEST")
  (make-package "LIMO_BEHAVIOUR_TREE::PATHTYPEREQUEST"))
(unless (find-package "LIMO_BEHAVIOUR_TREE::PATHTYPERESPONSE")
  (make-package "LIMO_BEHAVIOUR_TREE::PATHTYPERESPONSE"))

(in-package "ROS")





(defclass limo_behaviour_tree::PathTypeRequest
  :super ros::object
  :slots (_pathType ))

(defmethod limo_behaviour_tree::PathTypeRequest
  (:init
   (&key
    ((:pathType __pathType) 0)
    )
   (send-super :init)
   (setq _pathType (round __pathType))
   self)
  (:pathType
   (&optional __pathType)
   (if __pathType (setq _pathType __pathType)) _pathType)
  (:serialization-length
   ()
   (+
    ;; int32 _pathType
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _pathType
       (write-long _pathType s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _pathType
     (setq _pathType (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass limo_behaviour_tree::PathTypeResponse
  :super ros::object
  :slots ())

(defmethod limo_behaviour_tree::PathTypeResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass limo_behaviour_tree::PathType
  :super ros::object
  :slots ())

(setf (get limo_behaviour_tree::PathType :md5sum-) "11e5c53def65876fc1708cc2295e05a4")
(setf (get limo_behaviour_tree::PathType :datatype-) "limo_behaviour_tree/PathType")
(setf (get limo_behaviour_tree::PathType :request) limo_behaviour_tree::PathTypeRequest)
(setf (get limo_behaviour_tree::PathType :response) limo_behaviour_tree::PathTypeResponse)

(defmethod limo_behaviour_tree::PathTypeRequest
  (:response () (instance limo_behaviour_tree::PathTypeResponse :init)))

(setf (get limo_behaviour_tree::PathTypeRequest :md5sum-) "11e5c53def65876fc1708cc2295e05a4")
(setf (get limo_behaviour_tree::PathTypeRequest :datatype-) "limo_behaviour_tree/PathTypeRequest")
(setf (get limo_behaviour_tree::PathTypeRequest :definition-)
      "int32 pathType
---

")

(setf (get limo_behaviour_tree::PathTypeResponse :md5sum-) "11e5c53def65876fc1708cc2295e05a4")
(setf (get limo_behaviour_tree::PathTypeResponse :datatype-) "limo_behaviour_tree/PathTypeResponse")
(setf (get limo_behaviour_tree::PathTypeResponse :definition-)
      "int32 pathType
---

")



(provide :limo_behaviour_tree/PathType "11e5c53def65876fc1708cc2295e05a4")


