;; Auto-generated. Do not edit!


(when (boundp 'limo_behaviour_tree::TypeObjectTracking)
  (if (not (find-package "LIMO_BEHAVIOUR_TREE"))
    (make-package "LIMO_BEHAVIOUR_TREE"))
  (shadow 'TypeObjectTracking (find-package "LIMO_BEHAVIOUR_TREE")))
(unless (find-package "LIMO_BEHAVIOUR_TREE::TYPEOBJECTTRACKING")
  (make-package "LIMO_BEHAVIOUR_TREE::TYPEOBJECTTRACKING"))
(unless (find-package "LIMO_BEHAVIOUR_TREE::TYPEOBJECTTRACKINGREQUEST")
  (make-package "LIMO_BEHAVIOUR_TREE::TYPEOBJECTTRACKINGREQUEST"))
(unless (find-package "LIMO_BEHAVIOUR_TREE::TYPEOBJECTTRACKINGRESPONSE")
  (make-package "LIMO_BEHAVIOUR_TREE::TYPEOBJECTTRACKINGRESPONSE"))

(in-package "ROS")





(defclass limo_behaviour_tree::TypeObjectTrackingRequest
  :super ros::object
  :slots (_objectID ))

(defmethod limo_behaviour_tree::TypeObjectTrackingRequest
  (:init
   (&key
    ((:objectID __objectID) 0)
    )
   (send-super :init)
   (setq _objectID (round __objectID))
   self)
  (:objectID
   (&optional __objectID)
   (if __objectID (setq _objectID __objectID)) _objectID)
  (:serialization-length
   ()
   (+
    ;; int32 _objectID
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _objectID
       (write-long _objectID s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _objectID
     (setq _objectID (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass limo_behaviour_tree::TypeObjectTrackingResponse
  :super ros::object
  :slots ())

(defmethod limo_behaviour_tree::TypeObjectTrackingResponse
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

(defclass limo_behaviour_tree::TypeObjectTracking
  :super ros::object
  :slots ())

(setf (get limo_behaviour_tree::TypeObjectTracking :md5sum-) "0b08c3191adc4004289244e57c20d2f6")
(setf (get limo_behaviour_tree::TypeObjectTracking :datatype-) "limo_behaviour_tree/TypeObjectTracking")
(setf (get limo_behaviour_tree::TypeObjectTracking :request) limo_behaviour_tree::TypeObjectTrackingRequest)
(setf (get limo_behaviour_tree::TypeObjectTracking :response) limo_behaviour_tree::TypeObjectTrackingResponse)

(defmethod limo_behaviour_tree::TypeObjectTrackingRequest
  (:response () (instance limo_behaviour_tree::TypeObjectTrackingResponse :init)))

(setf (get limo_behaviour_tree::TypeObjectTrackingRequest :md5sum-) "0b08c3191adc4004289244e57c20d2f6")
(setf (get limo_behaviour_tree::TypeObjectTrackingRequest :datatype-) "limo_behaviour_tree/TypeObjectTrackingRequest")
(setf (get limo_behaviour_tree::TypeObjectTrackingRequest :definition-)
      "int32 objectID
---
")

(setf (get limo_behaviour_tree::TypeObjectTrackingResponse :md5sum-) "0b08c3191adc4004289244e57c20d2f6")
(setf (get limo_behaviour_tree::TypeObjectTrackingResponse :datatype-) "limo_behaviour_tree/TypeObjectTrackingResponse")
(setf (get limo_behaviour_tree::TypeObjectTrackingResponse :definition-)
      "int32 objectID
---
")



(provide :limo_behaviour_tree/TypeObjectTracking "0b08c3191adc4004289244e57c20d2f6")


