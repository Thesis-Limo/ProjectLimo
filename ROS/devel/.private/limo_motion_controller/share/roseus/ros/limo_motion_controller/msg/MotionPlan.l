;; Auto-generated. Do not edit!


(when (boundp 'limo_motion_controller::MotionPlan)
  (if (not (find-package "LIMO_MOTION_CONTROLLER"))
    (make-package "LIMO_MOTION_CONTROLLER"))
  (shadow 'MotionPlan (find-package "LIMO_MOTION_CONTROLLER")))
(unless (find-package "LIMO_MOTION_CONTROLLER::MOTIONPLAN")
  (make-package "LIMO_MOTION_CONTROLLER::MOTIONPLAN"))

(in-package "ROS")
;;//! \htmlinclude MotionPlan.msg.html


(defclass limo_motion_controller::MotionPlan
  :super ros::object
  :slots (_sequence ))

(defmethod limo_motion_controller::MotionPlan
  (:init
   (&key
    ((:sequence __sequence) (let (r) (dotimes (i 0) (push (instance limo_motion_controller::MovementController :init) r)) r))
    )
   (send-super :init)
   (setq _sequence __sequence)
   self)
  (:sequence
   (&rest __sequence)
   (if (keywordp (car __sequence))
       (send* _sequence __sequence)
     (progn
       (if __sequence (setq _sequence (car __sequence)))
       _sequence)))
  (:serialization-length
   ()
   (+
    ;; limo_motion_controller/MovementController[] _sequence
    (apply #'+ (send-all _sequence :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; limo_motion_controller/MovementController[] _sequence
     (write-long (length _sequence) s)
     (dolist (elem _sequence)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; limo_motion_controller/MovementController[] _sequence
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _sequence (let (r) (dotimes (i n) (push (instance limo_motion_controller::MovementController :init) r)) r))
     (dolist (elem- _sequence)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get limo_motion_controller::MotionPlan :md5sum-) "862f364494b8552b6422445d2907f57e")
(setf (get limo_motion_controller::MotionPlan :datatype-) "limo_motion_controller/MotionPlan")
(setf (get limo_motion_controller::MotionPlan :definition-)
      "limo_motion_controller/MovementController[] sequence
================================================================================
MSG: limo_motion_controller/MovementController
float32 speed
float32 angle
float32 duration
")



(provide :limo_motion_controller/MotionPlan "862f364494b8552b6422445d2907f57e")


