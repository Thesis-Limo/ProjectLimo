;; Auto-generated. Do not edit!


(when (boundp 'limo_motion_controller::MovementController)
  (if (not (find-package "LIMO_MOTION_CONTROLLER"))
    (make-package "LIMO_MOTION_CONTROLLER"))
  (shadow 'MovementController (find-package "LIMO_MOTION_CONTROLLER")))
(unless (find-package "LIMO_MOTION_CONTROLLER::MOVEMENTCONTROLLER")
  (make-package "LIMO_MOTION_CONTROLLER::MOVEMENTCONTROLLER"))

(in-package "ROS")
;;//! \htmlinclude MovementController.msg.html


(defclass limo_motion_controller::MovementController
  :super ros::object
  :slots (_speed _angle _duration ))

(defmethod limo_motion_controller::MovementController
  (:init
   (&key
    ((:speed __speed) 0.0)
    ((:angle __angle) 0.0)
    ((:duration __duration) 0.0)
    )
   (send-super :init)
   (setq _speed (float __speed))
   (setq _angle (float __angle))
   (setq _duration (float __duration))
   self)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:serialization-length
   ()
   (+
    ;; float32 _speed
    4
    ;; float32 _angle
    4
    ;; float32 _duration
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _duration
       (sys::poke _duration (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle
     (setq _angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _duration
     (setq _duration (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get limo_motion_controller::MovementController :md5sum-) "abc78f3d8ed307de203cb3588b86e07a")
(setf (get limo_motion_controller::MovementController :datatype-) "limo_motion_controller/MovementController")
(setf (get limo_motion_controller::MovementController :definition-)
      "float32 speed
float32 angle
float32 duration
")



(provide :limo_motion_controller/MovementController "abc78f3d8ed307de203cb3588b86e07a")


