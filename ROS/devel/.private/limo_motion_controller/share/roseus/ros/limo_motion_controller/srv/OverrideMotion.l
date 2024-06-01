;; Auto-generated. Do not edit!


(when (boundp 'limo_motion_controller::OverrideMotion)
  (if (not (find-package "LIMO_MOTION_CONTROLLER"))
    (make-package "LIMO_MOTION_CONTROLLER"))
  (shadow 'OverrideMotion (find-package "LIMO_MOTION_CONTROLLER")))
(unless (find-package "LIMO_MOTION_CONTROLLER::OVERRIDEMOTION")
  (make-package "LIMO_MOTION_CONTROLLER::OVERRIDEMOTION"))
(unless (find-package "LIMO_MOTION_CONTROLLER::OVERRIDEMOTIONREQUEST")
  (make-package "LIMO_MOTION_CONTROLLER::OVERRIDEMOTIONREQUEST"))
(unless (find-package "LIMO_MOTION_CONTROLLER::OVERRIDEMOTIONRESPONSE")
  (make-package "LIMO_MOTION_CONTROLLER::OVERRIDEMOTIONRESPONSE"))

(in-package "ROS")





(defclass limo_motion_controller::OverrideMotionRequest
  :super ros::object
  :slots (_speed _angle _duration _sameSpeedStart ))

(defmethod limo_motion_controller::OverrideMotionRequest
  (:init
   (&key
    ((:speed __speed) 0.0)
    ((:angle __angle) 0.0)
    ((:duration __duration) 0.0)
    ((:sameSpeedStart __sameSpeedStart) nil)
    )
   (send-super :init)
   (setq _speed (float __speed))
   (setq _angle (float __angle))
   (setq _duration (float __duration))
   (setq _sameSpeedStart __sameSpeedStart)
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
  (:sameSpeedStart
   (&optional __sameSpeedStart)
   (if __sameSpeedStart (setq _sameSpeedStart __sameSpeedStart)) _sameSpeedStart)
  (:serialization-length
   ()
   (+
    ;; float32 _speed
    4
    ;; float32 _angle
    4
    ;; float32 _duration
    4
    ;; bool _sameSpeedStart
    1
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
     ;; bool _sameSpeedStart
       (if _sameSpeedStart (write-byte -1 s) (write-byte 0 s))
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
   ;; bool _sameSpeedStart
     (setq _sameSpeedStart (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass limo_motion_controller::OverrideMotionResponse
  :super ros::object
  :slots (_alreadyRunning ))

(defmethod limo_motion_controller::OverrideMotionResponse
  (:init
   (&key
    ((:alreadyRunning __alreadyRunning) nil)
    )
   (send-super :init)
   (setq _alreadyRunning __alreadyRunning)
   self)
  (:alreadyRunning
   (&optional __alreadyRunning)
   (if __alreadyRunning (setq _alreadyRunning __alreadyRunning)) _alreadyRunning)
  (:serialization-length
   ()
   (+
    ;; bool _alreadyRunning
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _alreadyRunning
       (if _alreadyRunning (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _alreadyRunning
     (setq _alreadyRunning (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass limo_motion_controller::OverrideMotion
  :super ros::object
  :slots ())

(setf (get limo_motion_controller::OverrideMotion :md5sum-) "4955e86101808a9dfce48b4756da7f72")
(setf (get limo_motion_controller::OverrideMotion :datatype-) "limo_motion_controller/OverrideMotion")
(setf (get limo_motion_controller::OverrideMotion :request) limo_motion_controller::OverrideMotionRequest)
(setf (get limo_motion_controller::OverrideMotion :response) limo_motion_controller::OverrideMotionResponse)

(defmethod limo_motion_controller::OverrideMotionRequest
  (:response () (instance limo_motion_controller::OverrideMotionResponse :init)))

(setf (get limo_motion_controller::OverrideMotionRequest :md5sum-) "4955e86101808a9dfce48b4756da7f72")
(setf (get limo_motion_controller::OverrideMotionRequest :datatype-) "limo_motion_controller/OverrideMotionRequest")
(setf (get limo_motion_controller::OverrideMotionRequest :definition-)
      "float32 speed
float32 angle
float32 duration
bool sameSpeedStart
---
bool alreadyRunning
")

(setf (get limo_motion_controller::OverrideMotionResponse :md5sum-) "4955e86101808a9dfce48b4756da7f72")
(setf (get limo_motion_controller::OverrideMotionResponse :datatype-) "limo_motion_controller/OverrideMotionResponse")
(setf (get limo_motion_controller::OverrideMotionResponse :definition-)
      "float32 speed
float32 angle
float32 duration
bool sameSpeedStart
---
bool alreadyRunning
")



(provide :limo_motion_controller/OverrideMotion "4955e86101808a9dfce48b4756da7f72")


