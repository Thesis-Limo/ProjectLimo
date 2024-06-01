;; Auto-generated. Do not edit!


(when (boundp 'limo_behaviour_tree::EndGoal)
  (if (not (find-package "LIMO_BEHAVIOUR_TREE"))
    (make-package "LIMO_BEHAVIOUR_TREE"))
  (shadow 'EndGoal (find-package "LIMO_BEHAVIOUR_TREE")))
(unless (find-package "LIMO_BEHAVIOUR_TREE::ENDGOAL")
  (make-package "LIMO_BEHAVIOUR_TREE::ENDGOAL"))
(unless (find-package "LIMO_BEHAVIOUR_TREE::ENDGOALREQUEST")
  (make-package "LIMO_BEHAVIOUR_TREE::ENDGOALREQUEST"))
(unless (find-package "LIMO_BEHAVIOUR_TREE::ENDGOALRESPONSE")
  (make-package "LIMO_BEHAVIOUR_TREE::ENDGOALRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass limo_behaviour_tree::EndGoalRequest
  :super ros::object
  :slots (_goalPos ))

(defmethod limo_behaviour_tree::EndGoalRequest
  (:init
   (&key
    ((:goalPos __goalPos) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _goalPos __goalPos)
   self)
  (:goalPos
   (&rest __goalPos)
   (if (keywordp (car __goalPos))
       (send* _goalPos __goalPos)
     (progn
       (if __goalPos (setq _goalPos (car __goalPos)))
       _goalPos)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _goalPos
    (send _goalPos :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _goalPos
       (send _goalPos :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _goalPos
     (send _goalPos :deserialize buf ptr-) (incf ptr- (send _goalPos :serialization-length))
   ;;
   self)
  )

(defclass limo_behaviour_tree::EndGoalResponse
  :super ros::object
  :slots ())

(defmethod limo_behaviour_tree::EndGoalResponse
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

(defclass limo_behaviour_tree::EndGoal
  :super ros::object
  :slots ())

(setf (get limo_behaviour_tree::EndGoal :md5sum-) "f43f04c866179f13e0ff9145477de5ab")
(setf (get limo_behaviour_tree::EndGoal :datatype-) "limo_behaviour_tree/EndGoal")
(setf (get limo_behaviour_tree::EndGoal :request) limo_behaviour_tree::EndGoalRequest)
(setf (get limo_behaviour_tree::EndGoal :response) limo_behaviour_tree::EndGoalResponse)

(defmethod limo_behaviour_tree::EndGoalRequest
  (:response () (instance limo_behaviour_tree::EndGoalResponse :init)))

(setf (get limo_behaviour_tree::EndGoalRequest :md5sum-) "f43f04c866179f13e0ff9145477de5ab")
(setf (get limo_behaviour_tree::EndGoalRequest :datatype-) "limo_behaviour_tree/EndGoalRequest")
(setf (get limo_behaviour_tree::EndGoalRequest :definition-)
      "geometry_msgs/Point goalPos

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---

")

(setf (get limo_behaviour_tree::EndGoalResponse :md5sum-) "f43f04c866179f13e0ff9145477de5ab")
(setf (get limo_behaviour_tree::EndGoalResponse :datatype-) "limo_behaviour_tree/EndGoalResponse")
(setf (get limo_behaviour_tree::EndGoalResponse :definition-)
      "geometry_msgs/Point goalPos

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---

")



(provide :limo_behaviour_tree/EndGoal "f43f04c866179f13e0ff9145477de5ab")


