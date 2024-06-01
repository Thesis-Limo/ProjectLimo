;; Auto-generated. Do not edit!


(when (boundp 'limo_yolo::map)
  (if (not (find-package "LIMO_YOLO"))
    (make-package "LIMO_YOLO"))
  (shadow 'map (find-package "LIMO_YOLO")))
(unless (find-package "LIMO_YOLO::MAP")
  (make-package "LIMO_YOLO::MAP"))

(in-package "ROS")
;;//! \htmlinclude map.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass limo_yolo::map
  :super ros::object
  :slots (_obstacles _goal ))

(defmethod limo_yolo::map
  (:init
   (&key
    ((:obstacles __obstacles) (let (r) (dotimes (i 0) (push (instance geometry_msgs::PointStamped :init) r)) r))
    ((:goal __goal) (let (r) (dotimes (i 0) (push (instance geometry_msgs::PointStamped :init) r)) r))
    )
   (send-super :init)
   (setq _obstacles __obstacles)
   (setq _goal __goal)
   self)
  (:obstacles
   (&rest __obstacles)
   (if (keywordp (car __obstacles))
       (send* _obstacles __obstacles)
     (progn
       (if __obstacles (setq _obstacles (car __obstacles)))
       _obstacles)))
  (:goal
   (&rest __goal)
   (if (keywordp (car __goal))
       (send* _goal __goal)
     (progn
       (if __goal (setq _goal (car __goal)))
       _goal)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PointStamped[] _obstacles
    (apply #'+ (send-all _obstacles :serialization-length)) 4
    ;; geometry_msgs/PointStamped[] _goal
    (apply #'+ (send-all _goal :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PointStamped[] _obstacles
     (write-long (length _obstacles) s)
     (dolist (elem _obstacles)
       (send elem :serialize s)
       )
     ;; geometry_msgs/PointStamped[] _goal
     (write-long (length _goal) s)
     (dolist (elem _goal)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PointStamped[] _obstacles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _obstacles (let (r) (dotimes (i n) (push (instance geometry_msgs::PointStamped :init) r)) r))
     (dolist (elem- _obstacles)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/PointStamped[] _goal
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _goal (let (r) (dotimes (i n) (push (instance geometry_msgs::PointStamped :init) r)) r))
     (dolist (elem- _goal)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get limo_yolo::map :md5sum-) "6e1faeb293b7b99053a7f0deeb8a2c78")
(setf (get limo_yolo::map :datatype-) "limo_yolo/map")
(setf (get limo_yolo::map :definition-)
      "geometry_msgs/PointStamped[] obstacles
geometry_msgs/PointStamped[] goal
================================================================================
MSG: geometry_msgs/PointStamped
# This represents a Point with reference coordinate frame and timestamp
Header header
Point point

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :limo_yolo/map "6e1faeb293b7b99053a7f0deeb8a2c78")


