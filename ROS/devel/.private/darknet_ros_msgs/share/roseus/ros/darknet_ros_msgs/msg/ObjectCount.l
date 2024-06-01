;; Auto-generated. Do not edit!


(when (boundp 'darknet_ros_msgs::ObjectCount)
  (if (not (find-package "DARKNET_ROS_MSGS"))
    (make-package "DARKNET_ROS_MSGS"))
  (shadow 'ObjectCount (find-package "DARKNET_ROS_MSGS")))
(unless (find-package "DARKNET_ROS_MSGS::OBJECTCOUNT")
  (make-package "DARKNET_ROS_MSGS::OBJECTCOUNT"))

(in-package "ROS")
;;//! \htmlinclude ObjectCount.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass darknet_ros_msgs::ObjectCount
  :super ros::object
  :slots (_header _count _image_header ))

(defmethod darknet_ros_msgs::ObjectCount
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:count __count) 0)
    ((:image_header __image_header) (instance std_msgs::Header :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _count (round __count))
   (setq _image_header __image_header)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:count
   (&optional __count)
   (if __count (setq _count __count)) _count)
  (:image_header
   (&rest __image_header)
   (if (keywordp (car __image_header))
       (send* _image_header __image_header)
     (progn
       (if __image_header (setq _image_header (car __image_header)))
       _image_header)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int8 _count
    1
    ;; std_msgs/Header _image_header
    (send _image_header :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int8 _count
       (write-byte _count s)
     ;; std_msgs/Header _image_header
       (send _image_header :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int8 _count
     (setq _count (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _count 127) (setq _count (- _count 256)))
   ;; std_msgs/Header _image_header
     (send _image_header :deserialize buf ptr-) (incf ptr- (send _image_header :serialization-length))
   ;;
   self)
  )

(setf (get darknet_ros_msgs::ObjectCount :md5sum-) "0929fd7662995663a55270abf2d47b20")
(setf (get darknet_ros_msgs::ObjectCount :datatype-) "darknet_ros_msgs/ObjectCount")
(setf (get darknet_ros_msgs::ObjectCount :definition-)
      "Header header
int8 count
Header image_header
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

")



(provide :darknet_ros_msgs/ObjectCount "0929fd7662995663a55270abf2d47b20")


