; Auto-generated. Do not edit!


(cl:in-package darknet_ros_msgs-msg)


;//! \htmlinclude ObjectCount.msg.html

(cl:defclass <ObjectCount> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (count
    :reader count
    :initarg :count
    :type cl:fixnum
    :initform 0)
   (image_header
    :reader image_header
    :initarg :image_header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header)))
)

(cl:defclass ObjectCount (<ObjectCount>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObjectCount>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObjectCount)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name darknet_ros_msgs-msg:<ObjectCount> is deprecated: use darknet_ros_msgs-msg:ObjectCount instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ObjectCount>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader darknet_ros_msgs-msg:header-val is deprecated.  Use darknet_ros_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'count-val :lambda-list '(m))
(cl:defmethod count-val ((m <ObjectCount>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader darknet_ros_msgs-msg:count-val is deprecated.  Use darknet_ros_msgs-msg:count instead.")
  (count m))

(cl:ensure-generic-function 'image_header-val :lambda-list '(m))
(cl:defmethod image_header-val ((m <ObjectCount>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader darknet_ros_msgs-msg:image_header-val is deprecated.  Use darknet_ros_msgs-msg:image_header instead.")
  (image_header m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObjectCount>) ostream)
  "Serializes a message object of type '<ObjectCount>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'count)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image_header) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObjectCount>) istream)
  "Deserializes a message object of type '<ObjectCount>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'count) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image_header) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObjectCount>)))
  "Returns string type for a message object of type '<ObjectCount>"
  "darknet_ros_msgs/ObjectCount")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObjectCount)))
  "Returns string type for a message object of type 'ObjectCount"
  "darknet_ros_msgs/ObjectCount")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObjectCount>)))
  "Returns md5sum for a message object of type '<ObjectCount>"
  "0929fd7662995663a55270abf2d47b20")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObjectCount)))
  "Returns md5sum for a message object of type 'ObjectCount"
  "0929fd7662995663a55270abf2d47b20")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObjectCount>)))
  "Returns full string definition for message of type '<ObjectCount>"
  (cl:format cl:nil "Header header~%int8 count~%Header image_header~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObjectCount)))
  "Returns full string definition for message of type 'ObjectCount"
  (cl:format cl:nil "Header header~%int8 count~%Header image_header~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObjectCount>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image_header))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObjectCount>))
  "Converts a ROS message object to a list"
  (cl:list 'ObjectCount
    (cl:cons ':header (header msg))
    (cl:cons ':count (count msg))
    (cl:cons ':image_header (image_header msg))
))
