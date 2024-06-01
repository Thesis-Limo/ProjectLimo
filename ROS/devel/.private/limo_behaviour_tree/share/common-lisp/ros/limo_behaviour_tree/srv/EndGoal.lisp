; Auto-generated. Do not edit!


(cl:in-package limo_behaviour_tree-srv)


;//! \htmlinclude EndGoal-request.msg.html

(cl:defclass <EndGoal-request> (roslisp-msg-protocol:ros-message)
  ((goalPos
    :reader goalPos
    :initarg :goalPos
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass EndGoal-request (<EndGoal-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EndGoal-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EndGoal-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name limo_behaviour_tree-srv:<EndGoal-request> is deprecated: use limo_behaviour_tree-srv:EndGoal-request instead.")))

(cl:ensure-generic-function 'goalPos-val :lambda-list '(m))
(cl:defmethod goalPos-val ((m <EndGoal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader limo_behaviour_tree-srv:goalPos-val is deprecated.  Use limo_behaviour_tree-srv:goalPos instead.")
  (goalPos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EndGoal-request>) ostream)
  "Serializes a message object of type '<EndGoal-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goalPos) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EndGoal-request>) istream)
  "Deserializes a message object of type '<EndGoal-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goalPos) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EndGoal-request>)))
  "Returns string type for a service object of type '<EndGoal-request>"
  "limo_behaviour_tree/EndGoalRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EndGoal-request)))
  "Returns string type for a service object of type 'EndGoal-request"
  "limo_behaviour_tree/EndGoalRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EndGoal-request>)))
  "Returns md5sum for a message object of type '<EndGoal-request>"
  "f43f04c866179f13e0ff9145477de5ab")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EndGoal-request)))
  "Returns md5sum for a message object of type 'EndGoal-request"
  "f43f04c866179f13e0ff9145477de5ab")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EndGoal-request>)))
  "Returns full string definition for message of type '<EndGoal-request>"
  (cl:format cl:nil "geometry_msgs/Point goalPos~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EndGoal-request)))
  "Returns full string definition for message of type 'EndGoal-request"
  (cl:format cl:nil "geometry_msgs/Point goalPos~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EndGoal-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goalPos))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EndGoal-request>))
  "Converts a ROS message object to a list"
  (cl:list 'EndGoal-request
    (cl:cons ':goalPos (goalPos msg))
))
;//! \htmlinclude EndGoal-response.msg.html

(cl:defclass <EndGoal-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass EndGoal-response (<EndGoal-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EndGoal-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EndGoal-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name limo_behaviour_tree-srv:<EndGoal-response> is deprecated: use limo_behaviour_tree-srv:EndGoal-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EndGoal-response>) ostream)
  "Serializes a message object of type '<EndGoal-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EndGoal-response>) istream)
  "Deserializes a message object of type '<EndGoal-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EndGoal-response>)))
  "Returns string type for a service object of type '<EndGoal-response>"
  "limo_behaviour_tree/EndGoalResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EndGoal-response)))
  "Returns string type for a service object of type 'EndGoal-response"
  "limo_behaviour_tree/EndGoalResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EndGoal-response>)))
  "Returns md5sum for a message object of type '<EndGoal-response>"
  "f43f04c866179f13e0ff9145477de5ab")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EndGoal-response)))
  "Returns md5sum for a message object of type 'EndGoal-response"
  "f43f04c866179f13e0ff9145477de5ab")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EndGoal-response>)))
  "Returns full string definition for message of type '<EndGoal-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EndGoal-response)))
  "Returns full string definition for message of type 'EndGoal-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EndGoal-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EndGoal-response>))
  "Converts a ROS message object to a list"
  (cl:list 'EndGoal-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'EndGoal)))
  'EndGoal-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'EndGoal)))
  'EndGoal-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EndGoal)))
  "Returns string type for a service object of type '<EndGoal>"
  "limo_behaviour_tree/EndGoal")