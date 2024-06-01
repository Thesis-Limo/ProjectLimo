; Auto-generated. Do not edit!


(cl:in-package limo_behaviour_tree-srv)


;//! \htmlinclude TypeObjectTracking-request.msg.html

(cl:defclass <TypeObjectTracking-request> (roslisp-msg-protocol:ros-message)
  ((objectID
    :reader objectID
    :initarg :objectID
    :type cl:integer
    :initform 0))
)

(cl:defclass TypeObjectTracking-request (<TypeObjectTracking-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TypeObjectTracking-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TypeObjectTracking-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name limo_behaviour_tree-srv:<TypeObjectTracking-request> is deprecated: use limo_behaviour_tree-srv:TypeObjectTracking-request instead.")))

(cl:ensure-generic-function 'objectID-val :lambda-list '(m))
(cl:defmethod objectID-val ((m <TypeObjectTracking-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader limo_behaviour_tree-srv:objectID-val is deprecated.  Use limo_behaviour_tree-srv:objectID instead.")
  (objectID m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TypeObjectTracking-request>) ostream)
  "Serializes a message object of type '<TypeObjectTracking-request>"
  (cl:let* ((signed (cl:slot-value msg 'objectID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TypeObjectTracking-request>) istream)
  "Deserializes a message object of type '<TypeObjectTracking-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'objectID) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TypeObjectTracking-request>)))
  "Returns string type for a service object of type '<TypeObjectTracking-request>"
  "limo_behaviour_tree/TypeObjectTrackingRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TypeObjectTracking-request)))
  "Returns string type for a service object of type 'TypeObjectTracking-request"
  "limo_behaviour_tree/TypeObjectTrackingRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TypeObjectTracking-request>)))
  "Returns md5sum for a message object of type '<TypeObjectTracking-request>"
  "0b08c3191adc4004289244e57c20d2f6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TypeObjectTracking-request)))
  "Returns md5sum for a message object of type 'TypeObjectTracking-request"
  "0b08c3191adc4004289244e57c20d2f6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TypeObjectTracking-request>)))
  "Returns full string definition for message of type '<TypeObjectTracking-request>"
  (cl:format cl:nil "int32 objectID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TypeObjectTracking-request)))
  "Returns full string definition for message of type 'TypeObjectTracking-request"
  (cl:format cl:nil "int32 objectID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TypeObjectTracking-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TypeObjectTracking-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TypeObjectTracking-request
    (cl:cons ':objectID (objectID msg))
))
;//! \htmlinclude TypeObjectTracking-response.msg.html

(cl:defclass <TypeObjectTracking-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass TypeObjectTracking-response (<TypeObjectTracking-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TypeObjectTracking-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TypeObjectTracking-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name limo_behaviour_tree-srv:<TypeObjectTracking-response> is deprecated: use limo_behaviour_tree-srv:TypeObjectTracking-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TypeObjectTracking-response>) ostream)
  "Serializes a message object of type '<TypeObjectTracking-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TypeObjectTracking-response>) istream)
  "Deserializes a message object of type '<TypeObjectTracking-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TypeObjectTracking-response>)))
  "Returns string type for a service object of type '<TypeObjectTracking-response>"
  "limo_behaviour_tree/TypeObjectTrackingResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TypeObjectTracking-response)))
  "Returns string type for a service object of type 'TypeObjectTracking-response"
  "limo_behaviour_tree/TypeObjectTrackingResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TypeObjectTracking-response>)))
  "Returns md5sum for a message object of type '<TypeObjectTracking-response>"
  "0b08c3191adc4004289244e57c20d2f6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TypeObjectTracking-response)))
  "Returns md5sum for a message object of type 'TypeObjectTracking-response"
  "0b08c3191adc4004289244e57c20d2f6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TypeObjectTracking-response>)))
  "Returns full string definition for message of type '<TypeObjectTracking-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TypeObjectTracking-response)))
  "Returns full string definition for message of type 'TypeObjectTracking-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TypeObjectTracking-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TypeObjectTracking-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TypeObjectTracking-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TypeObjectTracking)))
  'TypeObjectTracking-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TypeObjectTracking)))
  'TypeObjectTracking-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TypeObjectTracking)))
  "Returns string type for a service object of type '<TypeObjectTracking>"
  "limo_behaviour_tree/TypeObjectTracking")