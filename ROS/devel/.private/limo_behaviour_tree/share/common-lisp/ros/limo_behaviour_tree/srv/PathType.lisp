; Auto-generated. Do not edit!


(cl:in-package limo_behaviour_tree-srv)


;//! \htmlinclude PathType-request.msg.html

(cl:defclass <PathType-request> (roslisp-msg-protocol:ros-message)
  ((pathType
    :reader pathType
    :initarg :pathType
    :type cl:integer
    :initform 0))
)

(cl:defclass PathType-request (<PathType-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PathType-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PathType-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name limo_behaviour_tree-srv:<PathType-request> is deprecated: use limo_behaviour_tree-srv:PathType-request instead.")))

(cl:ensure-generic-function 'pathType-val :lambda-list '(m))
(cl:defmethod pathType-val ((m <PathType-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader limo_behaviour_tree-srv:pathType-val is deprecated.  Use limo_behaviour_tree-srv:pathType instead.")
  (pathType m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PathType-request>) ostream)
  "Serializes a message object of type '<PathType-request>"
  (cl:let* ((signed (cl:slot-value msg 'pathType)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PathType-request>) istream)
  "Deserializes a message object of type '<PathType-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pathType) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PathType-request>)))
  "Returns string type for a service object of type '<PathType-request>"
  "limo_behaviour_tree/PathTypeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PathType-request)))
  "Returns string type for a service object of type 'PathType-request"
  "limo_behaviour_tree/PathTypeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PathType-request>)))
  "Returns md5sum for a message object of type '<PathType-request>"
  "11e5c53def65876fc1708cc2295e05a4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PathType-request)))
  "Returns md5sum for a message object of type 'PathType-request"
  "11e5c53def65876fc1708cc2295e05a4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PathType-request>)))
  "Returns full string definition for message of type '<PathType-request>"
  (cl:format cl:nil "int32 pathType~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PathType-request)))
  "Returns full string definition for message of type 'PathType-request"
  (cl:format cl:nil "int32 pathType~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PathType-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PathType-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PathType-request
    (cl:cons ':pathType (pathType msg))
))
;//! \htmlinclude PathType-response.msg.html

(cl:defclass <PathType-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass PathType-response (<PathType-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PathType-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PathType-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name limo_behaviour_tree-srv:<PathType-response> is deprecated: use limo_behaviour_tree-srv:PathType-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PathType-response>) ostream)
  "Serializes a message object of type '<PathType-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PathType-response>) istream)
  "Deserializes a message object of type '<PathType-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PathType-response>)))
  "Returns string type for a service object of type '<PathType-response>"
  "limo_behaviour_tree/PathTypeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PathType-response)))
  "Returns string type for a service object of type 'PathType-response"
  "limo_behaviour_tree/PathTypeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PathType-response>)))
  "Returns md5sum for a message object of type '<PathType-response>"
  "11e5c53def65876fc1708cc2295e05a4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PathType-response)))
  "Returns md5sum for a message object of type 'PathType-response"
  "11e5c53def65876fc1708cc2295e05a4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PathType-response>)))
  "Returns full string definition for message of type '<PathType-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PathType-response)))
  "Returns full string definition for message of type 'PathType-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PathType-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PathType-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PathType-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PathType)))
  'PathType-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PathType)))
  'PathType-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PathType)))
  "Returns string type for a service object of type '<PathType>"
  "limo_behaviour_tree/PathType")