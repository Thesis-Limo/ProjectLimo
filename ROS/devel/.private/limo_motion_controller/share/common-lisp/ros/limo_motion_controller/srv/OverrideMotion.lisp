; Auto-generated. Do not edit!


(cl:in-package limo_motion_controller-srv)


;//! \htmlinclude OverrideMotion-request.msg.html

(cl:defclass <OverrideMotion-request> (roslisp-msg-protocol:ros-message)
  ((speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0)
   (angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0)
   (duration
    :reader duration
    :initarg :duration
    :type cl:float
    :initform 0.0)
   (sameSpeedStart
    :reader sameSpeedStart
    :initarg :sameSpeedStart
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass OverrideMotion-request (<OverrideMotion-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OverrideMotion-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OverrideMotion-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name limo_motion_controller-srv:<OverrideMotion-request> is deprecated: use limo_motion_controller-srv:OverrideMotion-request instead.")))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <OverrideMotion-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader limo_motion_controller-srv:speed-val is deprecated.  Use limo_motion_controller-srv:speed instead.")
  (speed m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <OverrideMotion-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader limo_motion_controller-srv:angle-val is deprecated.  Use limo_motion_controller-srv:angle instead.")
  (angle m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <OverrideMotion-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader limo_motion_controller-srv:duration-val is deprecated.  Use limo_motion_controller-srv:duration instead.")
  (duration m))

(cl:ensure-generic-function 'sameSpeedStart-val :lambda-list '(m))
(cl:defmethod sameSpeedStart-val ((m <OverrideMotion-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader limo_motion_controller-srv:sameSpeedStart-val is deprecated.  Use limo_motion_controller-srv:sameSpeedStart instead.")
  (sameSpeedStart m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OverrideMotion-request>) ostream)
  "Serializes a message object of type '<OverrideMotion-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'duration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'sameSpeedStart) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OverrideMotion-request>) istream)
  "Deserializes a message object of type '<OverrideMotion-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'duration) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'sameSpeedStart) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OverrideMotion-request>)))
  "Returns string type for a service object of type '<OverrideMotion-request>"
  "limo_motion_controller/OverrideMotionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OverrideMotion-request)))
  "Returns string type for a service object of type 'OverrideMotion-request"
  "limo_motion_controller/OverrideMotionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OverrideMotion-request>)))
  "Returns md5sum for a message object of type '<OverrideMotion-request>"
  "4955e86101808a9dfce48b4756da7f72")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OverrideMotion-request)))
  "Returns md5sum for a message object of type 'OverrideMotion-request"
  "4955e86101808a9dfce48b4756da7f72")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OverrideMotion-request>)))
  "Returns full string definition for message of type '<OverrideMotion-request>"
  (cl:format cl:nil "float32 speed~%float32 angle~%float32 duration~%bool sameSpeedStart~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OverrideMotion-request)))
  "Returns full string definition for message of type 'OverrideMotion-request"
  (cl:format cl:nil "float32 speed~%float32 angle~%float32 duration~%bool sameSpeedStart~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OverrideMotion-request>))
  (cl:+ 0
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OverrideMotion-request>))
  "Converts a ROS message object to a list"
  (cl:list 'OverrideMotion-request
    (cl:cons ':speed (speed msg))
    (cl:cons ':angle (angle msg))
    (cl:cons ':duration (duration msg))
    (cl:cons ':sameSpeedStart (sameSpeedStart msg))
))
;//! \htmlinclude OverrideMotion-response.msg.html

(cl:defclass <OverrideMotion-response> (roslisp-msg-protocol:ros-message)
  ((alreadyRunning
    :reader alreadyRunning
    :initarg :alreadyRunning
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass OverrideMotion-response (<OverrideMotion-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OverrideMotion-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OverrideMotion-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name limo_motion_controller-srv:<OverrideMotion-response> is deprecated: use limo_motion_controller-srv:OverrideMotion-response instead.")))

(cl:ensure-generic-function 'alreadyRunning-val :lambda-list '(m))
(cl:defmethod alreadyRunning-val ((m <OverrideMotion-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader limo_motion_controller-srv:alreadyRunning-val is deprecated.  Use limo_motion_controller-srv:alreadyRunning instead.")
  (alreadyRunning m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OverrideMotion-response>) ostream)
  "Serializes a message object of type '<OverrideMotion-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'alreadyRunning) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OverrideMotion-response>) istream)
  "Deserializes a message object of type '<OverrideMotion-response>"
    (cl:setf (cl:slot-value msg 'alreadyRunning) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OverrideMotion-response>)))
  "Returns string type for a service object of type '<OverrideMotion-response>"
  "limo_motion_controller/OverrideMotionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OverrideMotion-response)))
  "Returns string type for a service object of type 'OverrideMotion-response"
  "limo_motion_controller/OverrideMotionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OverrideMotion-response>)))
  "Returns md5sum for a message object of type '<OverrideMotion-response>"
  "4955e86101808a9dfce48b4756da7f72")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OverrideMotion-response)))
  "Returns md5sum for a message object of type 'OverrideMotion-response"
  "4955e86101808a9dfce48b4756da7f72")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OverrideMotion-response>)))
  "Returns full string definition for message of type '<OverrideMotion-response>"
  (cl:format cl:nil "bool alreadyRunning~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OverrideMotion-response)))
  "Returns full string definition for message of type 'OverrideMotion-response"
  (cl:format cl:nil "bool alreadyRunning~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OverrideMotion-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OverrideMotion-response>))
  "Converts a ROS message object to a list"
  (cl:list 'OverrideMotion-response
    (cl:cons ':alreadyRunning (alreadyRunning msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'OverrideMotion)))
  'OverrideMotion-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'OverrideMotion)))
  'OverrideMotion-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OverrideMotion)))
  "Returns string type for a service object of type '<OverrideMotion>"
  "limo_motion_controller/OverrideMotion")