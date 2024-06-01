; Auto-generated. Do not edit!


(cl:in-package limo_motion_controller-msg)


;//! \htmlinclude MovementController.msg.html

(cl:defclass <MovementController> (roslisp-msg-protocol:ros-message)
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
    :initform 0.0))
)

(cl:defclass MovementController (<MovementController>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MovementController>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MovementController)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name limo_motion_controller-msg:<MovementController> is deprecated: use limo_motion_controller-msg:MovementController instead.")))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <MovementController>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader limo_motion_controller-msg:speed-val is deprecated.  Use limo_motion_controller-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <MovementController>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader limo_motion_controller-msg:angle-val is deprecated.  Use limo_motion_controller-msg:angle instead.")
  (angle m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <MovementController>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader limo_motion_controller-msg:duration-val is deprecated.  Use limo_motion_controller-msg:duration instead.")
  (duration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MovementController>) ostream)
  "Serializes a message object of type '<MovementController>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MovementController>) istream)
  "Deserializes a message object of type '<MovementController>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MovementController>)))
  "Returns string type for a message object of type '<MovementController>"
  "limo_motion_controller/MovementController")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MovementController)))
  "Returns string type for a message object of type 'MovementController"
  "limo_motion_controller/MovementController")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MovementController>)))
  "Returns md5sum for a message object of type '<MovementController>"
  "abc78f3d8ed307de203cb3588b86e07a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MovementController)))
  "Returns md5sum for a message object of type 'MovementController"
  "abc78f3d8ed307de203cb3588b86e07a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MovementController>)))
  "Returns full string definition for message of type '<MovementController>"
  (cl:format cl:nil "float32 speed~%float32 angle~%float32 duration~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MovementController)))
  "Returns full string definition for message of type 'MovementController"
  (cl:format cl:nil "float32 speed~%float32 angle~%float32 duration~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MovementController>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MovementController>))
  "Converts a ROS message object to a list"
  (cl:list 'MovementController
    (cl:cons ':speed (speed msg))
    (cl:cons ':angle (angle msg))
    (cl:cons ':duration (duration msg))
))
