; Auto-generated. Do not edit!


(cl:in-package limo_motion_controller-msg)


;//! \htmlinclude MotionPlan.msg.html

(cl:defclass <MotionPlan> (roslisp-msg-protocol:ros-message)
  ((sequence
    :reader sequence
    :initarg :sequence
    :type (cl:vector limo_motion_controller-msg:MovementController)
   :initform (cl:make-array 0 :element-type 'limo_motion_controller-msg:MovementController :initial-element (cl:make-instance 'limo_motion_controller-msg:MovementController))))
)

(cl:defclass MotionPlan (<MotionPlan>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotionPlan>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotionPlan)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name limo_motion_controller-msg:<MotionPlan> is deprecated: use limo_motion_controller-msg:MotionPlan instead.")))

(cl:ensure-generic-function 'sequence-val :lambda-list '(m))
(cl:defmethod sequence-val ((m <MotionPlan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader limo_motion_controller-msg:sequence-val is deprecated.  Use limo_motion_controller-msg:sequence instead.")
  (sequence m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotionPlan>) ostream)
  "Serializes a message object of type '<MotionPlan>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'sequence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'sequence))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotionPlan>) istream)
  "Deserializes a message object of type '<MotionPlan>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'sequence) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'sequence)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'limo_motion_controller-msg:MovementController))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotionPlan>)))
  "Returns string type for a message object of type '<MotionPlan>"
  "limo_motion_controller/MotionPlan")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotionPlan)))
  "Returns string type for a message object of type 'MotionPlan"
  "limo_motion_controller/MotionPlan")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotionPlan>)))
  "Returns md5sum for a message object of type '<MotionPlan>"
  "862f364494b8552b6422445d2907f57e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotionPlan)))
  "Returns md5sum for a message object of type 'MotionPlan"
  "862f364494b8552b6422445d2907f57e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotionPlan>)))
  "Returns full string definition for message of type '<MotionPlan>"
  (cl:format cl:nil "limo_motion_controller/MovementController[] sequence~%================================================================================~%MSG: limo_motion_controller/MovementController~%float32 speed~%float32 angle~%float32 duration~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotionPlan)))
  "Returns full string definition for message of type 'MotionPlan"
  (cl:format cl:nil "limo_motion_controller/MovementController[] sequence~%================================================================================~%MSG: limo_motion_controller/MovementController~%float32 speed~%float32 angle~%float32 duration~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotionPlan>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'sequence) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotionPlan>))
  "Converts a ROS message object to a list"
  (cl:list 'MotionPlan
    (cl:cons ':sequence (sequence msg))
))
