; Auto-generated. Do not edit!


(cl:in-package hand_comm-srv)


;//! \htmlinclude hand_SetAngle-request.msg.html

(cl:defclass <hand_SetAngle-request> (roslisp-msg-protocol:ros-message)
  ((angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass hand_SetAngle-request (<hand_SetAngle-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <hand_SetAngle-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'hand_SetAngle-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hand_comm-srv:<hand_SetAngle-request> is deprecated: use hand_comm-srv:hand_SetAngle-request instead.")))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <hand_SetAngle-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hand_comm-srv:angle-val is deprecated.  Use hand_comm-srv:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <hand_SetAngle-request>) ostream)
  "Serializes a message object of type '<hand_SetAngle-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <hand_SetAngle-request>) istream)
  "Deserializes a message object of type '<hand_SetAngle-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<hand_SetAngle-request>)))
  "Returns string type for a service object of type '<hand_SetAngle-request>"
  "hand_comm/hand_SetAngleRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'hand_SetAngle-request)))
  "Returns string type for a service object of type 'hand_SetAngle-request"
  "hand_comm/hand_SetAngleRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<hand_SetAngle-request>)))
  "Returns md5sum for a message object of type '<hand_SetAngle-request>"
  "2af98a0f7367c07e91d2380026087984")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'hand_SetAngle-request)))
  "Returns md5sum for a message object of type 'hand_SetAngle-request"
  "2af98a0f7367c07e91d2380026087984")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<hand_SetAngle-request>)))
  "Returns full string definition for message of type '<hand_SetAngle-request>"
  (cl:format cl:nil "~%float64 angle~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'hand_SetAngle-request)))
  "Returns full string definition for message of type 'hand_SetAngle-request"
  (cl:format cl:nil "~%float64 angle~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <hand_SetAngle-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <hand_SetAngle-request>))
  "Converts a ROS message object to a list"
  (cl:list 'hand_SetAngle-request
    (cl:cons ':angle (angle msg))
))
;//! \htmlinclude hand_SetAngle-response.msg.html

(cl:defclass <hand_SetAngle-response> (roslisp-msg-protocol:ros-message)
  ((ret
    :reader ret
    :initarg :ret
    :type cl:integer
    :initform 0)
   (msg
    :reader msg
    :initarg :msg
    :type cl:string
    :initform ""))
)

(cl:defclass hand_SetAngle-response (<hand_SetAngle-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <hand_SetAngle-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'hand_SetAngle-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hand_comm-srv:<hand_SetAngle-response> is deprecated: use hand_comm-srv:hand_SetAngle-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <hand_SetAngle-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hand_comm-srv:ret-val is deprecated.  Use hand_comm-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <hand_SetAngle-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hand_comm-srv:msg-val is deprecated.  Use hand_comm-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <hand_SetAngle-response>) ostream)
  "Serializes a message object of type '<hand_SetAngle-response>"
  (cl:let* ((signed (cl:slot-value msg 'ret)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'msg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'msg))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <hand_SetAngle-response>) istream)
  "Deserializes a message object of type '<hand_SetAngle-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ret) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'msg) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'msg) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<hand_SetAngle-response>)))
  "Returns string type for a service object of type '<hand_SetAngle-response>"
  "hand_comm/hand_SetAngleResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'hand_SetAngle-response)))
  "Returns string type for a service object of type 'hand_SetAngle-response"
  "hand_comm/hand_SetAngleResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<hand_SetAngle-response>)))
  "Returns md5sum for a message object of type '<hand_SetAngle-response>"
  "2af98a0f7367c07e91d2380026087984")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'hand_SetAngle-response)))
  "Returns md5sum for a message object of type 'hand_SetAngle-response"
  "2af98a0f7367c07e91d2380026087984")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<hand_SetAngle-response>)))
  "Returns full string definition for message of type '<hand_SetAngle-response>"
  (cl:format cl:nil "int64 ret~%string msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'hand_SetAngle-response)))
  "Returns full string definition for message of type 'hand_SetAngle-response"
  (cl:format cl:nil "int64 ret~%string msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <hand_SetAngle-response>))
  (cl:+ 0
     8
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <hand_SetAngle-response>))
  "Converts a ROS message object to a list"
  (cl:list 'hand_SetAngle-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':msg (msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'hand_SetAngle)))
  'hand_SetAngle-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'hand_SetAngle)))
  'hand_SetAngle-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'hand_SetAngle)))
  "Returns string type for a service object of type '<hand_SetAngle>"
  "hand_comm/hand_SetAngle")