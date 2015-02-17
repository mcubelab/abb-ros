; Auto-generated. Do not edit!


(cl:in-package robot_comm-srv)


;//! \htmlinclude robot_SpecialCommand-request.msg.html

(cl:defclass <robot_SpecialCommand-request> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:integer
    :initform 0)
   (param1
    :reader param1
    :initarg :param1
    :type cl:float
    :initform 0.0)
   (param2
    :reader param2
    :initarg :param2
    :type cl:float
    :initform 0.0)
   (param3
    :reader param3
    :initarg :param3
    :type cl:float
    :initform 0.0)
   (param4
    :reader param4
    :initarg :param4
    :type cl:float
    :initform 0.0)
   (param5
    :reader param5
    :initarg :param5
    :type cl:float
    :initform 0.0))
)

(cl:defclass robot_SpecialCommand-request (<robot_SpecialCommand-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <robot_SpecialCommand-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'robot_SpecialCommand-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_comm-srv:<robot_SpecialCommand-request> is deprecated: use robot_comm-srv:robot_SpecialCommand-request instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <robot_SpecialCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_comm-srv:command-val is deprecated.  Use robot_comm-srv:command instead.")
  (command m))

(cl:ensure-generic-function 'param1-val :lambda-list '(m))
(cl:defmethod param1-val ((m <robot_SpecialCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_comm-srv:param1-val is deprecated.  Use robot_comm-srv:param1 instead.")
  (param1 m))

(cl:ensure-generic-function 'param2-val :lambda-list '(m))
(cl:defmethod param2-val ((m <robot_SpecialCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_comm-srv:param2-val is deprecated.  Use robot_comm-srv:param2 instead.")
  (param2 m))

(cl:ensure-generic-function 'param3-val :lambda-list '(m))
(cl:defmethod param3-val ((m <robot_SpecialCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_comm-srv:param3-val is deprecated.  Use robot_comm-srv:param3 instead.")
  (param3 m))

(cl:ensure-generic-function 'param4-val :lambda-list '(m))
(cl:defmethod param4-val ((m <robot_SpecialCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_comm-srv:param4-val is deprecated.  Use robot_comm-srv:param4 instead.")
  (param4 m))

(cl:ensure-generic-function 'param5-val :lambda-list '(m))
(cl:defmethod param5-val ((m <robot_SpecialCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_comm-srv:param5-val is deprecated.  Use robot_comm-srv:param5 instead.")
  (param5 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <robot_SpecialCommand-request>) ostream)
  "Serializes a message object of type '<robot_SpecialCommand-request>"
  (cl:let* ((signed (cl:slot-value msg 'command)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'param1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'param2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'param3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'param4))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'param5))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <robot_SpecialCommand-request>) istream)
  "Deserializes a message object of type '<robot_SpecialCommand-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'command) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'param1) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'param2) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'param3) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'param4) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'param5) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<robot_SpecialCommand-request>)))
  "Returns string type for a service object of type '<robot_SpecialCommand-request>"
  "robot_comm/robot_SpecialCommandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robot_SpecialCommand-request)))
  "Returns string type for a service object of type 'robot_SpecialCommand-request"
  "robot_comm/robot_SpecialCommandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<robot_SpecialCommand-request>)))
  "Returns md5sum for a message object of type '<robot_SpecialCommand-request>"
  "dd470c01f667d001d212037b2fe2c918")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'robot_SpecialCommand-request)))
  "Returns md5sum for a message object of type 'robot_SpecialCommand-request"
  "dd470c01f667d001d212037b2fe2c918")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<robot_SpecialCommand-request>)))
  "Returns full string definition for message of type '<robot_SpecialCommand-request>"
  (cl:format cl:nil "~%~%int64 command~%float64 param1~%float64 param2~%float64 param3~%float64 param4~%float64 param5~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'robot_SpecialCommand-request)))
  "Returns full string definition for message of type 'robot_SpecialCommand-request"
  (cl:format cl:nil "~%~%int64 command~%float64 param1~%float64 param2~%float64 param3~%float64 param4~%float64 param5~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <robot_SpecialCommand-request>))
  (cl:+ 0
     8
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <robot_SpecialCommand-request>))
  "Converts a ROS message object to a list"
  (cl:list 'robot_SpecialCommand-request
    (cl:cons ':command (command msg))
    (cl:cons ':param1 (param1 msg))
    (cl:cons ':param2 (param2 msg))
    (cl:cons ':param3 (param3 msg))
    (cl:cons ':param4 (param4 msg))
    (cl:cons ':param5 (param5 msg))
))
;//! \htmlinclude robot_SpecialCommand-response.msg.html

(cl:defclass <robot_SpecialCommand-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass robot_SpecialCommand-response (<robot_SpecialCommand-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <robot_SpecialCommand-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'robot_SpecialCommand-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_comm-srv:<robot_SpecialCommand-response> is deprecated: use robot_comm-srv:robot_SpecialCommand-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <robot_SpecialCommand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_comm-srv:ret-val is deprecated.  Use robot_comm-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <robot_SpecialCommand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_comm-srv:msg-val is deprecated.  Use robot_comm-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <robot_SpecialCommand-response>) ostream)
  "Serializes a message object of type '<robot_SpecialCommand-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <robot_SpecialCommand-response>) istream)
  "Deserializes a message object of type '<robot_SpecialCommand-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<robot_SpecialCommand-response>)))
  "Returns string type for a service object of type '<robot_SpecialCommand-response>"
  "robot_comm/robot_SpecialCommandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robot_SpecialCommand-response)))
  "Returns string type for a service object of type 'robot_SpecialCommand-response"
  "robot_comm/robot_SpecialCommandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<robot_SpecialCommand-response>)))
  "Returns md5sum for a message object of type '<robot_SpecialCommand-response>"
  "dd470c01f667d001d212037b2fe2c918")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'robot_SpecialCommand-response)))
  "Returns md5sum for a message object of type 'robot_SpecialCommand-response"
  "dd470c01f667d001d212037b2fe2c918")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<robot_SpecialCommand-response>)))
  "Returns full string definition for message of type '<robot_SpecialCommand-response>"
  (cl:format cl:nil "int64 ret~%string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'robot_SpecialCommand-response)))
  "Returns full string definition for message of type 'robot_SpecialCommand-response"
  (cl:format cl:nil "int64 ret~%string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <robot_SpecialCommand-response>))
  (cl:+ 0
     8
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <robot_SpecialCommand-response>))
  "Converts a ROS message object to a list"
  (cl:list 'robot_SpecialCommand-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':msg (msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'robot_SpecialCommand)))
  'robot_SpecialCommand-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'robot_SpecialCommand)))
  'robot_SpecialCommand-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robot_SpecialCommand)))
  "Returns string type for a service object of type '<robot_SpecialCommand>"
  "robot_comm/robot_SpecialCommand")