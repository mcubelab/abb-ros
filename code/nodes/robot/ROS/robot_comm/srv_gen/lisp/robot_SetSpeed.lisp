; Auto-generated. Do not edit!


(cl:in-package robot_comm-srv)


;//! \htmlinclude robot_SetSpeed-request.msg.html

(cl:defclass <robot_SetSpeed-request> (roslisp-msg-protocol:ros-message)
  ((tcp
    :reader tcp
    :initarg :tcp
    :type cl:float
    :initform 0.0)
   (ori
    :reader ori
    :initarg :ori
    :type cl:float
    :initform 0.0))
)

(cl:defclass robot_SetSpeed-request (<robot_SetSpeed-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <robot_SetSpeed-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'robot_SetSpeed-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_comm-srv:<robot_SetSpeed-request> is deprecated: use robot_comm-srv:robot_SetSpeed-request instead.")))

(cl:ensure-generic-function 'tcp-val :lambda-list '(m))
(cl:defmethod tcp-val ((m <robot_SetSpeed-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_comm-srv:tcp-val is deprecated.  Use robot_comm-srv:tcp instead.")
  (tcp m))

(cl:ensure-generic-function 'ori-val :lambda-list '(m))
(cl:defmethod ori-val ((m <robot_SetSpeed-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_comm-srv:ori-val is deprecated.  Use robot_comm-srv:ori instead.")
  (ori m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <robot_SetSpeed-request>) ostream)
  "Serializes a message object of type '<robot_SetSpeed-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'tcp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ori))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <robot_SetSpeed-request>) istream)
  "Deserializes a message object of type '<robot_SetSpeed-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tcp) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ori) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<robot_SetSpeed-request>)))
  "Returns string type for a service object of type '<robot_SetSpeed-request>"
  "robot_comm/robot_SetSpeedRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robot_SetSpeed-request)))
  "Returns string type for a service object of type 'robot_SetSpeed-request"
  "robot_comm/robot_SetSpeedRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<robot_SetSpeed-request>)))
  "Returns md5sum for a message object of type '<robot_SetSpeed-request>"
  "333d89e728aabc050c209d2cd3cd6dc2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'robot_SetSpeed-request)))
  "Returns md5sum for a message object of type 'robot_SetSpeed-request"
  "333d89e728aabc050c209d2cd3cd6dc2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<robot_SetSpeed-request>)))
  "Returns full string definition for message of type '<robot_SetSpeed-request>"
  (cl:format cl:nil "~%~%float64 tcp~%float64 ori~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'robot_SetSpeed-request)))
  "Returns full string definition for message of type 'robot_SetSpeed-request"
  (cl:format cl:nil "~%~%float64 tcp~%float64 ori~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <robot_SetSpeed-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <robot_SetSpeed-request>))
  "Converts a ROS message object to a list"
  (cl:list 'robot_SetSpeed-request
    (cl:cons ':tcp (tcp msg))
    (cl:cons ':ori (ori msg))
))
;//! \htmlinclude robot_SetSpeed-response.msg.html

(cl:defclass <robot_SetSpeed-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass robot_SetSpeed-response (<robot_SetSpeed-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <robot_SetSpeed-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'robot_SetSpeed-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_comm-srv:<robot_SetSpeed-response> is deprecated: use robot_comm-srv:robot_SetSpeed-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <robot_SetSpeed-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_comm-srv:ret-val is deprecated.  Use robot_comm-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <robot_SetSpeed-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_comm-srv:msg-val is deprecated.  Use robot_comm-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <robot_SetSpeed-response>) ostream)
  "Serializes a message object of type '<robot_SetSpeed-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <robot_SetSpeed-response>) istream)
  "Deserializes a message object of type '<robot_SetSpeed-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<robot_SetSpeed-response>)))
  "Returns string type for a service object of type '<robot_SetSpeed-response>"
  "robot_comm/robot_SetSpeedResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robot_SetSpeed-response)))
  "Returns string type for a service object of type 'robot_SetSpeed-response"
  "robot_comm/robot_SetSpeedResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<robot_SetSpeed-response>)))
  "Returns md5sum for a message object of type '<robot_SetSpeed-response>"
  "333d89e728aabc050c209d2cd3cd6dc2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'robot_SetSpeed-response)))
  "Returns md5sum for a message object of type 'robot_SetSpeed-response"
  "333d89e728aabc050c209d2cd3cd6dc2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<robot_SetSpeed-response>)))
  "Returns full string definition for message of type '<robot_SetSpeed-response>"
  (cl:format cl:nil "int64 ret~%string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'robot_SetSpeed-response)))
  "Returns full string definition for message of type 'robot_SetSpeed-response"
  (cl:format cl:nil "int64 ret~%string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <robot_SetSpeed-response>))
  (cl:+ 0
     8
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <robot_SetSpeed-response>))
  "Converts a ROS message object to a list"
  (cl:list 'robot_SetSpeed-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':msg (msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'robot_SetSpeed)))
  'robot_SetSpeed-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'robot_SetSpeed)))
  'robot_SetSpeed-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robot_SetSpeed)))
  "Returns string type for a service object of type '<robot_SetSpeed>"
  "robot_comm/robot_SetSpeed")