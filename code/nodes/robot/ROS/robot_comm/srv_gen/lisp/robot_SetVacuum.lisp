; Auto-generated. Do not edit!


(cl:in-package robot_comm-srv)


;//! \htmlinclude robot_SetVacuum-request.msg.html

(cl:defclass <robot_SetVacuum-request> (roslisp-msg-protocol:ros-message)
  ((vacuum
    :reader vacuum
    :initarg :vacuum
    :type cl:integer
    :initform 0))
)

(cl:defclass robot_SetVacuum-request (<robot_SetVacuum-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <robot_SetVacuum-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'robot_SetVacuum-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_comm-srv:<robot_SetVacuum-request> is deprecated: use robot_comm-srv:robot_SetVacuum-request instead.")))

(cl:ensure-generic-function 'vacuum-val :lambda-list '(m))
(cl:defmethod vacuum-val ((m <robot_SetVacuum-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_comm-srv:vacuum-val is deprecated.  Use robot_comm-srv:vacuum instead.")
  (vacuum m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <robot_SetVacuum-request>) ostream)
  "Serializes a message object of type '<robot_SetVacuum-request>"
  (cl:let* ((signed (cl:slot-value msg 'vacuum)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <robot_SetVacuum-request>) istream)
  "Deserializes a message object of type '<robot_SetVacuum-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'vacuum) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<robot_SetVacuum-request>)))
  "Returns string type for a service object of type '<robot_SetVacuum-request>"
  "robot_comm/robot_SetVacuumRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robot_SetVacuum-request)))
  "Returns string type for a service object of type 'robot_SetVacuum-request"
  "robot_comm/robot_SetVacuumRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<robot_SetVacuum-request>)))
  "Returns md5sum for a message object of type '<robot_SetVacuum-request>"
  "bc2b2a4e5d5d788edc958a62cf84dc1f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'robot_SetVacuum-request)))
  "Returns md5sum for a message object of type 'robot_SetVacuum-request"
  "bc2b2a4e5d5d788edc958a62cf84dc1f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<robot_SetVacuum-request>)))
  "Returns full string definition for message of type '<robot_SetVacuum-request>"
  (cl:format cl:nil "~%~%int64 vacuum~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'robot_SetVacuum-request)))
  "Returns full string definition for message of type 'robot_SetVacuum-request"
  (cl:format cl:nil "~%~%int64 vacuum~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <robot_SetVacuum-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <robot_SetVacuum-request>))
  "Converts a ROS message object to a list"
  (cl:list 'robot_SetVacuum-request
    (cl:cons ':vacuum (vacuum msg))
))
;//! \htmlinclude robot_SetVacuum-response.msg.html

(cl:defclass <robot_SetVacuum-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass robot_SetVacuum-response (<robot_SetVacuum-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <robot_SetVacuum-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'robot_SetVacuum-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_comm-srv:<robot_SetVacuum-response> is deprecated: use robot_comm-srv:robot_SetVacuum-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <robot_SetVacuum-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_comm-srv:ret-val is deprecated.  Use robot_comm-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <robot_SetVacuum-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_comm-srv:msg-val is deprecated.  Use robot_comm-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <robot_SetVacuum-response>) ostream)
  "Serializes a message object of type '<robot_SetVacuum-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <robot_SetVacuum-response>) istream)
  "Deserializes a message object of type '<robot_SetVacuum-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<robot_SetVacuum-response>)))
  "Returns string type for a service object of type '<robot_SetVacuum-response>"
  "robot_comm/robot_SetVacuumResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robot_SetVacuum-response)))
  "Returns string type for a service object of type 'robot_SetVacuum-response"
  "robot_comm/robot_SetVacuumResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<robot_SetVacuum-response>)))
  "Returns md5sum for a message object of type '<robot_SetVacuum-response>"
  "bc2b2a4e5d5d788edc958a62cf84dc1f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'robot_SetVacuum-response)))
  "Returns md5sum for a message object of type 'robot_SetVacuum-response"
  "bc2b2a4e5d5d788edc958a62cf84dc1f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<robot_SetVacuum-response>)))
  "Returns full string definition for message of type '<robot_SetVacuum-response>"
  (cl:format cl:nil "int64 ret~%string msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'robot_SetVacuum-response)))
  "Returns full string definition for message of type 'robot_SetVacuum-response"
  (cl:format cl:nil "int64 ret~%string msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <robot_SetVacuum-response>))
  (cl:+ 0
     8
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <robot_SetVacuum-response>))
  "Converts a ROS message object to a list"
  (cl:list 'robot_SetVacuum-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':msg (msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'robot_SetVacuum)))
  'robot_SetVacuum-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'robot_SetVacuum)))
  'robot_SetVacuum-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robot_SetVacuum)))
  "Returns string type for a service object of type '<robot_SetVacuum>"
  "robot_comm/robot_SetVacuum")