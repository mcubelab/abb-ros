; Auto-generated. Do not edit!


(cl:in-package robot_comm-srv)


;//! \htmlinclude robot_SetComm-request.msg.html

(cl:defclass <robot_SetComm-request> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:integer
    :initform 0))
)

(cl:defclass robot_SetComm-request (<robot_SetComm-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <robot_SetComm-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'robot_SetComm-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_comm-srv:<robot_SetComm-request> is deprecated: use robot_comm-srv:robot_SetComm-request instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <robot_SetComm-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_comm-srv:mode-val is deprecated.  Use robot_comm-srv:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <robot_SetComm-request>) ostream)
  "Serializes a message object of type '<robot_SetComm-request>"
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <robot_SetComm-request>) istream)
  "Deserializes a message object of type '<robot_SetComm-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<robot_SetComm-request>)))
  "Returns string type for a service object of type '<robot_SetComm-request>"
  "robot_comm/robot_SetCommRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robot_SetComm-request)))
  "Returns string type for a service object of type 'robot_SetComm-request"
  "robot_comm/robot_SetCommRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<robot_SetComm-request>)))
  "Returns md5sum for a message object of type '<robot_SetComm-request>"
  "9a8139bbede588258d8a8cdc808e7b88")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'robot_SetComm-request)))
  "Returns md5sum for a message object of type 'robot_SetComm-request"
  "9a8139bbede588258d8a8cdc808e7b88")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<robot_SetComm-request>)))
  "Returns full string definition for message of type '<robot_SetComm-request>"
  (cl:format cl:nil "~%~%int64 mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'robot_SetComm-request)))
  "Returns full string definition for message of type 'robot_SetComm-request"
  (cl:format cl:nil "~%~%int64 mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <robot_SetComm-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <robot_SetComm-request>))
  "Converts a ROS message object to a list"
  (cl:list 'robot_SetComm-request
    (cl:cons ':mode (mode msg))
))
;//! \htmlinclude robot_SetComm-response.msg.html

(cl:defclass <robot_SetComm-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass robot_SetComm-response (<robot_SetComm-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <robot_SetComm-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'robot_SetComm-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_comm-srv:<robot_SetComm-response> is deprecated: use robot_comm-srv:robot_SetComm-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <robot_SetComm-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_comm-srv:ret-val is deprecated.  Use robot_comm-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <robot_SetComm-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_comm-srv:msg-val is deprecated.  Use robot_comm-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <robot_SetComm-response>) ostream)
  "Serializes a message object of type '<robot_SetComm-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <robot_SetComm-response>) istream)
  "Deserializes a message object of type '<robot_SetComm-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<robot_SetComm-response>)))
  "Returns string type for a service object of type '<robot_SetComm-response>"
  "robot_comm/robot_SetCommResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robot_SetComm-response)))
  "Returns string type for a service object of type 'robot_SetComm-response"
  "robot_comm/robot_SetCommResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<robot_SetComm-response>)))
  "Returns md5sum for a message object of type '<robot_SetComm-response>"
  "9a8139bbede588258d8a8cdc808e7b88")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'robot_SetComm-response)))
  "Returns md5sum for a message object of type 'robot_SetComm-response"
  "9a8139bbede588258d8a8cdc808e7b88")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<robot_SetComm-response>)))
  "Returns full string definition for message of type '<robot_SetComm-response>"
  (cl:format cl:nil "int64 ret~%string msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'robot_SetComm-response)))
  "Returns full string definition for message of type 'robot_SetComm-response"
  (cl:format cl:nil "int64 ret~%string msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <robot_SetComm-response>))
  (cl:+ 0
     8
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <robot_SetComm-response>))
  "Converts a ROS message object to a list"
  (cl:list 'robot_SetComm-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':msg (msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'robot_SetComm)))
  'robot_SetComm-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'robot_SetComm)))
  'robot_SetComm-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robot_SetComm)))
  "Returns string type for a service object of type '<robot_SetComm>"
  "robot_comm/robot_SetComm")