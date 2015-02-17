; Auto-generated. Do not edit!


(cl:in-package hand_comm-srv)


;//! \htmlinclude hand_SetForce-request.msg.html

(cl:defclass <hand_SetForce-request> (roslisp-msg-protocol:ros-message)
  ((force
    :reader force
    :initarg :force
    :type cl:float
    :initform 0.0))
)

(cl:defclass hand_SetForce-request (<hand_SetForce-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <hand_SetForce-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'hand_SetForce-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hand_comm-srv:<hand_SetForce-request> is deprecated: use hand_comm-srv:hand_SetForce-request instead.")))

(cl:ensure-generic-function 'force-val :lambda-list '(m))
(cl:defmethod force-val ((m <hand_SetForce-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hand_comm-srv:force-val is deprecated.  Use hand_comm-srv:force instead.")
  (force m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <hand_SetForce-request>) ostream)
  "Serializes a message object of type '<hand_SetForce-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'force))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <hand_SetForce-request>) istream)
  "Deserializes a message object of type '<hand_SetForce-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'force) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<hand_SetForce-request>)))
  "Returns string type for a service object of type '<hand_SetForce-request>"
  "hand_comm/hand_SetForceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'hand_SetForce-request)))
  "Returns string type for a service object of type 'hand_SetForce-request"
  "hand_comm/hand_SetForceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<hand_SetForce-request>)))
  "Returns md5sum for a message object of type '<hand_SetForce-request>"
  "5acf3310deec4ac406314c0b5d8f4e2b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'hand_SetForce-request)))
  "Returns md5sum for a message object of type 'hand_SetForce-request"
  "5acf3310deec4ac406314c0b5d8f4e2b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<hand_SetForce-request>)))
  "Returns full string definition for message of type '<hand_SetForce-request>"
  (cl:format cl:nil "~%float64 force~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'hand_SetForce-request)))
  "Returns full string definition for message of type 'hand_SetForce-request"
  (cl:format cl:nil "~%float64 force~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <hand_SetForce-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <hand_SetForce-request>))
  "Converts a ROS message object to a list"
  (cl:list 'hand_SetForce-request
    (cl:cons ':force (force msg))
))
;//! \htmlinclude hand_SetForce-response.msg.html

(cl:defclass <hand_SetForce-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass hand_SetForce-response (<hand_SetForce-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <hand_SetForce-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'hand_SetForce-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hand_comm-srv:<hand_SetForce-response> is deprecated: use hand_comm-srv:hand_SetForce-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <hand_SetForce-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hand_comm-srv:ret-val is deprecated.  Use hand_comm-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <hand_SetForce-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hand_comm-srv:msg-val is deprecated.  Use hand_comm-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <hand_SetForce-response>) ostream)
  "Serializes a message object of type '<hand_SetForce-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <hand_SetForce-response>) istream)
  "Deserializes a message object of type '<hand_SetForce-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<hand_SetForce-response>)))
  "Returns string type for a service object of type '<hand_SetForce-response>"
  "hand_comm/hand_SetForceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'hand_SetForce-response)))
  "Returns string type for a service object of type 'hand_SetForce-response"
  "hand_comm/hand_SetForceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<hand_SetForce-response>)))
  "Returns md5sum for a message object of type '<hand_SetForce-response>"
  "5acf3310deec4ac406314c0b5d8f4e2b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'hand_SetForce-response)))
  "Returns md5sum for a message object of type 'hand_SetForce-response"
  "5acf3310deec4ac406314c0b5d8f4e2b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<hand_SetForce-response>)))
  "Returns full string definition for message of type '<hand_SetForce-response>"
  (cl:format cl:nil "int64 ret~%string msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'hand_SetForce-response)))
  "Returns full string definition for message of type 'hand_SetForce-response"
  (cl:format cl:nil "int64 ret~%string msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <hand_SetForce-response>))
  (cl:+ 0
     8
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <hand_SetForce-response>))
  "Converts a ROS message object to a list"
  (cl:list 'hand_SetForce-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':msg (msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'hand_SetForce)))
  'hand_SetForce-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'hand_SetForce)))
  'hand_SetForce-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'hand_SetForce)))
  "Returns string type for a service object of type '<hand_SetForce>"
  "hand_comm/hand_SetForce")