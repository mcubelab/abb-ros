; Auto-generated. Do not edit!


(cl:in-package hand_comm-srv)


;//! \htmlinclude hand_WaitRest-request.msg.html

(cl:defclass <hand_WaitRest-request> (roslisp-msg-protocol:ros-message)
  ((delay
    :reader delay
    :initarg :delay
    :type cl:float
    :initform 0.0))
)

(cl:defclass hand_WaitRest-request (<hand_WaitRest-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <hand_WaitRest-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'hand_WaitRest-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hand_comm-srv:<hand_WaitRest-request> is deprecated: use hand_comm-srv:hand_WaitRest-request instead.")))

(cl:ensure-generic-function 'delay-val :lambda-list '(m))
(cl:defmethod delay-val ((m <hand_WaitRest-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hand_comm-srv:delay-val is deprecated.  Use hand_comm-srv:delay instead.")
  (delay m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <hand_WaitRest-request>) ostream)
  "Serializes a message object of type '<hand_WaitRest-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'delay))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <hand_WaitRest-request>) istream)
  "Deserializes a message object of type '<hand_WaitRest-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'delay) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<hand_WaitRest-request>)))
  "Returns string type for a service object of type '<hand_WaitRest-request>"
  "hand_comm/hand_WaitRestRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'hand_WaitRest-request)))
  "Returns string type for a service object of type 'hand_WaitRest-request"
  "hand_comm/hand_WaitRestRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<hand_WaitRest-request>)))
  "Returns md5sum for a message object of type '<hand_WaitRest-request>"
  "db3a0abc5a7b38d65070db42a856d203")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'hand_WaitRest-request)))
  "Returns md5sum for a message object of type 'hand_WaitRest-request"
  "db3a0abc5a7b38d65070db42a856d203")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<hand_WaitRest-request>)))
  "Returns full string definition for message of type '<hand_WaitRest-request>"
  (cl:format cl:nil "~%float64 delay~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'hand_WaitRest-request)))
  "Returns full string definition for message of type 'hand_WaitRest-request"
  (cl:format cl:nil "~%float64 delay~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <hand_WaitRest-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <hand_WaitRest-request>))
  "Converts a ROS message object to a list"
  (cl:list 'hand_WaitRest-request
    (cl:cons ':delay (delay msg))
))
;//! \htmlinclude hand_WaitRest-response.msg.html

(cl:defclass <hand_WaitRest-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass hand_WaitRest-response (<hand_WaitRest-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <hand_WaitRest-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'hand_WaitRest-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hand_comm-srv:<hand_WaitRest-response> is deprecated: use hand_comm-srv:hand_WaitRest-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <hand_WaitRest-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hand_comm-srv:ret-val is deprecated.  Use hand_comm-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <hand_WaitRest-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hand_comm-srv:msg-val is deprecated.  Use hand_comm-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <hand_WaitRest-response>) ostream)
  "Serializes a message object of type '<hand_WaitRest-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <hand_WaitRest-response>) istream)
  "Deserializes a message object of type '<hand_WaitRest-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<hand_WaitRest-response>)))
  "Returns string type for a service object of type '<hand_WaitRest-response>"
  "hand_comm/hand_WaitRestResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'hand_WaitRest-response)))
  "Returns string type for a service object of type 'hand_WaitRest-response"
  "hand_comm/hand_WaitRestResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<hand_WaitRest-response>)))
  "Returns md5sum for a message object of type '<hand_WaitRest-response>"
  "db3a0abc5a7b38d65070db42a856d203")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'hand_WaitRest-response)))
  "Returns md5sum for a message object of type 'hand_WaitRest-response"
  "db3a0abc5a7b38d65070db42a856d203")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<hand_WaitRest-response>)))
  "Returns full string definition for message of type '<hand_WaitRest-response>"
  (cl:format cl:nil "int64 ret~%string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'hand_WaitRest-response)))
  "Returns full string definition for message of type 'hand_WaitRest-response"
  (cl:format cl:nil "int64 ret~%string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <hand_WaitRest-response>))
  (cl:+ 0
     8
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <hand_WaitRest-response>))
  "Converts a ROS message object to a list"
  (cl:list 'hand_WaitRest-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':msg (msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'hand_WaitRest)))
  'hand_WaitRest-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'hand_WaitRest)))
  'hand_WaitRest-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'hand_WaitRest)))
  "Returns string type for a service object of type '<hand_WaitRest>"
  "hand_comm/hand_WaitRest")