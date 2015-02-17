; Auto-generated. Do not edit!


(cl:in-package hand_comm-srv)


;//! \htmlinclude hand_IsMoving-request.msg.html

(cl:defclass <hand_IsMoving-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass hand_IsMoving-request (<hand_IsMoving-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <hand_IsMoving-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'hand_IsMoving-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hand_comm-srv:<hand_IsMoving-request> is deprecated: use hand_comm-srv:hand_IsMoving-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <hand_IsMoving-request>) ostream)
  "Serializes a message object of type '<hand_IsMoving-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <hand_IsMoving-request>) istream)
  "Deserializes a message object of type '<hand_IsMoving-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<hand_IsMoving-request>)))
  "Returns string type for a service object of type '<hand_IsMoving-request>"
  "hand_comm/hand_IsMovingRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'hand_IsMoving-request)))
  "Returns string type for a service object of type 'hand_IsMoving-request"
  "hand_comm/hand_IsMovingRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<hand_IsMoving-request>)))
  "Returns md5sum for a message object of type '<hand_IsMoving-request>"
  "75e70a2e16741096695c5c1c244090d8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'hand_IsMoving-request)))
  "Returns md5sum for a message object of type 'hand_IsMoving-request"
  "75e70a2e16741096695c5c1c244090d8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<hand_IsMoving-request>)))
  "Returns full string definition for message of type '<hand_IsMoving-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'hand_IsMoving-request)))
  "Returns full string definition for message of type 'hand_IsMoving-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <hand_IsMoving-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <hand_IsMoving-request>))
  "Converts a ROS message object to a list"
  (cl:list 'hand_IsMoving-request
))
;//! \htmlinclude hand_IsMoving-response.msg.html

(cl:defclass <hand_IsMoving-response> (roslisp-msg-protocol:ros-message)
  ((moving
    :reader moving
    :initarg :moving
    :type cl:integer
    :initform 0)
   (ret
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

(cl:defclass hand_IsMoving-response (<hand_IsMoving-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <hand_IsMoving-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'hand_IsMoving-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hand_comm-srv:<hand_IsMoving-response> is deprecated: use hand_comm-srv:hand_IsMoving-response instead.")))

(cl:ensure-generic-function 'moving-val :lambda-list '(m))
(cl:defmethod moving-val ((m <hand_IsMoving-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hand_comm-srv:moving-val is deprecated.  Use hand_comm-srv:moving instead.")
  (moving m))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <hand_IsMoving-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hand_comm-srv:ret-val is deprecated.  Use hand_comm-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <hand_IsMoving-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hand_comm-srv:msg-val is deprecated.  Use hand_comm-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <hand_IsMoving-response>) ostream)
  "Serializes a message object of type '<hand_IsMoving-response>"
  (cl:let* ((signed (cl:slot-value msg 'moving)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <hand_IsMoving-response>) istream)
  "Deserializes a message object of type '<hand_IsMoving-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'moving) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<hand_IsMoving-response>)))
  "Returns string type for a service object of type '<hand_IsMoving-response>"
  "hand_comm/hand_IsMovingResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'hand_IsMoving-response)))
  "Returns string type for a service object of type 'hand_IsMoving-response"
  "hand_comm/hand_IsMovingResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<hand_IsMoving-response>)))
  "Returns md5sum for a message object of type '<hand_IsMoving-response>"
  "75e70a2e16741096695c5c1c244090d8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'hand_IsMoving-response)))
  "Returns md5sum for a message object of type 'hand_IsMoving-response"
  "75e70a2e16741096695c5c1c244090d8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<hand_IsMoving-response>)))
  "Returns full string definition for message of type '<hand_IsMoving-response>"
  (cl:format cl:nil "int64 moving~%int64 ret~%string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'hand_IsMoving-response)))
  "Returns full string definition for message of type 'hand_IsMoving-response"
  (cl:format cl:nil "int64 moving~%int64 ret~%string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <hand_IsMoving-response>))
  (cl:+ 0
     8
     8
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <hand_IsMoving-response>))
  "Converts a ROS message object to a list"
  (cl:list 'hand_IsMoving-response
    (cl:cons ':moving (moving msg))
    (cl:cons ':ret (ret msg))
    (cl:cons ':msg (msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'hand_IsMoving)))
  'hand_IsMoving-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'hand_IsMoving)))
  'hand_IsMoving-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'hand_IsMoving)))
  "Returns string type for a service object of type '<hand_IsMoving>"
  "hand_comm/hand_IsMoving")