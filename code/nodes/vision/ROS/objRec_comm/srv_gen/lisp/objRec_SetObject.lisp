; Auto-generated. Do not edit!


(cl:in-package objRec_comm-srv)


;//! \htmlinclude objRec_SetObject-request.msg.html

(cl:defclass <objRec_SetObject-request> (roslisp-msg-protocol:ros-message)
  ((use_object
    :reader use_object
    :initarg :use_object
    :type cl:boolean
    :initform cl:nil)
   (objNum
    :reader objNum
    :initarg :objNum
    :type cl:integer
    :initform 0))
)

(cl:defclass objRec_SetObject-request (<objRec_SetObject-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <objRec_SetObject-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'objRec_SetObject-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name objRec_comm-srv:<objRec_SetObject-request> is deprecated: use objRec_comm-srv:objRec_SetObject-request instead.")))

(cl:ensure-generic-function 'use_object-val :lambda-list '(m))
(cl:defmethod use_object-val ((m <objRec_SetObject-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:use_object-val is deprecated.  Use objRec_comm-srv:use_object instead.")
  (use_object m))

(cl:ensure-generic-function 'objNum-val :lambda-list '(m))
(cl:defmethod objNum-val ((m <objRec_SetObject-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:objNum-val is deprecated.  Use objRec_comm-srv:objNum instead.")
  (objNum m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <objRec_SetObject-request>) ostream)
  "Serializes a message object of type '<objRec_SetObject-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'use_object) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'objNum)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <objRec_SetObject-request>) istream)
  "Deserializes a message object of type '<objRec_SetObject-request>"
    (cl:setf (cl:slot-value msg 'use_object) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'objNum) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<objRec_SetObject-request>)))
  "Returns string type for a service object of type '<objRec_SetObject-request>"
  "objRec_comm/objRec_SetObjectRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'objRec_SetObject-request)))
  "Returns string type for a service object of type 'objRec_SetObject-request"
  "objRec_comm/objRec_SetObjectRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<objRec_SetObject-request>)))
  "Returns md5sum for a message object of type '<objRec_SetObject-request>"
  "625946597f9f6db8e1d7a0b323024245")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'objRec_SetObject-request)))
  "Returns md5sum for a message object of type 'objRec_SetObject-request"
  "625946597f9f6db8e1d7a0b323024245")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<objRec_SetObject-request>)))
  "Returns full string definition for message of type '<objRec_SetObject-request>"
  (cl:format cl:nil "~%~%~%~%~%bool use_object~%int64 objNum~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'objRec_SetObject-request)))
  "Returns full string definition for message of type 'objRec_SetObject-request"
  (cl:format cl:nil "~%~%~%~%~%bool use_object~%int64 objNum~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <objRec_SetObject-request>))
  (cl:+ 0
     1
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <objRec_SetObject-request>))
  "Converts a ROS message object to a list"
  (cl:list 'objRec_SetObject-request
    (cl:cons ':use_object (use_object msg))
    (cl:cons ':objNum (objNum msg))
))
;//! \htmlinclude objRec_SetObject-response.msg.html

(cl:defclass <objRec_SetObject-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass objRec_SetObject-response (<objRec_SetObject-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <objRec_SetObject-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'objRec_SetObject-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name objRec_comm-srv:<objRec_SetObject-response> is deprecated: use objRec_comm-srv:objRec_SetObject-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <objRec_SetObject-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:ret-val is deprecated.  Use objRec_comm-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <objRec_SetObject-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:msg-val is deprecated.  Use objRec_comm-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <objRec_SetObject-response>) ostream)
  "Serializes a message object of type '<objRec_SetObject-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <objRec_SetObject-response>) istream)
  "Deserializes a message object of type '<objRec_SetObject-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<objRec_SetObject-response>)))
  "Returns string type for a service object of type '<objRec_SetObject-response>"
  "objRec_comm/objRec_SetObjectResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'objRec_SetObject-response)))
  "Returns string type for a service object of type 'objRec_SetObject-response"
  "objRec_comm/objRec_SetObjectResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<objRec_SetObject-response>)))
  "Returns md5sum for a message object of type '<objRec_SetObject-response>"
  "625946597f9f6db8e1d7a0b323024245")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'objRec_SetObject-response)))
  "Returns md5sum for a message object of type 'objRec_SetObject-response"
  "625946597f9f6db8e1d7a0b323024245")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<objRec_SetObject-response>)))
  "Returns full string definition for message of type '<objRec_SetObject-response>"
  (cl:format cl:nil "~%int64 ret~%string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'objRec_SetObject-response)))
  "Returns full string definition for message of type 'objRec_SetObject-response"
  (cl:format cl:nil "~%int64 ret~%string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <objRec_SetObject-response>))
  (cl:+ 0
     8
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <objRec_SetObject-response>))
  "Converts a ROS message object to a list"
  (cl:list 'objRec_SetObject-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':msg (msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'objRec_SetObject)))
  'objRec_SetObject-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'objRec_SetObject)))
  'objRec_SetObject-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'objRec_SetObject)))
  "Returns string type for a service object of type '<objRec_SetObject>"
  "objRec_comm/objRec_SetObject")