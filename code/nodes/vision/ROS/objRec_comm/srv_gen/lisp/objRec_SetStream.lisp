; Auto-generated. Do not edit!


(cl:in-package objRec_comm-srv)


;//! \htmlinclude objRec_SetStream-request.msg.html

(cl:defclass <objRec_SetStream-request> (roslisp-msg-protocol:ros-message)
  ((streaming
    :reader streaming
    :initarg :streaming
    :type cl:boolean
    :initform cl:nil)
   (cameraName
    :reader cameraName
    :initarg :cameraName
    :type cl:string
    :initform ""))
)

(cl:defclass objRec_SetStream-request (<objRec_SetStream-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <objRec_SetStream-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'objRec_SetStream-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name objRec_comm-srv:<objRec_SetStream-request> is deprecated: use objRec_comm-srv:objRec_SetStream-request instead.")))

(cl:ensure-generic-function 'streaming-val :lambda-list '(m))
(cl:defmethod streaming-val ((m <objRec_SetStream-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:streaming-val is deprecated.  Use objRec_comm-srv:streaming instead.")
  (streaming m))

(cl:ensure-generic-function 'cameraName-val :lambda-list '(m))
(cl:defmethod cameraName-val ((m <objRec_SetStream-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:cameraName-val is deprecated.  Use objRec_comm-srv:cameraName instead.")
  (cameraName m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <objRec_SetStream-request>) ostream)
  "Serializes a message object of type '<objRec_SetStream-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'streaming) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cameraName))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cameraName))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <objRec_SetStream-request>) istream)
  "Deserializes a message object of type '<objRec_SetStream-request>"
    (cl:setf (cl:slot-value msg 'streaming) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cameraName) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'cameraName) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<objRec_SetStream-request>)))
  "Returns string type for a service object of type '<objRec_SetStream-request>"
  "objRec_comm/objRec_SetStreamRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'objRec_SetStream-request)))
  "Returns string type for a service object of type 'objRec_SetStream-request"
  "objRec_comm/objRec_SetStreamRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<objRec_SetStream-request>)))
  "Returns md5sum for a message object of type '<objRec_SetStream-request>"
  "d91a7e76b8093c49a72a190a56fc161b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'objRec_SetStream-request)))
  "Returns md5sum for a message object of type 'objRec_SetStream-request"
  "d91a7e76b8093c49a72a190a56fc161b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<objRec_SetStream-request>)))
  "Returns full string definition for message of type '<objRec_SetStream-request>"
  (cl:format cl:nil "~%~%~%~%bool streaming~%string cameraName~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'objRec_SetStream-request)))
  "Returns full string definition for message of type 'objRec_SetStream-request"
  (cl:format cl:nil "~%~%~%~%bool streaming~%string cameraName~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <objRec_SetStream-request>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'cameraName))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <objRec_SetStream-request>))
  "Converts a ROS message object to a list"
  (cl:list 'objRec_SetStream-request
    (cl:cons ':streaming (streaming msg))
    (cl:cons ':cameraName (cameraName msg))
))
;//! \htmlinclude objRec_SetStream-response.msg.html

(cl:defclass <objRec_SetStream-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass objRec_SetStream-response (<objRec_SetStream-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <objRec_SetStream-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'objRec_SetStream-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name objRec_comm-srv:<objRec_SetStream-response> is deprecated: use objRec_comm-srv:objRec_SetStream-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <objRec_SetStream-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:ret-val is deprecated.  Use objRec_comm-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <objRec_SetStream-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:msg-val is deprecated.  Use objRec_comm-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <objRec_SetStream-response>) ostream)
  "Serializes a message object of type '<objRec_SetStream-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <objRec_SetStream-response>) istream)
  "Deserializes a message object of type '<objRec_SetStream-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<objRec_SetStream-response>)))
  "Returns string type for a service object of type '<objRec_SetStream-response>"
  "objRec_comm/objRec_SetStreamResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'objRec_SetStream-response)))
  "Returns string type for a service object of type 'objRec_SetStream-response"
  "objRec_comm/objRec_SetStreamResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<objRec_SetStream-response>)))
  "Returns md5sum for a message object of type '<objRec_SetStream-response>"
  "d91a7e76b8093c49a72a190a56fc161b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'objRec_SetStream-response)))
  "Returns md5sum for a message object of type 'objRec_SetStream-response"
  "d91a7e76b8093c49a72a190a56fc161b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<objRec_SetStream-response>)))
  "Returns full string definition for message of type '<objRec_SetStream-response>"
  (cl:format cl:nil "~%int64 ret~%string msg~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'objRec_SetStream-response)))
  "Returns full string definition for message of type 'objRec_SetStream-response"
  (cl:format cl:nil "~%int64 ret~%string msg~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <objRec_SetStream-response>))
  (cl:+ 0
     8
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <objRec_SetStream-response>))
  "Converts a ROS message object to a list"
  (cl:list 'objRec_SetStream-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':msg (msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'objRec_SetStream)))
  'objRec_SetStream-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'objRec_SetStream)))
  'objRec_SetStream-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'objRec_SetStream)))
  "Returns string type for a service object of type '<objRec_SetStream>"
  "objRec_comm/objRec_SetStream")