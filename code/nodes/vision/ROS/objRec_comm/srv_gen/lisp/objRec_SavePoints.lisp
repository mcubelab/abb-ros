; Auto-generated. Do not edit!


(cl:in-package objRec_comm-srv)


;//! \htmlinclude objRec_SavePoints-request.msg.html

(cl:defclass <objRec_SavePoints-request> (roslisp-msg-protocol:ros-message)
  ((is_binary
    :reader is_binary
    :initarg :is_binary
    :type cl:boolean
    :initform cl:nil)
   (use_filter
    :reader use_filter
    :initarg :use_filter
    :type cl:boolean
    :initform cl:nil)
   (cameraName
    :reader cameraName
    :initarg :cameraName
    :type cl:string
    :initform ""))
)

(cl:defclass objRec_SavePoints-request (<objRec_SavePoints-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <objRec_SavePoints-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'objRec_SavePoints-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name objRec_comm-srv:<objRec_SavePoints-request> is deprecated: use objRec_comm-srv:objRec_SavePoints-request instead.")))

(cl:ensure-generic-function 'is_binary-val :lambda-list '(m))
(cl:defmethod is_binary-val ((m <objRec_SavePoints-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:is_binary-val is deprecated.  Use objRec_comm-srv:is_binary instead.")
  (is_binary m))

(cl:ensure-generic-function 'use_filter-val :lambda-list '(m))
(cl:defmethod use_filter-val ((m <objRec_SavePoints-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:use_filter-val is deprecated.  Use objRec_comm-srv:use_filter instead.")
  (use_filter m))

(cl:ensure-generic-function 'cameraName-val :lambda-list '(m))
(cl:defmethod cameraName-val ((m <objRec_SavePoints-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:cameraName-val is deprecated.  Use objRec_comm-srv:cameraName instead.")
  (cameraName m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <objRec_SavePoints-request>) ostream)
  "Serializes a message object of type '<objRec_SavePoints-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_binary) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'use_filter) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cameraName))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cameraName))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <objRec_SavePoints-request>) istream)
  "Deserializes a message object of type '<objRec_SavePoints-request>"
    (cl:setf (cl:slot-value msg 'is_binary) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'use_filter) (cl:not (cl:zerop (cl:read-byte istream))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<objRec_SavePoints-request>)))
  "Returns string type for a service object of type '<objRec_SavePoints-request>"
  "objRec_comm/objRec_SavePointsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'objRec_SavePoints-request)))
  "Returns string type for a service object of type 'objRec_SavePoints-request"
  "objRec_comm/objRec_SavePointsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<objRec_SavePoints-request>)))
  "Returns md5sum for a message object of type '<objRec_SavePoints-request>"
  "d13bc45391fe78d603201522338f3080")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'objRec_SavePoints-request)))
  "Returns md5sum for a message object of type 'objRec_SavePoints-request"
  "d13bc45391fe78d603201522338f3080")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<objRec_SavePoints-request>)))
  "Returns full string definition for message of type '<objRec_SavePoints-request>"
  (cl:format cl:nil "~%~%~%~%bool is_binary~%bool use_filter~%string cameraName~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'objRec_SavePoints-request)))
  "Returns full string definition for message of type 'objRec_SavePoints-request"
  (cl:format cl:nil "~%~%~%~%bool is_binary~%bool use_filter~%string cameraName~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <objRec_SavePoints-request>))
  (cl:+ 0
     1
     1
     4 (cl:length (cl:slot-value msg 'cameraName))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <objRec_SavePoints-request>))
  "Converts a ROS message object to a list"
  (cl:list 'objRec_SavePoints-request
    (cl:cons ':is_binary (is_binary msg))
    (cl:cons ':use_filter (use_filter msg))
    (cl:cons ':cameraName (cameraName msg))
))
;//! \htmlinclude objRec_SavePoints-response.msg.html

(cl:defclass <objRec_SavePoints-response> (roslisp-msg-protocol:ros-message)
  ((filename
    :reader filename
    :initarg :filename
    :type cl:string
    :initform "")
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

(cl:defclass objRec_SavePoints-response (<objRec_SavePoints-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <objRec_SavePoints-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'objRec_SavePoints-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name objRec_comm-srv:<objRec_SavePoints-response> is deprecated: use objRec_comm-srv:objRec_SavePoints-response instead.")))

(cl:ensure-generic-function 'filename-val :lambda-list '(m))
(cl:defmethod filename-val ((m <objRec_SavePoints-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:filename-val is deprecated.  Use objRec_comm-srv:filename instead.")
  (filename m))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <objRec_SavePoints-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:ret-val is deprecated.  Use objRec_comm-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <objRec_SavePoints-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:msg-val is deprecated.  Use objRec_comm-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <objRec_SavePoints-response>) ostream)
  "Serializes a message object of type '<objRec_SavePoints-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'filename))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'filename))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <objRec_SavePoints-response>) istream)
  "Deserializes a message object of type '<objRec_SavePoints-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'filename) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'filename) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<objRec_SavePoints-response>)))
  "Returns string type for a service object of type '<objRec_SavePoints-response>"
  "objRec_comm/objRec_SavePointsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'objRec_SavePoints-response)))
  "Returns string type for a service object of type 'objRec_SavePoints-response"
  "objRec_comm/objRec_SavePointsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<objRec_SavePoints-response>)))
  "Returns md5sum for a message object of type '<objRec_SavePoints-response>"
  "d13bc45391fe78d603201522338f3080")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'objRec_SavePoints-response)))
  "Returns md5sum for a message object of type 'objRec_SavePoints-response"
  "d13bc45391fe78d603201522338f3080")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<objRec_SavePoints-response>)))
  "Returns full string definition for message of type '<objRec_SavePoints-response>"
  (cl:format cl:nil "~%string filename~%int64 ret~%string msg~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'objRec_SavePoints-response)))
  "Returns full string definition for message of type 'objRec_SavePoints-response"
  (cl:format cl:nil "~%string filename~%int64 ret~%string msg~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <objRec_SavePoints-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'filename))
     8
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <objRec_SavePoints-response>))
  "Converts a ROS message object to a list"
  (cl:list 'objRec_SavePoints-response
    (cl:cons ':filename (filename msg))
    (cl:cons ':ret (ret msg))
    (cl:cons ':msg (msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'objRec_SavePoints)))
  'objRec_SavePoints-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'objRec_SavePoints)))
  'objRec_SavePoints-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'objRec_SavePoints)))
  "Returns string type for a service object of type '<objRec_SavePoints>"
  "objRec_comm/objRec_SavePoints")