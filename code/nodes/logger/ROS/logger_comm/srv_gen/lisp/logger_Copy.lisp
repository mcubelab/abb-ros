; Auto-generated. Do not edit!


(cl:in-package logger_comm-srv)


;//! \htmlinclude logger_Copy-request.msg.html

(cl:defclass <logger_Copy-request> (roslisp-msg-protocol:ros-message)
  ((filename
    :reader filename
    :initarg :filename
    :type cl:string
    :initform "")
   (folder
    :reader folder
    :initarg :folder
    :type cl:string
    :initform ""))
)

(cl:defclass logger_Copy-request (<logger_Copy-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <logger_Copy-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'logger_Copy-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name logger_comm-srv:<logger_Copy-request> is deprecated: use logger_comm-srv:logger_Copy-request instead.")))

(cl:ensure-generic-function 'filename-val :lambda-list '(m))
(cl:defmethod filename-val ((m <logger_Copy-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader logger_comm-srv:filename-val is deprecated.  Use logger_comm-srv:filename instead.")
  (filename m))

(cl:ensure-generic-function 'folder-val :lambda-list '(m))
(cl:defmethod folder-val ((m <logger_Copy-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader logger_comm-srv:folder-val is deprecated.  Use logger_comm-srv:folder instead.")
  (folder m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <logger_Copy-request>) ostream)
  "Serializes a message object of type '<logger_Copy-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'filename))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'filename))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'folder))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'folder))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <logger_Copy-request>) istream)
  "Deserializes a message object of type '<logger_Copy-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'filename) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'filename) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'folder) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'folder) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<logger_Copy-request>)))
  "Returns string type for a service object of type '<logger_Copy-request>"
  "logger_comm/logger_CopyRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'logger_Copy-request)))
  "Returns string type for a service object of type 'logger_Copy-request"
  "logger_comm/logger_CopyRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<logger_Copy-request>)))
  "Returns md5sum for a message object of type '<logger_Copy-request>"
  "3427d5531530b2330c888216f5d9175b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'logger_Copy-request)))
  "Returns md5sum for a message object of type 'logger_Copy-request"
  "3427d5531530b2330c888216f5d9175b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<logger_Copy-request>)))
  "Returns full string definition for message of type '<logger_Copy-request>"
  (cl:format cl:nil "~%string filename~%string folder~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'logger_Copy-request)))
  "Returns full string definition for message of type 'logger_Copy-request"
  (cl:format cl:nil "~%string filename~%string folder~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <logger_Copy-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'filename))
     4 (cl:length (cl:slot-value msg 'folder))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <logger_Copy-request>))
  "Converts a ROS message object to a list"
  (cl:list 'logger_Copy-request
    (cl:cons ':filename (filename msg))
    (cl:cons ':folder (folder msg))
))
;//! \htmlinclude logger_Copy-response.msg.html

(cl:defclass <logger_Copy-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass logger_Copy-response (<logger_Copy-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <logger_Copy-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'logger_Copy-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name logger_comm-srv:<logger_Copy-response> is deprecated: use logger_comm-srv:logger_Copy-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <logger_Copy-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader logger_comm-srv:ret-val is deprecated.  Use logger_comm-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <logger_Copy-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader logger_comm-srv:msg-val is deprecated.  Use logger_comm-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <logger_Copy-response>) ostream)
  "Serializes a message object of type '<logger_Copy-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <logger_Copy-response>) istream)
  "Deserializes a message object of type '<logger_Copy-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<logger_Copy-response>)))
  "Returns string type for a service object of type '<logger_Copy-response>"
  "logger_comm/logger_CopyResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'logger_Copy-response)))
  "Returns string type for a service object of type 'logger_Copy-response"
  "logger_comm/logger_CopyResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<logger_Copy-response>)))
  "Returns md5sum for a message object of type '<logger_Copy-response>"
  "3427d5531530b2330c888216f5d9175b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'logger_Copy-response)))
  "Returns md5sum for a message object of type 'logger_Copy-response"
  "3427d5531530b2330c888216f5d9175b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<logger_Copy-response>)))
  "Returns full string definition for message of type '<logger_Copy-response>"
  (cl:format cl:nil "int64 ret~%string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'logger_Copy-response)))
  "Returns full string definition for message of type 'logger_Copy-response"
  (cl:format cl:nil "int64 ret~%string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <logger_Copy-response>))
  (cl:+ 0
     8
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <logger_Copy-response>))
  "Converts a ROS message object to a list"
  (cl:list 'logger_Copy-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':msg (msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'logger_Copy)))
  'logger_Copy-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'logger_Copy)))
  'logger_Copy-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'logger_Copy)))
  "Returns string type for a service object of type '<logger_Copy>"
  "logger_comm/logger_Copy")