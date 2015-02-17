; Auto-generated. Do not edit!


(cl:in-package objRec_comm-srv)


;//! \htmlinclude objRec_GetObject-request.msg.html

(cl:defclass <objRec_GetObject-request> (roslisp-msg-protocol:ros-message)
  ((filename
    :reader filename
    :initarg :filename
    :type cl:string
    :initform "")
   (cameraName
    :reader cameraName
    :initarg :cameraName
    :type cl:string
    :initform "")
   (onTable
    :reader onTable
    :initarg :onTable
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass objRec_GetObject-request (<objRec_GetObject-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <objRec_GetObject-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'objRec_GetObject-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name objRec_comm-srv:<objRec_GetObject-request> is deprecated: use objRec_comm-srv:objRec_GetObject-request instead.")))

(cl:ensure-generic-function 'filename-val :lambda-list '(m))
(cl:defmethod filename-val ((m <objRec_GetObject-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:filename-val is deprecated.  Use objRec_comm-srv:filename instead.")
  (filename m))

(cl:ensure-generic-function 'cameraName-val :lambda-list '(m))
(cl:defmethod cameraName-val ((m <objRec_GetObject-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:cameraName-val is deprecated.  Use objRec_comm-srv:cameraName instead.")
  (cameraName m))

(cl:ensure-generic-function 'onTable-val :lambda-list '(m))
(cl:defmethod onTable-val ((m <objRec_GetObject-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:onTable-val is deprecated.  Use objRec_comm-srv:onTable instead.")
  (onTable m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <objRec_GetObject-request>) ostream)
  "Serializes a message object of type '<objRec_GetObject-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'filename))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'filename))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cameraName))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cameraName))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'onTable) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <objRec_GetObject-request>) istream)
  "Deserializes a message object of type '<objRec_GetObject-request>"
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
      (cl:setf (cl:slot-value msg 'cameraName) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'cameraName) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'onTable) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<objRec_GetObject-request>)))
  "Returns string type for a service object of type '<objRec_GetObject-request>"
  "objRec_comm/objRec_GetObjectRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'objRec_GetObject-request)))
  "Returns string type for a service object of type 'objRec_GetObject-request"
  "objRec_comm/objRec_GetObjectRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<objRec_GetObject-request>)))
  "Returns md5sum for a message object of type '<objRec_GetObject-request>"
  "91650f8c373d26a2f3629b3e8c8f5cfe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'objRec_GetObject-request)))
  "Returns md5sum for a message object of type 'objRec_GetObject-request"
  "91650f8c373d26a2f3629b3e8c8f5cfe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<objRec_GetObject-request>)))
  "Returns full string definition for message of type '<objRec_GetObject-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%string filename~%~%string cameraName~%~%bool onTable~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'objRec_GetObject-request)))
  "Returns full string definition for message of type 'objRec_GetObject-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%string filename~%~%string cameraName~%~%bool onTable~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <objRec_GetObject-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'filename))
     4 (cl:length (cl:slot-value msg 'cameraName))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <objRec_GetObject-request>))
  "Converts a ROS message object to a list"
  (cl:list 'objRec_GetObject-request
    (cl:cons ':filename (filename msg))
    (cl:cons ':cameraName (cameraName msg))
    (cl:cons ':onTable (onTable msg))
))
;//! \htmlinclude objRec_GetObject-response.msg.html

(cl:defclass <objRec_GetObject-response> (roslisp-msg-protocol:ros-message)
  ((objNum
    :reader objNum
    :initarg :objNum
    :type cl:integer
    :initform 0)
   (trans
    :reader trans
    :initarg :trans
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (quat
    :reader quat
    :initarg :quat
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0))
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

(cl:defclass objRec_GetObject-response (<objRec_GetObject-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <objRec_GetObject-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'objRec_GetObject-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name objRec_comm-srv:<objRec_GetObject-response> is deprecated: use objRec_comm-srv:objRec_GetObject-response instead.")))

(cl:ensure-generic-function 'objNum-val :lambda-list '(m))
(cl:defmethod objNum-val ((m <objRec_GetObject-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:objNum-val is deprecated.  Use objRec_comm-srv:objNum instead.")
  (objNum m))

(cl:ensure-generic-function 'trans-val :lambda-list '(m))
(cl:defmethod trans-val ((m <objRec_GetObject-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:trans-val is deprecated.  Use objRec_comm-srv:trans instead.")
  (trans m))

(cl:ensure-generic-function 'quat-val :lambda-list '(m))
(cl:defmethod quat-val ((m <objRec_GetObject-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:quat-val is deprecated.  Use objRec_comm-srv:quat instead.")
  (quat m))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <objRec_GetObject-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:ret-val is deprecated.  Use objRec_comm-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <objRec_GetObject-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:msg-val is deprecated.  Use objRec_comm-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <objRec_GetObject-response>) ostream)
  "Serializes a message object of type '<objRec_GetObject-response>"
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
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'trans))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'quat))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <objRec_GetObject-response>) istream)
  "Deserializes a message object of type '<objRec_GetObject-response>"
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
  (cl:setf (cl:slot-value msg 'trans) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'trans)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'quat) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'quat)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<objRec_GetObject-response>)))
  "Returns string type for a service object of type '<objRec_GetObject-response>"
  "objRec_comm/objRec_GetObjectResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'objRec_GetObject-response)))
  "Returns string type for a service object of type 'objRec_GetObject-response"
  "objRec_comm/objRec_GetObjectResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<objRec_GetObject-response>)))
  "Returns md5sum for a message object of type '<objRec_GetObject-response>"
  "91650f8c373d26a2f3629b3e8c8f5cfe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'objRec_GetObject-response)))
  "Returns md5sum for a message object of type 'objRec_GetObject-response"
  "91650f8c373d26a2f3629b3e8c8f5cfe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<objRec_GetObject-response>)))
  "Returns full string definition for message of type '<objRec_GetObject-response>"
  (cl:format cl:nil "~%int64 objNum~%float64[3] trans~%float64[4] quat~%~%int64 ret~%string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'objRec_GetObject-response)))
  "Returns full string definition for message of type 'objRec_GetObject-response"
  (cl:format cl:nil "~%int64 objNum~%float64[3] trans~%float64[4] quat~%~%int64 ret~%string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <objRec_GetObject-response>))
  (cl:+ 0
     8
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'trans) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'quat) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     8
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <objRec_GetObject-response>))
  "Converts a ROS message object to a list"
  (cl:list 'objRec_GetObject-response
    (cl:cons ':objNum (objNum msg))
    (cl:cons ':trans (trans msg))
    (cl:cons ':quat (quat msg))
    (cl:cons ':ret (ret msg))
    (cl:cons ':msg (msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'objRec_GetObject)))
  'objRec_GetObject-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'objRec_GetObject)))
  'objRec_GetObject-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'objRec_GetObject)))
  "Returns string type for a service object of type '<objRec_GetObject>"
  "objRec_comm/objRec_GetObject")