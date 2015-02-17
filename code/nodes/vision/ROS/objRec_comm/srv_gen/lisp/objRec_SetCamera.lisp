; Auto-generated. Do not edit!


(cl:in-package objRec_comm-srv)


;//! \htmlinclude objRec_SetCamera-request.msg.html

(cl:defclass <objRec_SetCamera-request> (roslisp-msg-protocol:ros-message)
  ((trans
    :reader trans
    :initarg :trans
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (quat
    :reader quat
    :initarg :quat
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0))
   (cameraName
    :reader cameraName
    :initarg :cameraName
    :type cl:string
    :initform ""))
)

(cl:defclass objRec_SetCamera-request (<objRec_SetCamera-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <objRec_SetCamera-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'objRec_SetCamera-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name objRec_comm-srv:<objRec_SetCamera-request> is deprecated: use objRec_comm-srv:objRec_SetCamera-request instead.")))

(cl:ensure-generic-function 'trans-val :lambda-list '(m))
(cl:defmethod trans-val ((m <objRec_SetCamera-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:trans-val is deprecated.  Use objRec_comm-srv:trans instead.")
  (trans m))

(cl:ensure-generic-function 'quat-val :lambda-list '(m))
(cl:defmethod quat-val ((m <objRec_SetCamera-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:quat-val is deprecated.  Use objRec_comm-srv:quat instead.")
  (quat m))

(cl:ensure-generic-function 'cameraName-val :lambda-list '(m))
(cl:defmethod cameraName-val ((m <objRec_SetCamera-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:cameraName-val is deprecated.  Use objRec_comm-srv:cameraName instead.")
  (cameraName m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <objRec_SetCamera-request>) ostream)
  "Serializes a message object of type '<objRec_SetCamera-request>"
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
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cameraName))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cameraName))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <objRec_SetCamera-request>) istream)
  "Deserializes a message object of type '<objRec_SetCamera-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<objRec_SetCamera-request>)))
  "Returns string type for a service object of type '<objRec_SetCamera-request>"
  "objRec_comm/objRec_SetCameraRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'objRec_SetCamera-request)))
  "Returns string type for a service object of type 'objRec_SetCamera-request"
  "objRec_comm/objRec_SetCameraRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<objRec_SetCamera-request>)))
  "Returns md5sum for a message object of type '<objRec_SetCamera-request>"
  "f1d8276e89a5b46b314ccef3039988a4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'objRec_SetCamera-request)))
  "Returns md5sum for a message object of type 'objRec_SetCamera-request"
  "f1d8276e89a5b46b314ccef3039988a4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<objRec_SetCamera-request>)))
  "Returns full string definition for message of type '<objRec_SetCamera-request>"
  (cl:format cl:nil "~%~%~%float64[3] trans~%float64[4] quat~%~%string cameraName~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'objRec_SetCamera-request)))
  "Returns full string definition for message of type 'objRec_SetCamera-request"
  (cl:format cl:nil "~%~%~%float64[3] trans~%float64[4] quat~%~%string cameraName~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <objRec_SetCamera-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'trans) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'quat) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:length (cl:slot-value msg 'cameraName))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <objRec_SetCamera-request>))
  "Converts a ROS message object to a list"
  (cl:list 'objRec_SetCamera-request
    (cl:cons ':trans (trans msg))
    (cl:cons ':quat (quat msg))
    (cl:cons ':cameraName (cameraName msg))
))
;//! \htmlinclude objRec_SetCamera-response.msg.html

(cl:defclass <objRec_SetCamera-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass objRec_SetCamera-response (<objRec_SetCamera-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <objRec_SetCamera-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'objRec_SetCamera-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name objRec_comm-srv:<objRec_SetCamera-response> is deprecated: use objRec_comm-srv:objRec_SetCamera-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <objRec_SetCamera-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:ret-val is deprecated.  Use objRec_comm-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <objRec_SetCamera-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:msg-val is deprecated.  Use objRec_comm-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <objRec_SetCamera-response>) ostream)
  "Serializes a message object of type '<objRec_SetCamera-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <objRec_SetCamera-response>) istream)
  "Deserializes a message object of type '<objRec_SetCamera-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<objRec_SetCamera-response>)))
  "Returns string type for a service object of type '<objRec_SetCamera-response>"
  "objRec_comm/objRec_SetCameraResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'objRec_SetCamera-response)))
  "Returns string type for a service object of type 'objRec_SetCamera-response"
  "objRec_comm/objRec_SetCameraResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<objRec_SetCamera-response>)))
  "Returns md5sum for a message object of type '<objRec_SetCamera-response>"
  "f1d8276e89a5b46b314ccef3039988a4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'objRec_SetCamera-response)))
  "Returns md5sum for a message object of type 'objRec_SetCamera-response"
  "f1d8276e89a5b46b314ccef3039988a4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<objRec_SetCamera-response>)))
  "Returns full string definition for message of type '<objRec_SetCamera-response>"
  (cl:format cl:nil "~%int64 ret~%string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'objRec_SetCamera-response)))
  "Returns full string definition for message of type 'objRec_SetCamera-response"
  (cl:format cl:nil "~%int64 ret~%string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <objRec_SetCamera-response>))
  (cl:+ 0
     8
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <objRec_SetCamera-response>))
  "Converts a ROS message object to a list"
  (cl:list 'objRec_SetCamera-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':msg (msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'objRec_SetCamera)))
  'objRec_SetCamera-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'objRec_SetCamera)))
  'objRec_SetCamera-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'objRec_SetCamera)))
  "Returns string type for a service object of type '<objRec_SetCamera>"
  "objRec_comm/objRec_SetCamera")