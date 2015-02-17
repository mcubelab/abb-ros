; Auto-generated. Do not edit!


(cl:in-package objRec_comm-srv)


;//! \htmlinclude objRec_SetIgnoreRegions-request.msg.html

(cl:defclass <objRec_SetIgnoreRegions-request> (roslisp-msg-protocol:ros-message)
  ((num_regions
    :reader num_regions
    :initarg :num_regions
    :type cl:integer
    :initform 0)
   (widths
    :reader widths
    :initarg :widths
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (trans
    :reader trans
    :initarg :trans
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (quats
    :reader quats
    :initarg :quats
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass objRec_SetIgnoreRegions-request (<objRec_SetIgnoreRegions-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <objRec_SetIgnoreRegions-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'objRec_SetIgnoreRegions-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name objRec_comm-srv:<objRec_SetIgnoreRegions-request> is deprecated: use objRec_comm-srv:objRec_SetIgnoreRegions-request instead.")))

(cl:ensure-generic-function 'num_regions-val :lambda-list '(m))
(cl:defmethod num_regions-val ((m <objRec_SetIgnoreRegions-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:num_regions-val is deprecated.  Use objRec_comm-srv:num_regions instead.")
  (num_regions m))

(cl:ensure-generic-function 'widths-val :lambda-list '(m))
(cl:defmethod widths-val ((m <objRec_SetIgnoreRegions-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:widths-val is deprecated.  Use objRec_comm-srv:widths instead.")
  (widths m))

(cl:ensure-generic-function 'trans-val :lambda-list '(m))
(cl:defmethod trans-val ((m <objRec_SetIgnoreRegions-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:trans-val is deprecated.  Use objRec_comm-srv:trans instead.")
  (trans m))

(cl:ensure-generic-function 'quats-val :lambda-list '(m))
(cl:defmethod quats-val ((m <objRec_SetIgnoreRegions-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:quats-val is deprecated.  Use objRec_comm-srv:quats instead.")
  (quats m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <objRec_SetIgnoreRegions-request>) ostream)
  "Serializes a message object of type '<objRec_SetIgnoreRegions-request>"
  (cl:let* ((signed (cl:slot-value msg 'num_regions)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'widths))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'widths))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'trans))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
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
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'quats))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'quats))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <objRec_SetIgnoreRegions-request>) istream)
  "Deserializes a message object of type '<objRec_SetIgnoreRegions-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_regions) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'widths) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'widths)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'trans) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'trans)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'quats) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'quats)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<objRec_SetIgnoreRegions-request>)))
  "Returns string type for a service object of type '<objRec_SetIgnoreRegions-request>"
  "objRec_comm/objRec_SetIgnoreRegionsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'objRec_SetIgnoreRegions-request)))
  "Returns string type for a service object of type 'objRec_SetIgnoreRegions-request"
  "objRec_comm/objRec_SetIgnoreRegionsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<objRec_SetIgnoreRegions-request>)))
  "Returns md5sum for a message object of type '<objRec_SetIgnoreRegions-request>"
  "12866496b8e2d989f2131728f0e5c751")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'objRec_SetIgnoreRegions-request)))
  "Returns md5sum for a message object of type 'objRec_SetIgnoreRegions-request"
  "12866496b8e2d989f2131728f0e5c751")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<objRec_SetIgnoreRegions-request>)))
  "Returns full string definition for message of type '<objRec_SetIgnoreRegions-request>"
  (cl:format cl:nil "~%~%~%int64 num_regions~%float64[] widths~%float64[] trans~%float64[] quats~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'objRec_SetIgnoreRegions-request)))
  "Returns full string definition for message of type 'objRec_SetIgnoreRegions-request"
  (cl:format cl:nil "~%~%~%int64 num_regions~%float64[] widths~%float64[] trans~%float64[] quats~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <objRec_SetIgnoreRegions-request>))
  (cl:+ 0
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'widths) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'trans) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'quats) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <objRec_SetIgnoreRegions-request>))
  "Converts a ROS message object to a list"
  (cl:list 'objRec_SetIgnoreRegions-request
    (cl:cons ':num_regions (num_regions msg))
    (cl:cons ':widths (widths msg))
    (cl:cons ':trans (trans msg))
    (cl:cons ':quats (quats msg))
))
;//! \htmlinclude objRec_SetIgnoreRegions-response.msg.html

(cl:defclass <objRec_SetIgnoreRegions-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass objRec_SetIgnoreRegions-response (<objRec_SetIgnoreRegions-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <objRec_SetIgnoreRegions-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'objRec_SetIgnoreRegions-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name objRec_comm-srv:<objRec_SetIgnoreRegions-response> is deprecated: use objRec_comm-srv:objRec_SetIgnoreRegions-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <objRec_SetIgnoreRegions-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:ret-val is deprecated.  Use objRec_comm-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <objRec_SetIgnoreRegions-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:msg-val is deprecated.  Use objRec_comm-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <objRec_SetIgnoreRegions-response>) ostream)
  "Serializes a message object of type '<objRec_SetIgnoreRegions-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <objRec_SetIgnoreRegions-response>) istream)
  "Deserializes a message object of type '<objRec_SetIgnoreRegions-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<objRec_SetIgnoreRegions-response>)))
  "Returns string type for a service object of type '<objRec_SetIgnoreRegions-response>"
  "objRec_comm/objRec_SetIgnoreRegionsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'objRec_SetIgnoreRegions-response)))
  "Returns string type for a service object of type 'objRec_SetIgnoreRegions-response"
  "objRec_comm/objRec_SetIgnoreRegionsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<objRec_SetIgnoreRegions-response>)))
  "Returns md5sum for a message object of type '<objRec_SetIgnoreRegions-response>"
  "12866496b8e2d989f2131728f0e5c751")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'objRec_SetIgnoreRegions-response)))
  "Returns md5sum for a message object of type 'objRec_SetIgnoreRegions-response"
  "12866496b8e2d989f2131728f0e5c751")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<objRec_SetIgnoreRegions-response>)))
  "Returns full string definition for message of type '<objRec_SetIgnoreRegions-response>"
  (cl:format cl:nil "~%int64 ret~%string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'objRec_SetIgnoreRegions-response)))
  "Returns full string definition for message of type 'objRec_SetIgnoreRegions-response"
  (cl:format cl:nil "~%int64 ret~%string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <objRec_SetIgnoreRegions-response>))
  (cl:+ 0
     8
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <objRec_SetIgnoreRegions-response>))
  "Converts a ROS message object to a list"
  (cl:list 'objRec_SetIgnoreRegions-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':msg (msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'objRec_SetIgnoreRegions)))
  'objRec_SetIgnoreRegions-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'objRec_SetIgnoreRegions)))
  'objRec_SetIgnoreRegions-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'objRec_SetIgnoreRegions)))
  "Returns string type for a service object of type '<objRec_SetIgnoreRegions>"
  "objRec_comm/objRec_SetIgnoreRegions")