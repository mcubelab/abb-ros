; Auto-generated. Do not edit!


(cl:in-package objRec_comm-srv)


;//! \htmlinclude objRec_SetPrefOrient-request.msg.html

(cl:defclass <objRec_SetPrefOrient-request> (roslisp-msg-protocol:ros-message)
  ((use_pref_orient
    :reader use_pref_orient
    :initarg :use_pref_orient
    :type cl:boolean
    :initform cl:nil)
   (quat
    :reader quat
    :initarg :quat
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass objRec_SetPrefOrient-request (<objRec_SetPrefOrient-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <objRec_SetPrefOrient-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'objRec_SetPrefOrient-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name objRec_comm-srv:<objRec_SetPrefOrient-request> is deprecated: use objRec_comm-srv:objRec_SetPrefOrient-request instead.")))

(cl:ensure-generic-function 'use_pref_orient-val :lambda-list '(m))
(cl:defmethod use_pref_orient-val ((m <objRec_SetPrefOrient-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:use_pref_orient-val is deprecated.  Use objRec_comm-srv:use_pref_orient instead.")
  (use_pref_orient m))

(cl:ensure-generic-function 'quat-val :lambda-list '(m))
(cl:defmethod quat-val ((m <objRec_SetPrefOrient-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:quat-val is deprecated.  Use objRec_comm-srv:quat instead.")
  (quat m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <objRec_SetPrefOrient-request>) ostream)
  "Serializes a message object of type '<objRec_SetPrefOrient-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'use_pref_orient) 1 0)) ostream)
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <objRec_SetPrefOrient-request>) istream)
  "Deserializes a message object of type '<objRec_SetPrefOrient-request>"
    (cl:setf (cl:slot-value msg 'use_pref_orient) (cl:not (cl:zerop (cl:read-byte istream))))
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<objRec_SetPrefOrient-request>)))
  "Returns string type for a service object of type '<objRec_SetPrefOrient-request>"
  "objRec_comm/objRec_SetPrefOrientRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'objRec_SetPrefOrient-request)))
  "Returns string type for a service object of type 'objRec_SetPrefOrient-request"
  "objRec_comm/objRec_SetPrefOrientRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<objRec_SetPrefOrient-request>)))
  "Returns md5sum for a message object of type '<objRec_SetPrefOrient-request>"
  "f418110df96aec9e7b7f254388241ede")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'objRec_SetPrefOrient-request)))
  "Returns md5sum for a message object of type 'objRec_SetPrefOrient-request"
  "f418110df96aec9e7b7f254388241ede")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<objRec_SetPrefOrient-request>)))
  "Returns full string definition for message of type '<objRec_SetPrefOrient-request>"
  (cl:format cl:nil "~%~%~%bool use_pref_orient~%float64[4] quat~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'objRec_SetPrefOrient-request)))
  "Returns full string definition for message of type 'objRec_SetPrefOrient-request"
  (cl:format cl:nil "~%~%~%bool use_pref_orient~%float64[4] quat~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <objRec_SetPrefOrient-request>))
  (cl:+ 0
     1
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'quat) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <objRec_SetPrefOrient-request>))
  "Converts a ROS message object to a list"
  (cl:list 'objRec_SetPrefOrient-request
    (cl:cons ':use_pref_orient (use_pref_orient msg))
    (cl:cons ':quat (quat msg))
))
;//! \htmlinclude objRec_SetPrefOrient-response.msg.html

(cl:defclass <objRec_SetPrefOrient-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass objRec_SetPrefOrient-response (<objRec_SetPrefOrient-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <objRec_SetPrefOrient-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'objRec_SetPrefOrient-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name objRec_comm-srv:<objRec_SetPrefOrient-response> is deprecated: use objRec_comm-srv:objRec_SetPrefOrient-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <objRec_SetPrefOrient-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:ret-val is deprecated.  Use objRec_comm-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <objRec_SetPrefOrient-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-srv:msg-val is deprecated.  Use objRec_comm-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <objRec_SetPrefOrient-response>) ostream)
  "Serializes a message object of type '<objRec_SetPrefOrient-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <objRec_SetPrefOrient-response>) istream)
  "Deserializes a message object of type '<objRec_SetPrefOrient-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<objRec_SetPrefOrient-response>)))
  "Returns string type for a service object of type '<objRec_SetPrefOrient-response>"
  "objRec_comm/objRec_SetPrefOrientResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'objRec_SetPrefOrient-response)))
  "Returns string type for a service object of type 'objRec_SetPrefOrient-response"
  "objRec_comm/objRec_SetPrefOrientResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<objRec_SetPrefOrient-response>)))
  "Returns md5sum for a message object of type '<objRec_SetPrefOrient-response>"
  "f418110df96aec9e7b7f254388241ede")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'objRec_SetPrefOrient-response)))
  "Returns md5sum for a message object of type 'objRec_SetPrefOrient-response"
  "f418110df96aec9e7b7f254388241ede")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<objRec_SetPrefOrient-response>)))
  "Returns full string definition for message of type '<objRec_SetPrefOrient-response>"
  (cl:format cl:nil "~%int64 ret~%string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'objRec_SetPrefOrient-response)))
  "Returns full string definition for message of type 'objRec_SetPrefOrient-response"
  (cl:format cl:nil "~%int64 ret~%string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <objRec_SetPrefOrient-response>))
  (cl:+ 0
     8
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <objRec_SetPrefOrient-response>))
  "Converts a ROS message object to a list"
  (cl:list 'objRec_SetPrefOrient-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':msg (msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'objRec_SetPrefOrient)))
  'objRec_SetPrefOrient-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'objRec_SetPrefOrient)))
  'objRec_SetPrefOrient-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'objRec_SetPrefOrient)))
  "Returns string type for a service object of type '<objRec_SetPrefOrient>"
  "objRec_comm/objRec_SetPrefOrient")