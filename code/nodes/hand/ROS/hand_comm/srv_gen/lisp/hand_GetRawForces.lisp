; Auto-generated. Do not edit!


(cl:in-package hand_comm-srv)


;//! \htmlinclude hand_GetRawForces-request.msg.html

(cl:defclass <hand_GetRawForces-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass hand_GetRawForces-request (<hand_GetRawForces-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <hand_GetRawForces-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'hand_GetRawForces-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hand_comm-srv:<hand_GetRawForces-request> is deprecated: use hand_comm-srv:hand_GetRawForces-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <hand_GetRawForces-request>) ostream)
  "Serializes a message object of type '<hand_GetRawForces-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <hand_GetRawForces-request>) istream)
  "Deserializes a message object of type '<hand_GetRawForces-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<hand_GetRawForces-request>)))
  "Returns string type for a service object of type '<hand_GetRawForces-request>"
  "hand_comm/hand_GetRawForcesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'hand_GetRawForces-request)))
  "Returns string type for a service object of type 'hand_GetRawForces-request"
  "hand_comm/hand_GetRawForcesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<hand_GetRawForces-request>)))
  "Returns md5sum for a message object of type '<hand_GetRawForces-request>"
  "fd50740d9347aa4f4ab5b4d7ba228b8e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'hand_GetRawForces-request)))
  "Returns md5sum for a message object of type 'hand_GetRawForces-request"
  "fd50740d9347aa4f4ab5b4d7ba228b8e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<hand_GetRawForces-request>)))
  "Returns full string definition for message of type '<hand_GetRawForces-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'hand_GetRawForces-request)))
  "Returns full string definition for message of type 'hand_GetRawForces-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <hand_GetRawForces-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <hand_GetRawForces-request>))
  "Converts a ROS message object to a list"
  (cl:list 'hand_GetRawForces-request
))
;//! \htmlinclude hand_GetRawForces-response.msg.html

(cl:defclass <hand_GetRawForces-response> (roslisp-msg-protocol:ros-message)
  ((forces
    :reader forces
    :initarg :forces
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
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

(cl:defclass hand_GetRawForces-response (<hand_GetRawForces-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <hand_GetRawForces-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'hand_GetRawForces-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hand_comm-srv:<hand_GetRawForces-response> is deprecated: use hand_comm-srv:hand_GetRawForces-response instead.")))

(cl:ensure-generic-function 'forces-val :lambda-list '(m))
(cl:defmethod forces-val ((m <hand_GetRawForces-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hand_comm-srv:forces-val is deprecated.  Use hand_comm-srv:forces instead.")
  (forces m))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <hand_GetRawForces-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hand_comm-srv:ret-val is deprecated.  Use hand_comm-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <hand_GetRawForces-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hand_comm-srv:msg-val is deprecated.  Use hand_comm-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <hand_GetRawForces-response>) ostream)
  "Serializes a message object of type '<hand_GetRawForces-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'forces))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    ))
   (cl:slot-value msg 'forces))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <hand_GetRawForces-response>) istream)
  "Deserializes a message object of type '<hand_GetRawForces-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'forces) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'forces)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616)))))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<hand_GetRawForces-response>)))
  "Returns string type for a service object of type '<hand_GetRawForces-response>"
  "hand_comm/hand_GetRawForcesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'hand_GetRawForces-response)))
  "Returns string type for a service object of type 'hand_GetRawForces-response"
  "hand_comm/hand_GetRawForcesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<hand_GetRawForces-response>)))
  "Returns md5sum for a message object of type '<hand_GetRawForces-response>"
  "fd50740d9347aa4f4ab5b4d7ba228b8e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'hand_GetRawForces-response)))
  "Returns md5sum for a message object of type 'hand_GetRawForces-response"
  "fd50740d9347aa4f4ab5b4d7ba228b8e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<hand_GetRawForces-response>)))
  "Returns full string definition for message of type '<hand_GetRawForces-response>"
  (cl:format cl:nil "int64[] forces~%int64 ret~%string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'hand_GetRawForces-response)))
  "Returns full string definition for message of type 'hand_GetRawForces-response"
  (cl:format cl:nil "int64[] forces~%int64 ret~%string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <hand_GetRawForces-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'forces) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     8
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <hand_GetRawForces-response>))
  "Converts a ROS message object to a list"
  (cl:list 'hand_GetRawForces-response
    (cl:cons ':forces (forces msg))
    (cl:cons ':ret (ret msg))
    (cl:cons ':msg (msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'hand_GetRawForces)))
  'hand_GetRawForces-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'hand_GetRawForces)))
  'hand_GetRawForces-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'hand_GetRawForces)))
  "Returns string type for a service object of type '<hand_GetRawForces>"
  "hand_comm/hand_GetRawForces")