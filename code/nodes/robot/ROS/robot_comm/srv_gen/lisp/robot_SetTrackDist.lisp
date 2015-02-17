; Auto-generated. Do not edit!


(cl:in-package robot_comm-srv)


;//! \htmlinclude robot_SetTrackDist-request.msg.html

(cl:defclass <robot_SetTrackDist-request> (roslisp-msg-protocol:ros-message)
  ((pos_dist
    :reader pos_dist
    :initarg :pos_dist
    :type cl:float
    :initform 0.0)
   (ang_dist
    :reader ang_dist
    :initarg :ang_dist
    :type cl:float
    :initform 0.0))
)

(cl:defclass robot_SetTrackDist-request (<robot_SetTrackDist-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <robot_SetTrackDist-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'robot_SetTrackDist-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_comm-srv:<robot_SetTrackDist-request> is deprecated: use robot_comm-srv:robot_SetTrackDist-request instead.")))

(cl:ensure-generic-function 'pos_dist-val :lambda-list '(m))
(cl:defmethod pos_dist-val ((m <robot_SetTrackDist-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_comm-srv:pos_dist-val is deprecated.  Use robot_comm-srv:pos_dist instead.")
  (pos_dist m))

(cl:ensure-generic-function 'ang_dist-val :lambda-list '(m))
(cl:defmethod ang_dist-val ((m <robot_SetTrackDist-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_comm-srv:ang_dist-val is deprecated.  Use robot_comm-srv:ang_dist instead.")
  (ang_dist m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <robot_SetTrackDist-request>) ostream)
  "Serializes a message object of type '<robot_SetTrackDist-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pos_dist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ang_dist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <robot_SetTrackDist-request>) istream)
  "Deserializes a message object of type '<robot_SetTrackDist-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pos_dist) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ang_dist) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<robot_SetTrackDist-request>)))
  "Returns string type for a service object of type '<robot_SetTrackDist-request>"
  "robot_comm/robot_SetTrackDistRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robot_SetTrackDist-request)))
  "Returns string type for a service object of type 'robot_SetTrackDist-request"
  "robot_comm/robot_SetTrackDistRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<robot_SetTrackDist-request>)))
  "Returns md5sum for a message object of type '<robot_SetTrackDist-request>"
  "eb267eb8369d9c3a686d934285ba7a11")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'robot_SetTrackDist-request)))
  "Returns md5sum for a message object of type 'robot_SetTrackDist-request"
  "eb267eb8369d9c3a686d934285ba7a11")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<robot_SetTrackDist-request>)))
  "Returns full string definition for message of type '<robot_SetTrackDist-request>"
  (cl:format cl:nil "~%~%float64 pos_dist~%float64 ang_dist~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'robot_SetTrackDist-request)))
  "Returns full string definition for message of type 'robot_SetTrackDist-request"
  (cl:format cl:nil "~%~%float64 pos_dist~%float64 ang_dist~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <robot_SetTrackDist-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <robot_SetTrackDist-request>))
  "Converts a ROS message object to a list"
  (cl:list 'robot_SetTrackDist-request
    (cl:cons ':pos_dist (pos_dist msg))
    (cl:cons ':ang_dist (ang_dist msg))
))
;//! \htmlinclude robot_SetTrackDist-response.msg.html

(cl:defclass <robot_SetTrackDist-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass robot_SetTrackDist-response (<robot_SetTrackDist-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <robot_SetTrackDist-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'robot_SetTrackDist-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_comm-srv:<robot_SetTrackDist-response> is deprecated: use robot_comm-srv:robot_SetTrackDist-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <robot_SetTrackDist-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_comm-srv:ret-val is deprecated.  Use robot_comm-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <robot_SetTrackDist-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_comm-srv:msg-val is deprecated.  Use robot_comm-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <robot_SetTrackDist-response>) ostream)
  "Serializes a message object of type '<robot_SetTrackDist-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <robot_SetTrackDist-response>) istream)
  "Deserializes a message object of type '<robot_SetTrackDist-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<robot_SetTrackDist-response>)))
  "Returns string type for a service object of type '<robot_SetTrackDist-response>"
  "robot_comm/robot_SetTrackDistResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robot_SetTrackDist-response)))
  "Returns string type for a service object of type 'robot_SetTrackDist-response"
  "robot_comm/robot_SetTrackDistResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<robot_SetTrackDist-response>)))
  "Returns md5sum for a message object of type '<robot_SetTrackDist-response>"
  "eb267eb8369d9c3a686d934285ba7a11")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'robot_SetTrackDist-response)))
  "Returns md5sum for a message object of type 'robot_SetTrackDist-response"
  "eb267eb8369d9c3a686d934285ba7a11")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<robot_SetTrackDist-response>)))
  "Returns full string definition for message of type '<robot_SetTrackDist-response>"
  (cl:format cl:nil "int64 ret~%string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'robot_SetTrackDist-response)))
  "Returns full string definition for message of type 'robot_SetTrackDist-response"
  (cl:format cl:nil "int64 ret~%string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <robot_SetTrackDist-response>))
  (cl:+ 0
     8
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <robot_SetTrackDist-response>))
  "Converts a ROS message object to a list"
  (cl:list 'robot_SetTrackDist-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':msg (msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'robot_SetTrackDist)))
  'robot_SetTrackDist-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'robot_SetTrackDist)))
  'robot_SetTrackDist-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robot_SetTrackDist)))
  "Returns string type for a service object of type '<robot_SetTrackDist>"
  "robot_comm/robot_SetTrackDist")