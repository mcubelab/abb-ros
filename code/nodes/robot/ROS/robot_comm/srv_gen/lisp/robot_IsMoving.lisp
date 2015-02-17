; Auto-generated. Do not edit!


(cl:in-package robot_comm-srv)


;//! \htmlinclude robot_IsMoving-request.msg.html

(cl:defclass <robot_IsMoving-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass robot_IsMoving-request (<robot_IsMoving-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <robot_IsMoving-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'robot_IsMoving-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_comm-srv:<robot_IsMoving-request> is deprecated: use robot_comm-srv:robot_IsMoving-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <robot_IsMoving-request>) ostream)
  "Serializes a message object of type '<robot_IsMoving-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <robot_IsMoving-request>) istream)
  "Deserializes a message object of type '<robot_IsMoving-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<robot_IsMoving-request>)))
  "Returns string type for a service object of type '<robot_IsMoving-request>"
  "robot_comm/robot_IsMovingRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robot_IsMoving-request)))
  "Returns string type for a service object of type 'robot_IsMoving-request"
  "robot_comm/robot_IsMovingRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<robot_IsMoving-request>)))
  "Returns md5sum for a message object of type '<robot_IsMoving-request>"
  "6a8e4a0e30cc934246f28f5db62a1332")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'robot_IsMoving-request)))
  "Returns md5sum for a message object of type 'robot_IsMoving-request"
  "6a8e4a0e30cc934246f28f5db62a1332")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<robot_IsMoving-request>)))
  "Returns full string definition for message of type '<robot_IsMoving-request>"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'robot_IsMoving-request)))
  "Returns full string definition for message of type 'robot_IsMoving-request"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <robot_IsMoving-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <robot_IsMoving-request>))
  "Converts a ROS message object to a list"
  (cl:list 'robot_IsMoving-request
))
;//! \htmlinclude robot_IsMoving-response.msg.html

(cl:defclass <robot_IsMoving-response> (roslisp-msg-protocol:ros-message)
  ((moving
    :reader moving
    :initarg :moving
    :type cl:boolean
    :initform cl:nil)
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

(cl:defclass robot_IsMoving-response (<robot_IsMoving-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <robot_IsMoving-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'robot_IsMoving-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_comm-srv:<robot_IsMoving-response> is deprecated: use robot_comm-srv:robot_IsMoving-response instead.")))

(cl:ensure-generic-function 'moving-val :lambda-list '(m))
(cl:defmethod moving-val ((m <robot_IsMoving-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_comm-srv:moving-val is deprecated.  Use robot_comm-srv:moving instead.")
  (moving m))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <robot_IsMoving-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_comm-srv:ret-val is deprecated.  Use robot_comm-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <robot_IsMoving-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_comm-srv:msg-val is deprecated.  Use robot_comm-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <robot_IsMoving-response>) ostream)
  "Serializes a message object of type '<robot_IsMoving-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'moving) 1 0)) ostream)
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <robot_IsMoving-response>) istream)
  "Deserializes a message object of type '<robot_IsMoving-response>"
    (cl:setf (cl:slot-value msg 'moving) (cl:not (cl:zerop (cl:read-byte istream))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<robot_IsMoving-response>)))
  "Returns string type for a service object of type '<robot_IsMoving-response>"
  "robot_comm/robot_IsMovingResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robot_IsMoving-response)))
  "Returns string type for a service object of type 'robot_IsMoving-response"
  "robot_comm/robot_IsMovingResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<robot_IsMoving-response>)))
  "Returns md5sum for a message object of type '<robot_IsMoving-response>"
  "6a8e4a0e30cc934246f28f5db62a1332")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'robot_IsMoving-response)))
  "Returns md5sum for a message object of type 'robot_IsMoving-response"
  "6a8e4a0e30cc934246f28f5db62a1332")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<robot_IsMoving-response>)))
  "Returns full string definition for message of type '<robot_IsMoving-response>"
  (cl:format cl:nil "bool moving~%int64 ret~%string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'robot_IsMoving-response)))
  "Returns full string definition for message of type 'robot_IsMoving-response"
  (cl:format cl:nil "bool moving~%int64 ret~%string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <robot_IsMoving-response>))
  (cl:+ 0
     1
     8
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <robot_IsMoving-response>))
  "Converts a ROS message object to a list"
  (cl:list 'robot_IsMoving-response
    (cl:cons ':moving (moving msg))
    (cl:cons ':ret (ret msg))
    (cl:cons ':msg (msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'robot_IsMoving)))
  'robot_IsMoving-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'robot_IsMoving)))
  'robot_IsMoving-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robot_IsMoving)))
  "Returns string type for a service object of type '<robot_IsMoving>"
  "robot_comm/robot_IsMoving")