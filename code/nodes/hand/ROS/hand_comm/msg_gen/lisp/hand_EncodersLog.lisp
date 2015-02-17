; Auto-generated. Do not edit!


(cl:in-package hand_comm-msg)


;//! \htmlinclude hand_EncodersLog.msg.html

(cl:defclass <hand_EncodersLog> (roslisp-msg-protocol:ros-message)
  ((timeStamp
    :reader timeStamp
    :initarg :timeStamp
    :type cl:float
    :initform 0.0)
   (encMotor
    :reader encMotor
    :initarg :encMotor
    :type cl:integer
    :initform 0)
   (encFinger
    :reader encFinger
    :initarg :encFinger
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass hand_EncodersLog (<hand_EncodersLog>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <hand_EncodersLog>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'hand_EncodersLog)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hand_comm-msg:<hand_EncodersLog> is deprecated: use hand_comm-msg:hand_EncodersLog instead.")))

(cl:ensure-generic-function 'timeStamp-val :lambda-list '(m))
(cl:defmethod timeStamp-val ((m <hand_EncodersLog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hand_comm-msg:timeStamp-val is deprecated.  Use hand_comm-msg:timeStamp instead.")
  (timeStamp m))

(cl:ensure-generic-function 'encMotor-val :lambda-list '(m))
(cl:defmethod encMotor-val ((m <hand_EncodersLog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hand_comm-msg:encMotor-val is deprecated.  Use hand_comm-msg:encMotor instead.")
  (encMotor m))

(cl:ensure-generic-function 'encFinger-val :lambda-list '(m))
(cl:defmethod encFinger-val ((m <hand_EncodersLog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hand_comm-msg:encFinger-val is deprecated.  Use hand_comm-msg:encFinger instead.")
  (encFinger m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <hand_EncodersLog>) ostream)
  "Serializes a message object of type '<hand_EncodersLog>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'timeStamp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'encMotor)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'encFinger))))
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
   (cl:slot-value msg 'encFinger))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <hand_EncodersLog>) istream)
  "Deserializes a message object of type '<hand_EncodersLog>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'timeStamp) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'encMotor) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'encFinger) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'encFinger)))
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<hand_EncodersLog>)))
  "Returns string type for a message object of type '<hand_EncodersLog>"
  "hand_comm/hand_EncodersLog")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'hand_EncodersLog)))
  "Returns string type for a message object of type 'hand_EncodersLog"
  "hand_comm/hand_EncodersLog")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<hand_EncodersLog>)))
  "Returns md5sum for a message object of type '<hand_EncodersLog>"
  "58a0fd0951943e291b6011d8e5c563ff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'hand_EncodersLog)))
  "Returns md5sum for a message object of type 'hand_EncodersLog"
  "58a0fd0951943e291b6011d8e5c563ff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<hand_EncodersLog>)))
  "Returns full string definition for message of type '<hand_EncodersLog>"
  (cl:format cl:nil "float64 timeStamp    # ROS time-stamp~%int64 encMotor       # Motor encoder~%int64[] encFinger    # Finger encoders~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'hand_EncodersLog)))
  "Returns full string definition for message of type 'hand_EncodersLog"
  (cl:format cl:nil "float64 timeStamp    # ROS time-stamp~%int64 encMotor       # Motor encoder~%int64[] encFinger    # Finger encoders~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <hand_EncodersLog>))
  (cl:+ 0
     8
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'encFinger) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <hand_EncodersLog>))
  "Converts a ROS message object to a list"
  (cl:list 'hand_EncodersLog
    (cl:cons ':timeStamp (timeStamp msg))
    (cl:cons ':encMotor (encMotor msg))
    (cl:cons ':encFinger (encFinger msg))
))
