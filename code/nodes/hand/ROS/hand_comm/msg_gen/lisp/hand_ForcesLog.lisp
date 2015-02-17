; Auto-generated. Do not edit!


(cl:in-package hand_comm-msg)


;//! \htmlinclude hand_ForcesLog.msg.html

(cl:defclass <hand_ForcesLog> (roslisp-msg-protocol:ros-message)
  ((timeStamp
    :reader timeStamp
    :initarg :timeStamp
    :type cl:float
    :initform 0.0)
   (forces
    :reader forces
    :initarg :forces
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass hand_ForcesLog (<hand_ForcesLog>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <hand_ForcesLog>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'hand_ForcesLog)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hand_comm-msg:<hand_ForcesLog> is deprecated: use hand_comm-msg:hand_ForcesLog instead.")))

(cl:ensure-generic-function 'timeStamp-val :lambda-list '(m))
(cl:defmethod timeStamp-val ((m <hand_ForcesLog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hand_comm-msg:timeStamp-val is deprecated.  Use hand_comm-msg:timeStamp instead.")
  (timeStamp m))

(cl:ensure-generic-function 'forces-val :lambda-list '(m))
(cl:defmethod forces-val ((m <hand_ForcesLog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hand_comm-msg:forces-val is deprecated.  Use hand_comm-msg:forces instead.")
  (forces m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <hand_ForcesLog>) ostream)
  "Serializes a message object of type '<hand_ForcesLog>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'timeStamp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'forces))))
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
   (cl:slot-value msg 'forces))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <hand_ForcesLog>) istream)
  "Deserializes a message object of type '<hand_ForcesLog>"
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
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'forces) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'forces)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<hand_ForcesLog>)))
  "Returns string type for a message object of type '<hand_ForcesLog>"
  "hand_comm/hand_ForcesLog")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'hand_ForcesLog)))
  "Returns string type for a message object of type 'hand_ForcesLog"
  "hand_comm/hand_ForcesLog")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<hand_ForcesLog>)))
  "Returns md5sum for a message object of type '<hand_ForcesLog>"
  "3dea637dd46fc45b51477a159c3698a9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'hand_ForcesLog)))
  "Returns md5sum for a message object of type 'hand_ForcesLog"
  "3dea637dd46fc45b51477a159c3698a9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<hand_ForcesLog>)))
  "Returns full string definition for message of type '<hand_ForcesLog>"
  (cl:format cl:nil "float64 timeStamp      # ROS time-stamp~%float64[] forces        # Calibrated force values~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'hand_ForcesLog)))
  "Returns full string definition for message of type 'hand_ForcesLog"
  (cl:format cl:nil "float64 timeStamp      # ROS time-stamp~%float64[] forces        # Calibrated force values~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <hand_ForcesLog>))
  (cl:+ 0
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'forces) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <hand_ForcesLog>))
  "Converts a ROS message object to a list"
  (cl:list 'hand_ForcesLog
    (cl:cons ':timeStamp (timeStamp msg))
    (cl:cons ':forces (forces msg))
))
