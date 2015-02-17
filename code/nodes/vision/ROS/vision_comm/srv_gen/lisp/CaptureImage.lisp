; Auto-generated. Do not edit!


(cl:in-package vision_comm-srv)


;//! \htmlinclude CaptureImage-request.msg.html

(cl:defclass <CaptureImage-request> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (cameraNum
    :reader cameraNum
    :initarg :cameraNum
    :type cl:integer
    :initform 0))
)

(cl:defclass CaptureImage-request (<CaptureImage-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CaptureImage-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CaptureImage-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision_comm-srv:<CaptureImage-request> is deprecated: use vision_comm-srv:CaptureImage-request instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CaptureImage-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision_comm-srv:header-val is deprecated.  Use vision_comm-srv:header instead.")
  (header m))

(cl:ensure-generic-function 'cameraNum-val :lambda-list '(m))
(cl:defmethod cameraNum-val ((m <CaptureImage-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision_comm-srv:cameraNum-val is deprecated.  Use vision_comm-srv:cameraNum instead.")
  (cameraNum m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CaptureImage-request>) ostream)
  "Serializes a message object of type '<CaptureImage-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'cameraNum)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CaptureImage-request>) istream)
  "Deserializes a message object of type '<CaptureImage-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cameraNum) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CaptureImage-request>)))
  "Returns string type for a service object of type '<CaptureImage-request>"
  "vision_comm/CaptureImageRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CaptureImage-request)))
  "Returns string type for a service object of type 'CaptureImage-request"
  "vision_comm/CaptureImageRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CaptureImage-request>)))
  "Returns md5sum for a message object of type '<CaptureImage-request>"
  "a0af4dcbda7838a3057789313ac665ad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CaptureImage-request)))
  "Returns md5sum for a message object of type 'CaptureImage-request"
  "a0af4dcbda7838a3057789313ac665ad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CaptureImage-request>)))
  "Returns full string definition for message of type '<CaptureImage-request>"
  (cl:format cl:nil "Header header~%~%int32 cameraNum~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CaptureImage-request)))
  "Returns full string definition for message of type 'CaptureImage-request"
  (cl:format cl:nil "Header header~%~%int32 cameraNum~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CaptureImage-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CaptureImage-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CaptureImage-request
    (cl:cons ':header (header msg))
    (cl:cons ':cameraNum (cameraNum msg))
))
;//! \htmlinclude CaptureImage-response.msg.html

(cl:defclass <CaptureImage-response> (roslisp-msg-protocol:ros-message)
  ((filename
    :reader filename
    :initarg :filename
    :type cl:string
    :initform ""))
)

(cl:defclass CaptureImage-response (<CaptureImage-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CaptureImage-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CaptureImage-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision_comm-srv:<CaptureImage-response> is deprecated: use vision_comm-srv:CaptureImage-response instead.")))

(cl:ensure-generic-function 'filename-val :lambda-list '(m))
(cl:defmethod filename-val ((m <CaptureImage-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision_comm-srv:filename-val is deprecated.  Use vision_comm-srv:filename instead.")
  (filename m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CaptureImage-response>) ostream)
  "Serializes a message object of type '<CaptureImage-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'filename))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'filename))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CaptureImage-response>) istream)
  "Deserializes a message object of type '<CaptureImage-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'filename) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'filename) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CaptureImage-response>)))
  "Returns string type for a service object of type '<CaptureImage-response>"
  "vision_comm/CaptureImageResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CaptureImage-response)))
  "Returns string type for a service object of type 'CaptureImage-response"
  "vision_comm/CaptureImageResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CaptureImage-response>)))
  "Returns md5sum for a message object of type '<CaptureImage-response>"
  "a0af4dcbda7838a3057789313ac665ad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CaptureImage-response)))
  "Returns md5sum for a message object of type 'CaptureImage-response"
  "a0af4dcbda7838a3057789313ac665ad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CaptureImage-response>)))
  "Returns full string definition for message of type '<CaptureImage-response>"
  (cl:format cl:nil "string filename~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CaptureImage-response)))
  "Returns full string definition for message of type 'CaptureImage-response"
  (cl:format cl:nil "string filename~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CaptureImage-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'filename))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CaptureImage-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CaptureImage-response
    (cl:cons ':filename (filename msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CaptureImage)))
  'CaptureImage-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CaptureImage)))
  'CaptureImage-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CaptureImage)))
  "Returns string type for a service object of type '<CaptureImage>"
  "vision_comm/CaptureImage")