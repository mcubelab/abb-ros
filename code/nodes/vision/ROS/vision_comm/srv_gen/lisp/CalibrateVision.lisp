; Auto-generated. Do not edit!


(cl:in-package vision_comm-srv)


;//! \htmlinclude CalibrateVision-request.msg.html

(cl:defclass <CalibrateVision-request> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (captureNew
    :reader captureNew
    :initarg :captureNew
    :type cl:boolean
    :initform cl:nil)
   (cameraNum
    :reader cameraNum
    :initarg :cameraNum
    :type cl:integer
    :initform 0)
   (calibrationFilename
    :reader calibrationFilename
    :initarg :calibrationFilename
    :type cl:string
    :initform ""))
)

(cl:defclass CalibrateVision-request (<CalibrateVision-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CalibrateVision-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CalibrateVision-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision_comm-srv:<CalibrateVision-request> is deprecated: use vision_comm-srv:CalibrateVision-request instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CalibrateVision-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision_comm-srv:header-val is deprecated.  Use vision_comm-srv:header instead.")
  (header m))

(cl:ensure-generic-function 'captureNew-val :lambda-list '(m))
(cl:defmethod captureNew-val ((m <CalibrateVision-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision_comm-srv:captureNew-val is deprecated.  Use vision_comm-srv:captureNew instead.")
  (captureNew m))

(cl:ensure-generic-function 'cameraNum-val :lambda-list '(m))
(cl:defmethod cameraNum-val ((m <CalibrateVision-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision_comm-srv:cameraNum-val is deprecated.  Use vision_comm-srv:cameraNum instead.")
  (cameraNum m))

(cl:ensure-generic-function 'calibrationFilename-val :lambda-list '(m))
(cl:defmethod calibrationFilename-val ((m <CalibrateVision-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision_comm-srv:calibrationFilename-val is deprecated.  Use vision_comm-srv:calibrationFilename instead.")
  (calibrationFilename m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CalibrateVision-request>) ostream)
  "Serializes a message object of type '<CalibrateVision-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'captureNew) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'cameraNum)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'calibrationFilename))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'calibrationFilename))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CalibrateVision-request>) istream)
  "Deserializes a message object of type '<CalibrateVision-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'captureNew) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cameraNum) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'calibrationFilename) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'calibrationFilename) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CalibrateVision-request>)))
  "Returns string type for a service object of type '<CalibrateVision-request>"
  "vision_comm/CalibrateVisionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CalibrateVision-request)))
  "Returns string type for a service object of type 'CalibrateVision-request"
  "vision_comm/CalibrateVisionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CalibrateVision-request>)))
  "Returns md5sum for a message object of type '<CalibrateVision-request>"
  "e7d577b66bbc0556d6ba1c6165a366b2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CalibrateVision-request)))
  "Returns md5sum for a message object of type 'CalibrateVision-request"
  "e7d577b66bbc0556d6ba1c6165a366b2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CalibrateVision-request>)))
  "Returns full string definition for message of type '<CalibrateVision-request>"
  (cl:format cl:nil "Header header~%~%bool captureNew~%int32 cameraNum~%~%~%~%~%~%~%string calibrationFilename~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CalibrateVision-request)))
  "Returns full string definition for message of type 'CalibrateVision-request"
  (cl:format cl:nil "Header header~%~%bool captureNew~%int32 cameraNum~%~%~%~%~%~%~%string calibrationFilename~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CalibrateVision-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4
     4 (cl:length (cl:slot-value msg 'calibrationFilename))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CalibrateVision-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CalibrateVision-request
    (cl:cons ':header (header msg))
    (cl:cons ':captureNew (captureNew msg))
    (cl:cons ':cameraNum (cameraNum msg))
    (cl:cons ':calibrationFilename (calibrationFilename msg))
))
;//! \htmlinclude CalibrateVision-response.msg.html

(cl:defclass <CalibrateVision-response> (roslisp-msg-protocol:ros-message)
  ((response
    :reader response
    :initarg :response
    :type cl:string
    :initform ""))
)

(cl:defclass CalibrateVision-response (<CalibrateVision-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CalibrateVision-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CalibrateVision-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision_comm-srv:<CalibrateVision-response> is deprecated: use vision_comm-srv:CalibrateVision-response instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <CalibrateVision-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision_comm-srv:response-val is deprecated.  Use vision_comm-srv:response instead.")
  (response m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CalibrateVision-response>) ostream)
  "Serializes a message object of type '<CalibrateVision-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'response))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'response))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CalibrateVision-response>) istream)
  "Deserializes a message object of type '<CalibrateVision-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'response) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'response) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CalibrateVision-response>)))
  "Returns string type for a service object of type '<CalibrateVision-response>"
  "vision_comm/CalibrateVisionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CalibrateVision-response)))
  "Returns string type for a service object of type 'CalibrateVision-response"
  "vision_comm/CalibrateVisionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CalibrateVision-response>)))
  "Returns md5sum for a message object of type '<CalibrateVision-response>"
  "e7d577b66bbc0556d6ba1c6165a366b2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CalibrateVision-response)))
  "Returns md5sum for a message object of type 'CalibrateVision-response"
  "e7d577b66bbc0556d6ba1c6165a366b2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CalibrateVision-response>)))
  "Returns full string definition for message of type '<CalibrateVision-response>"
  (cl:format cl:nil "string response~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CalibrateVision-response)))
  "Returns full string definition for message of type 'CalibrateVision-response"
  (cl:format cl:nil "string response~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CalibrateVision-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'response))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CalibrateVision-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CalibrateVision-response
    (cl:cons ':response (response msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CalibrateVision)))
  'CalibrateVision-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CalibrateVision)))
  'CalibrateVision-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CalibrateVision)))
  "Returns string type for a service object of type '<CalibrateVision>"
  "vision_comm/CalibrateVision")