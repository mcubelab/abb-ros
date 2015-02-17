; Auto-generated. Do not edit!


(cl:in-package vision_comm-srv)


;//! \htmlinclude DropDetect-request.msg.html

(cl:defclass <DropDetect-request> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (filename
    :reader filename
    :initarg :filename
    :type cl:string
    :initform ""))
)

(cl:defclass DropDetect-request (<DropDetect-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DropDetect-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DropDetect-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision_comm-srv:<DropDetect-request> is deprecated: use vision_comm-srv:DropDetect-request instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <DropDetect-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision_comm-srv:header-val is deprecated.  Use vision_comm-srv:header instead.")
  (header m))

(cl:ensure-generic-function 'filename-val :lambda-list '(m))
(cl:defmethod filename-val ((m <DropDetect-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision_comm-srv:filename-val is deprecated.  Use vision_comm-srv:filename instead.")
  (filename m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DropDetect-request>) ostream)
  "Serializes a message object of type '<DropDetect-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'filename))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'filename))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DropDetect-request>) istream)
  "Deserializes a message object of type '<DropDetect-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DropDetect-request>)))
  "Returns string type for a service object of type '<DropDetect-request>"
  "vision_comm/DropDetectRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DropDetect-request)))
  "Returns string type for a service object of type 'DropDetect-request"
  "vision_comm/DropDetectRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DropDetect-request>)))
  "Returns md5sum for a message object of type '<DropDetect-request>"
  "75e106f1bb8a88f6b8f0c3d82213e7db")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DropDetect-request)))
  "Returns md5sum for a message object of type 'DropDetect-request"
  "75e106f1bb8a88f6b8f0c3d82213e7db")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DropDetect-request>)))
  "Returns full string definition for message of type '<DropDetect-request>"
  (cl:format cl:nil "Header header~%~%string filename~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DropDetect-request)))
  "Returns full string definition for message of type 'DropDetect-request"
  (cl:format cl:nil "Header header~%~%string filename~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DropDetect-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'filename))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DropDetect-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DropDetect-request
    (cl:cons ':header (header msg))
    (cl:cons ':filename (filename msg))
))
;//! \htmlinclude DropDetect-response.msg.html

(cl:defclass <DropDetect-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass DropDetect-response (<DropDetect-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DropDetect-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DropDetect-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision_comm-srv:<DropDetect-response> is deprecated: use vision_comm-srv:DropDetect-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <DropDetect-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision_comm-srv:success-val is deprecated.  Use vision_comm-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DropDetect-response>) ostream)
  "Serializes a message object of type '<DropDetect-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DropDetect-response>) istream)
  "Deserializes a message object of type '<DropDetect-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DropDetect-response>)))
  "Returns string type for a service object of type '<DropDetect-response>"
  "vision_comm/DropDetectResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DropDetect-response)))
  "Returns string type for a service object of type 'DropDetect-response"
  "vision_comm/DropDetectResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DropDetect-response>)))
  "Returns md5sum for a message object of type '<DropDetect-response>"
  "75e106f1bb8a88f6b8f0c3d82213e7db")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DropDetect-response)))
  "Returns md5sum for a message object of type 'DropDetect-response"
  "75e106f1bb8a88f6b8f0c3d82213e7db")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DropDetect-response>)))
  "Returns full string definition for message of type '<DropDetect-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DropDetect-response)))
  "Returns full string definition for message of type 'DropDetect-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DropDetect-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DropDetect-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DropDetect-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DropDetect)))
  'DropDetect-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DropDetect)))
  'DropDetect-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DropDetect)))
  "Returns string type for a service object of type '<DropDetect>"
  "vision_comm/DropDetect")