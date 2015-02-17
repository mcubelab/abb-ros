; Auto-generated. Do not edit!


(cl:in-package vision_comm-srv)


;//! \htmlinclude PingVision-request.msg.html

(cl:defclass <PingVision-request> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header)))
)

(cl:defclass PingVision-request (<PingVision-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PingVision-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PingVision-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision_comm-srv:<PingVision-request> is deprecated: use vision_comm-srv:PingVision-request instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PingVision-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision_comm-srv:header-val is deprecated.  Use vision_comm-srv:header instead.")
  (header m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PingVision-request>) ostream)
  "Serializes a message object of type '<PingVision-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PingVision-request>) istream)
  "Deserializes a message object of type '<PingVision-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PingVision-request>)))
  "Returns string type for a service object of type '<PingVision-request>"
  "vision_comm/PingVisionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PingVision-request)))
  "Returns string type for a service object of type 'PingVision-request"
  "vision_comm/PingVisionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PingVision-request>)))
  "Returns md5sum for a message object of type '<PingVision-request>"
  "d7be0bb39af8fb9129d5a76e6b63a290")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PingVision-request)))
  "Returns md5sum for a message object of type 'PingVision-request"
  "d7be0bb39af8fb9129d5a76e6b63a290")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PingVision-request>)))
  "Returns full string definition for message of type '<PingVision-request>"
  (cl:format cl:nil "Header header~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PingVision-request)))
  "Returns full string definition for message of type 'PingVision-request"
  (cl:format cl:nil "Header header~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PingVision-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PingVision-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PingVision-request
    (cl:cons ':header (header msg))
))
;//! \htmlinclude PingVision-response.msg.html

(cl:defclass <PingVision-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass PingVision-response (<PingVision-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PingVision-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PingVision-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision_comm-srv:<PingVision-response> is deprecated: use vision_comm-srv:PingVision-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PingVision-response>) ostream)
  "Serializes a message object of type '<PingVision-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PingVision-response>) istream)
  "Deserializes a message object of type '<PingVision-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PingVision-response>)))
  "Returns string type for a service object of type '<PingVision-response>"
  "vision_comm/PingVisionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PingVision-response)))
  "Returns string type for a service object of type 'PingVision-response"
  "vision_comm/PingVisionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PingVision-response>)))
  "Returns md5sum for a message object of type '<PingVision-response>"
  "d7be0bb39af8fb9129d5a76e6b63a290")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PingVision-response)))
  "Returns md5sum for a message object of type 'PingVision-response"
  "d7be0bb39af8fb9129d5a76e6b63a290")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PingVision-response>)))
  "Returns full string definition for message of type '<PingVision-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PingVision-response)))
  "Returns full string definition for message of type 'PingVision-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PingVision-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PingVision-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PingVision-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PingVision)))
  'PingVision-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PingVision)))
  'PingVision-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PingVision)))
  "Returns string type for a service object of type '<PingVision>"
  "vision_comm/PingVision")