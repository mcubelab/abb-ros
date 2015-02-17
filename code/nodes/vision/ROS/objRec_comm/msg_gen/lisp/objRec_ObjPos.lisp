; Auto-generated. Do not edit!


(cl:in-package objRec_comm-msg)


;//! \htmlinclude objRec_ObjPos.msg.html

(cl:defclass <objRec_ObjPos> (roslisp-msg-protocol:ros-message)
  ((objNum
    :reader objNum
    :initarg :objNum
    :type cl:integer
    :initform 0)
   (trans
    :reader trans
    :initarg :trans
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (quat
    :reader quat
    :initarg :quat
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass objRec_ObjPos (<objRec_ObjPos>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <objRec_ObjPos>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'objRec_ObjPos)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name objRec_comm-msg:<objRec_ObjPos> is deprecated: use objRec_comm-msg:objRec_ObjPos instead.")))

(cl:ensure-generic-function 'objNum-val :lambda-list '(m))
(cl:defmethod objNum-val ((m <objRec_ObjPos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-msg:objNum-val is deprecated.  Use objRec_comm-msg:objNum instead.")
  (objNum m))

(cl:ensure-generic-function 'trans-val :lambda-list '(m))
(cl:defmethod trans-val ((m <objRec_ObjPos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-msg:trans-val is deprecated.  Use objRec_comm-msg:trans instead.")
  (trans m))

(cl:ensure-generic-function 'quat-val :lambda-list '(m))
(cl:defmethod quat-val ((m <objRec_ObjPos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objRec_comm-msg:quat-val is deprecated.  Use objRec_comm-msg:quat instead.")
  (quat m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <objRec_ObjPos>) ostream)
  "Serializes a message object of type '<objRec_ObjPos>"
  (cl:let* ((signed (cl:slot-value msg 'objNum)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'trans))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <objRec_ObjPos>) istream)
  "Deserializes a message object of type '<objRec_ObjPos>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'objNum) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  (cl:setf (cl:slot-value msg 'trans) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'trans)))
    (cl:dotimes (i 3)
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<objRec_ObjPos>)))
  "Returns string type for a message object of type '<objRec_ObjPos>"
  "objRec_comm/objRec_ObjPos")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'objRec_ObjPos)))
  "Returns string type for a message object of type 'objRec_ObjPos"
  "objRec_comm/objRec_ObjPos")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<objRec_ObjPos>)))
  "Returns md5sum for a message object of type '<objRec_ObjPos>"
  "bf14d19203cfc71676eab1bfd46375f9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'objRec_ObjPos)))
  "Returns md5sum for a message object of type 'objRec_ObjPos"
  "bf14d19203cfc71676eab1bfd46375f9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<objRec_ObjPos>)))
  "Returns full string definition for message of type '<objRec_ObjPos>"
  (cl:format cl:nil "# This message is the current pose of an object in the point cloud frame~%~%int64 objNum~%float64[3] trans~%float64[4] quat~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'objRec_ObjPos)))
  "Returns full string definition for message of type 'objRec_ObjPos"
  (cl:format cl:nil "# This message is the current pose of an object in the point cloud frame~%~%int64 objNum~%float64[3] trans~%float64[4] quat~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <objRec_ObjPos>))
  (cl:+ 0
     8
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'trans) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'quat) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <objRec_ObjPos>))
  "Converts a ROS message object to a list"
  (cl:list 'objRec_ObjPos
    (cl:cons ':objNum (objNum msg))
    (cl:cons ':trans (trans msg))
    (cl:cons ':quat (quat msg))
))
