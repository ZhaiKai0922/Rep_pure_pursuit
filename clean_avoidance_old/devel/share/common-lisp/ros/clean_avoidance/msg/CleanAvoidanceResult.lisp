; Auto-generated. Do not edit!


(cl:in-package clean_avoidance-msg)


;//! \htmlinclude CleanAvoidanceResult.msg.html

(cl:defclass <CleanAvoidanceResult> (roslisp-msg-protocol:ros-message)
  ((result_x
    :reader result_x
    :initarg :result_x
    :type cl:float
    :initform 0.0)
   (result_y
    :reader result_y
    :initarg :result_y
    :type cl:float
    :initform 0.0))
)

(cl:defclass CleanAvoidanceResult (<CleanAvoidanceResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CleanAvoidanceResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CleanAvoidanceResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name clean_avoidance-msg:<CleanAvoidanceResult> is deprecated: use clean_avoidance-msg:CleanAvoidanceResult instead.")))

(cl:ensure-generic-function 'result_x-val :lambda-list '(m))
(cl:defmethod result_x-val ((m <CleanAvoidanceResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clean_avoidance-msg:result_x-val is deprecated.  Use clean_avoidance-msg:result_x instead.")
  (result_x m))

(cl:ensure-generic-function 'result_y-val :lambda-list '(m))
(cl:defmethod result_y-val ((m <CleanAvoidanceResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clean_avoidance-msg:result_y-val is deprecated.  Use clean_avoidance-msg:result_y instead.")
  (result_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CleanAvoidanceResult>) ostream)
  "Serializes a message object of type '<CleanAvoidanceResult>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'result_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'result_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CleanAvoidanceResult>) istream)
  "Deserializes a message object of type '<CleanAvoidanceResult>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'result_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'result_y) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CleanAvoidanceResult>)))
  "Returns string type for a message object of type '<CleanAvoidanceResult>"
  "clean_avoidance/CleanAvoidanceResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CleanAvoidanceResult)))
  "Returns string type for a message object of type 'CleanAvoidanceResult"
  "clean_avoidance/CleanAvoidanceResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CleanAvoidanceResult>)))
  "Returns md5sum for a message object of type '<CleanAvoidanceResult>"
  "fe56f01890c8a9edde3f6efd85780c31")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CleanAvoidanceResult)))
  "Returns md5sum for a message object of type 'CleanAvoidanceResult"
  "fe56f01890c8a9edde3f6efd85780c31")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CleanAvoidanceResult>)))
  "Returns full string definition for message of type '<CleanAvoidanceResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#result definition~%float64 result_x~%float64 result_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CleanAvoidanceResult)))
  "Returns full string definition for message of type 'CleanAvoidanceResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#result definition~%float64 result_x~%float64 result_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CleanAvoidanceResult>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CleanAvoidanceResult>))
  "Converts a ROS message object to a list"
  (cl:list 'CleanAvoidanceResult
    (cl:cons ':result_x (result_x msg))
    (cl:cons ':result_y (result_y msg))
))
