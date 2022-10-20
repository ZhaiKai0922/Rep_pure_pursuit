; Auto-generated. Do not edit!


(cl:in-package clean_avoidance-msg)


;//! \htmlinclude CleanAvoidanceGoal.msg.html

(cl:defclass <CleanAvoidanceGoal> (roslisp-msg-protocol:ros-message)
  ((goal_x
    :reader goal_x
    :initarg :goal_x
    :type cl:float
    :initform 0.0)
   (goal_y
    :reader goal_y
    :initarg :goal_y
    :type cl:float
    :initform 0.0))
)

(cl:defclass CleanAvoidanceGoal (<CleanAvoidanceGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CleanAvoidanceGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CleanAvoidanceGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name clean_avoidance-msg:<CleanAvoidanceGoal> is deprecated: use clean_avoidance-msg:CleanAvoidanceGoal instead.")))

(cl:ensure-generic-function 'goal_x-val :lambda-list '(m))
(cl:defmethod goal_x-val ((m <CleanAvoidanceGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clean_avoidance-msg:goal_x-val is deprecated.  Use clean_avoidance-msg:goal_x instead.")
  (goal_x m))

(cl:ensure-generic-function 'goal_y-val :lambda-list '(m))
(cl:defmethod goal_y-val ((m <CleanAvoidanceGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clean_avoidance-msg:goal_y-val is deprecated.  Use clean_avoidance-msg:goal_y instead.")
  (goal_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CleanAvoidanceGoal>) ostream)
  "Serializes a message object of type '<CleanAvoidanceGoal>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'goal_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'goal_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CleanAvoidanceGoal>) istream)
  "Deserializes a message object of type '<CleanAvoidanceGoal>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'goal_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'goal_y) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CleanAvoidanceGoal>)))
  "Returns string type for a message object of type '<CleanAvoidanceGoal>"
  "clean_avoidance/CleanAvoidanceGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CleanAvoidanceGoal)))
  "Returns string type for a message object of type 'CleanAvoidanceGoal"
  "clean_avoidance/CleanAvoidanceGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CleanAvoidanceGoal>)))
  "Returns md5sum for a message object of type '<CleanAvoidanceGoal>"
  "13b965790a69b58a06825eaf607cdbe7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CleanAvoidanceGoal)))
  "Returns md5sum for a message object of type 'CleanAvoidanceGoal"
  "13b965790a69b58a06825eaf607cdbe7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CleanAvoidanceGoal>)))
  "Returns full string definition for message of type '<CleanAvoidanceGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal definition~%float64 goal_x~%float64 goal_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CleanAvoidanceGoal)))
  "Returns full string definition for message of type 'CleanAvoidanceGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal definition~%float64 goal_x~%float64 goal_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CleanAvoidanceGoal>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CleanAvoidanceGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'CleanAvoidanceGoal
    (cl:cons ':goal_x (goal_x msg))
    (cl:cons ':goal_y (goal_y msg))
))
