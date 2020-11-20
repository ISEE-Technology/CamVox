; Auto-generated. Do not edit!


(cl:in-package inertial_sense-msg)


;//! \htmlinclude RTKRel.msg.html

(cl:defclass <RTKRel> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (differential_age
    :reader differential_age
    :initarg :differential_age
    :type cl:float
    :initform 0.0)
   (ar_ratio
    :reader ar_ratio
    :initarg :ar_ratio
    :type cl:float
    :initform 0.0)
   (vector_base_to_rover
    :reader vector_base_to_rover
    :initarg :vector_base_to_rover
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (distance_base_to_rover
    :reader distance_base_to_rover
    :initarg :distance_base_to_rover
    :type cl:float
    :initform 0.0)
   (heading_base_to_rover
    :reader heading_base_to_rover
    :initarg :heading_base_to_rover
    :type cl:float
    :initform 0.0))
)

(cl:defclass RTKRel (<RTKRel>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RTKRel>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RTKRel)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name inertial_sense-msg:<RTKRel> is deprecated: use inertial_sense-msg:RTKRel instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RTKRel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:header-val is deprecated.  Use inertial_sense-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'differential_age-val :lambda-list '(m))
(cl:defmethod differential_age-val ((m <RTKRel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:differential_age-val is deprecated.  Use inertial_sense-msg:differential_age instead.")
  (differential_age m))

(cl:ensure-generic-function 'ar_ratio-val :lambda-list '(m))
(cl:defmethod ar_ratio-val ((m <RTKRel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:ar_ratio-val is deprecated.  Use inertial_sense-msg:ar_ratio instead.")
  (ar_ratio m))

(cl:ensure-generic-function 'vector_base_to_rover-val :lambda-list '(m))
(cl:defmethod vector_base_to_rover-val ((m <RTKRel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:vector_base_to_rover-val is deprecated.  Use inertial_sense-msg:vector_base_to_rover instead.")
  (vector_base_to_rover m))

(cl:ensure-generic-function 'distance_base_to_rover-val :lambda-list '(m))
(cl:defmethod distance_base_to_rover-val ((m <RTKRel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:distance_base_to_rover-val is deprecated.  Use inertial_sense-msg:distance_base_to_rover instead.")
  (distance_base_to_rover m))

(cl:ensure-generic-function 'heading_base_to_rover-val :lambda-list '(m))
(cl:defmethod heading_base_to_rover-val ((m <RTKRel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:heading_base_to_rover-val is deprecated.  Use inertial_sense-msg:heading_base_to_rover instead.")
  (heading_base_to_rover m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RTKRel>) ostream)
  "Serializes a message object of type '<RTKRel>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'differential_age))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ar_ratio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'vector_base_to_rover) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance_base_to_rover))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'heading_base_to_rover))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RTKRel>) istream)
  "Deserializes a message object of type '<RTKRel>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'differential_age) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ar_ratio) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'vector_base_to_rover) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance_base_to_rover) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'heading_base_to_rover) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RTKRel>)))
  "Returns string type for a message object of type '<RTKRel>"
  "inertial_sense/RTKRel")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RTKRel)))
  "Returns string type for a message object of type 'RTKRel"
  "inertial_sense/RTKRel")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RTKRel>)))
  "Returns md5sum for a message object of type '<RTKRel>"
  "ff67521259c93bfc5ebd55747655bff6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RTKRel)))
  "Returns md5sum for a message object of type 'RTKRel"
  "ff67521259c93bfc5ebd55747655bff6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RTKRel>)))
  "Returns full string definition for message of type '<RTKRel>"
  (cl:format cl:nil "Header header~%float32 differential_age 				# Age of differential (seconds)~%float32 ar_ratio 						# Ambiguity resolution ratio factor for validation~%geometry_msgs/Vector3 vector_base_to_rover 	# Vector to base (m) in ECEF - If Compassing enabled, this is the 3-vector from antenna 2 to antenna 1~%float32 distance_base_to_rover 				# Distance to Base (m)~%float32 heading_base_to_rover 				# Angle from north to vectorToBase in local tangent plane. (rad)~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RTKRel)))
  "Returns full string definition for message of type 'RTKRel"
  (cl:format cl:nil "Header header~%float32 differential_age 				# Age of differential (seconds)~%float32 ar_ratio 						# Ambiguity resolution ratio factor for validation~%geometry_msgs/Vector3 vector_base_to_rover 	# Vector to base (m) in ECEF - If Compassing enabled, this is the 3-vector from antenna 2 to antenna 1~%float32 distance_base_to_rover 				# Distance to Base (m)~%float32 heading_base_to_rover 				# Angle from north to vectorToBase in local tangent plane. (rad)~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RTKRel>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'vector_base_to_rover))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RTKRel>))
  "Converts a ROS message object to a list"
  (cl:list 'RTKRel
    (cl:cons ':header (header msg))
    (cl:cons ':differential_age (differential_age msg))
    (cl:cons ':ar_ratio (ar_ratio msg))
    (cl:cons ':vector_base_to_rover (vector_base_to_rover msg))
    (cl:cons ':distance_base_to_rover (distance_base_to_rover msg))
    (cl:cons ':heading_base_to_rover (heading_base_to_rover msg))
))
