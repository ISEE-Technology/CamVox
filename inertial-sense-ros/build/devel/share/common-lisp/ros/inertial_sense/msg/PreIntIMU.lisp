; Auto-generated. Do not edit!


(cl:in-package inertial_sense-msg)


;//! \htmlinclude PreIntIMU.msg.html

(cl:defclass <PreIntIMU> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (dtheta
    :reader dtheta
    :initarg :dtheta
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (dvel
    :reader dvel
    :initarg :dvel
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (dt
    :reader dt
    :initarg :dt
    :type cl:float
    :initform 0.0))
)

(cl:defclass PreIntIMU (<PreIntIMU>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PreIntIMU>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PreIntIMU)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name inertial_sense-msg:<PreIntIMU> is deprecated: use inertial_sense-msg:PreIntIMU instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PreIntIMU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:header-val is deprecated.  Use inertial_sense-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'dtheta-val :lambda-list '(m))
(cl:defmethod dtheta-val ((m <PreIntIMU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:dtheta-val is deprecated.  Use inertial_sense-msg:dtheta instead.")
  (dtheta m))

(cl:ensure-generic-function 'dvel-val :lambda-list '(m))
(cl:defmethod dvel-val ((m <PreIntIMU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:dvel-val is deprecated.  Use inertial_sense-msg:dvel instead.")
  (dvel m))

(cl:ensure-generic-function 'dt-val :lambda-list '(m))
(cl:defmethod dt-val ((m <PreIntIMU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:dt-val is deprecated.  Use inertial_sense-msg:dt instead.")
  (dt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PreIntIMU>) ostream)
  "Serializes a message object of type '<PreIntIMU>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'dtheta) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'dvel) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'dt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PreIntIMU>) istream)
  "Deserializes a message object of type '<PreIntIMU>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'dtheta) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'dvel) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dt) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PreIntIMU>)))
  "Returns string type for a message object of type '<PreIntIMU>"
  "inertial_sense/PreIntIMU")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PreIntIMU)))
  "Returns string type for a message object of type 'PreIntIMU"
  "inertial_sense/PreIntIMU")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PreIntIMU>)))
  "Returns md5sum for a message object of type '<PreIntIMU>"
  "8cedd64d41ec2b2b45424a92ffe74e5a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PreIntIMU)))
  "Returns md5sum for a message object of type 'PreIntIMU"
  "8cedd64d41ec2b2b45424a92ffe74e5a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PreIntIMU>)))
  "Returns full string definition for message of type '<PreIntIMU>"
  (cl:format cl:nil "Header header~%geometry_msgs/Vector3 dtheta 	# change in angle over time period (rodriguez vector)~%geometry_msgs/Vector3 dvel		# change in velocity over time period (m/s)~%float64 dt 						# length of time period (s)~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PreIntIMU)))
  "Returns full string definition for message of type 'PreIntIMU"
  (cl:format cl:nil "Header header~%geometry_msgs/Vector3 dtheta 	# change in angle over time period (rodriguez vector)~%geometry_msgs/Vector3 dvel		# change in velocity over time period (m/s)~%float64 dt 						# length of time period (s)~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PreIntIMU>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'dtheta))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'dvel))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PreIntIMU>))
  "Converts a ROS message object to a list"
  (cl:list 'PreIntIMU
    (cl:cons ':header (header msg))
    (cl:cons ':dtheta (dtheta msg))
    (cl:cons ':dvel (dvel msg))
    (cl:cons ':dt (dt msg))
))
