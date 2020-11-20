; Auto-generated. Do not edit!


(cl:in-package inertial_sense-msg)


;//! \htmlinclude INL2States.msg.html

(cl:defclass <INL2States> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (quatEcef
    :reader quatEcef
    :initarg :quatEcef
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (velEcef
    :reader velEcef
    :initarg :velEcef
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (posEcef
    :reader posEcef
    :initarg :posEcef
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (gyroBias
    :reader gyroBias
    :initarg :gyroBias
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (accelBias
    :reader accelBias
    :initarg :accelBias
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (baroBias
    :reader baroBias
    :initarg :baroBias
    :type cl:float
    :initform 0.0)
   (magDec
    :reader magDec
    :initarg :magDec
    :type cl:float
    :initform 0.0)
   (magInc
    :reader magInc
    :initarg :magInc
    :type cl:float
    :initform 0.0))
)

(cl:defclass INL2States (<INL2States>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <INL2States>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'INL2States)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name inertial_sense-msg:<INL2States> is deprecated: use inertial_sense-msg:INL2States instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <INL2States>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:header-val is deprecated.  Use inertial_sense-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'quatEcef-val :lambda-list '(m))
(cl:defmethod quatEcef-val ((m <INL2States>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:quatEcef-val is deprecated.  Use inertial_sense-msg:quatEcef instead.")
  (quatEcef m))

(cl:ensure-generic-function 'velEcef-val :lambda-list '(m))
(cl:defmethod velEcef-val ((m <INL2States>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:velEcef-val is deprecated.  Use inertial_sense-msg:velEcef instead.")
  (velEcef m))

(cl:ensure-generic-function 'posEcef-val :lambda-list '(m))
(cl:defmethod posEcef-val ((m <INL2States>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:posEcef-val is deprecated.  Use inertial_sense-msg:posEcef instead.")
  (posEcef m))

(cl:ensure-generic-function 'gyroBias-val :lambda-list '(m))
(cl:defmethod gyroBias-val ((m <INL2States>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:gyroBias-val is deprecated.  Use inertial_sense-msg:gyroBias instead.")
  (gyroBias m))

(cl:ensure-generic-function 'accelBias-val :lambda-list '(m))
(cl:defmethod accelBias-val ((m <INL2States>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:accelBias-val is deprecated.  Use inertial_sense-msg:accelBias instead.")
  (accelBias m))

(cl:ensure-generic-function 'baroBias-val :lambda-list '(m))
(cl:defmethod baroBias-val ((m <INL2States>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:baroBias-val is deprecated.  Use inertial_sense-msg:baroBias instead.")
  (baroBias m))

(cl:ensure-generic-function 'magDec-val :lambda-list '(m))
(cl:defmethod magDec-val ((m <INL2States>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:magDec-val is deprecated.  Use inertial_sense-msg:magDec instead.")
  (magDec m))

(cl:ensure-generic-function 'magInc-val :lambda-list '(m))
(cl:defmethod magInc-val ((m <INL2States>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:magInc-val is deprecated.  Use inertial_sense-msg:magInc instead.")
  (magInc m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <INL2States>) ostream)
  "Serializes a message object of type '<INL2States>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'quatEcef) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velEcef) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'posEcef) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'gyroBias) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'accelBias) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'baroBias))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'magDec))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'magInc))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <INL2States>) istream)
  "Deserializes a message object of type '<INL2States>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'quatEcef) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velEcef) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'posEcef) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'gyroBias) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'accelBias) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'baroBias) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'magDec) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'magInc) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<INL2States>)))
  "Returns string type for a message object of type '<INL2States>"
  "inertial_sense/INL2States")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'INL2States)))
  "Returns string type for a message object of type 'INL2States"
  "inertial_sense/INL2States")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<INL2States>)))
  "Returns md5sum for a message object of type '<INL2States>"
  "06de6b8d1957718b78007390d5c6fc67")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'INL2States)))
  "Returns md5sum for a message object of type 'INL2States"
  "06de6b8d1957718b78007390d5c6fc67")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<INL2States>)))
  "Returns full string definition for message of type '<INL2States>"
  (cl:format cl:nil "Header header                       # GPS time of week (since Sunday morning) in seconds~%geometry_msgs/Quaternion quatEcef   # Quaternion body rotation with respect to ECEF~%geometry_msgs/Vector3 velEcef       # (m/s) Velocity in ECEF frame~%geometry_msgs/Vector3 posEcef       # (m) Position in ECEF frame~%geometry_msgs/Vector3 gyroBias      # (rad/s) Gyro bias~%geometry_msgs/Vector3 accelBias     # (m/s^2) Accelerometer bias~%float32 baroBias                    # (m) Barometer bias~%float32 magDec                      # (rad) Magnetic declination~%float32 magInc                      # (rad) Magnetic inclination~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'INL2States)))
  "Returns full string definition for message of type 'INL2States"
  (cl:format cl:nil "Header header                       # GPS time of week (since Sunday morning) in seconds~%geometry_msgs/Quaternion quatEcef   # Quaternion body rotation with respect to ECEF~%geometry_msgs/Vector3 velEcef       # (m/s) Velocity in ECEF frame~%geometry_msgs/Vector3 posEcef       # (m) Position in ECEF frame~%geometry_msgs/Vector3 gyroBias      # (rad/s) Gyro bias~%geometry_msgs/Vector3 accelBias     # (m/s^2) Accelerometer bias~%float32 baroBias                    # (m) Barometer bias~%float32 magDec                      # (rad) Magnetic declination~%float32 magInc                      # (rad) Magnetic inclination~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <INL2States>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'quatEcef))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velEcef))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'posEcef))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'gyroBias))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'accelBias))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <INL2States>))
  "Converts a ROS message object to a list"
  (cl:list 'INL2States
    (cl:cons ':header (header msg))
    (cl:cons ':quatEcef (quatEcef msg))
    (cl:cons ':velEcef (velEcef msg))
    (cl:cons ':posEcef (posEcef msg))
    (cl:cons ':gyroBias (gyroBias msg))
    (cl:cons ':accelBias (accelBias msg))
    (cl:cons ':baroBias (baroBias msg))
    (cl:cons ':magDec (magDec msg))
    (cl:cons ':magInc (magInc msg))
))
