; Auto-generated. Do not edit!


(cl:in-package inertial_sense-msg)


;//! \htmlinclude GPS.msg.html

(cl:defclass <GPS> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (num_sat
    :reader num_sat
    :initarg :num_sat
    :type cl:fixnum
    :initform 0)
   (fix_type
    :reader fix_type
    :initarg :fix_type
    :type cl:integer
    :initform 0)
   (cno
    :reader cno
    :initarg :cno
    :type cl:integer
    :initform 0)
   (latitude
    :reader latitude
    :initarg :latitude
    :type cl:float
    :initform 0.0)
   (longitude
    :reader longitude
    :initarg :longitude
    :type cl:float
    :initform 0.0)
   (altitude
    :reader altitude
    :initarg :altitude
    :type cl:float
    :initform 0.0)
   (posEcef
    :reader posEcef
    :initarg :posEcef
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (velEcef
    :reader velEcef
    :initarg :velEcef
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (hMSL
    :reader hMSL
    :initarg :hMSL
    :type cl:float
    :initform 0.0)
   (hAcc
    :reader hAcc
    :initarg :hAcc
    :type cl:float
    :initform 0.0)
   (vAcc
    :reader vAcc
    :initarg :vAcc
    :type cl:float
    :initform 0.0)
   (sAcc
    :reader sAcc
    :initarg :sAcc
    :type cl:float
    :initform 0.0)
   (pDop
    :reader pDop
    :initarg :pDop
    :type cl:float
    :initform 0.0))
)

(cl:defclass GPS (<GPS>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GPS>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GPS)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name inertial_sense-msg:<GPS> is deprecated: use inertial_sense-msg:GPS instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GPS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:header-val is deprecated.  Use inertial_sense-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'num_sat-val :lambda-list '(m))
(cl:defmethod num_sat-val ((m <GPS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:num_sat-val is deprecated.  Use inertial_sense-msg:num_sat instead.")
  (num_sat m))

(cl:ensure-generic-function 'fix_type-val :lambda-list '(m))
(cl:defmethod fix_type-val ((m <GPS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:fix_type-val is deprecated.  Use inertial_sense-msg:fix_type instead.")
  (fix_type m))

(cl:ensure-generic-function 'cno-val :lambda-list '(m))
(cl:defmethod cno-val ((m <GPS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:cno-val is deprecated.  Use inertial_sense-msg:cno instead.")
  (cno m))

(cl:ensure-generic-function 'latitude-val :lambda-list '(m))
(cl:defmethod latitude-val ((m <GPS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:latitude-val is deprecated.  Use inertial_sense-msg:latitude instead.")
  (latitude m))

(cl:ensure-generic-function 'longitude-val :lambda-list '(m))
(cl:defmethod longitude-val ((m <GPS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:longitude-val is deprecated.  Use inertial_sense-msg:longitude instead.")
  (longitude m))

(cl:ensure-generic-function 'altitude-val :lambda-list '(m))
(cl:defmethod altitude-val ((m <GPS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:altitude-val is deprecated.  Use inertial_sense-msg:altitude instead.")
  (altitude m))

(cl:ensure-generic-function 'posEcef-val :lambda-list '(m))
(cl:defmethod posEcef-val ((m <GPS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:posEcef-val is deprecated.  Use inertial_sense-msg:posEcef instead.")
  (posEcef m))

(cl:ensure-generic-function 'velEcef-val :lambda-list '(m))
(cl:defmethod velEcef-val ((m <GPS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:velEcef-val is deprecated.  Use inertial_sense-msg:velEcef instead.")
  (velEcef m))

(cl:ensure-generic-function 'hMSL-val :lambda-list '(m))
(cl:defmethod hMSL-val ((m <GPS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:hMSL-val is deprecated.  Use inertial_sense-msg:hMSL instead.")
  (hMSL m))

(cl:ensure-generic-function 'hAcc-val :lambda-list '(m))
(cl:defmethod hAcc-val ((m <GPS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:hAcc-val is deprecated.  Use inertial_sense-msg:hAcc instead.")
  (hAcc m))

(cl:ensure-generic-function 'vAcc-val :lambda-list '(m))
(cl:defmethod vAcc-val ((m <GPS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:vAcc-val is deprecated.  Use inertial_sense-msg:vAcc instead.")
  (vAcc m))

(cl:ensure-generic-function 'sAcc-val :lambda-list '(m))
(cl:defmethod sAcc-val ((m <GPS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:sAcc-val is deprecated.  Use inertial_sense-msg:sAcc instead.")
  (sAcc m))

(cl:ensure-generic-function 'pDop-val :lambda-list '(m))
(cl:defmethod pDop-val ((m <GPS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:pDop-val is deprecated.  Use inertial_sense-msg:pDop instead.")
  (pDop m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<GPS>)))
    "Constants for message type '<GPS>"
  '((:GPS_STATUS_FIX_TYPE_NO_FIX . 0)
    (:GPS_STATUS_FIX_TYPE_DEAD_RECKONING_ONLY . 256)
    (:GPS_STATUS_FIX_TYPE_2D_FIX . 512)
    (:GPS_STATUS_FIX_TYPE_3D_FIX . 768)
    (:GPS_STATUS_FIX_TYPE_GPS_PLUS_DEAD_RECK . 1024)
    (:GPS_STATUS_FIX_TYPE_TIME_ONLY_FIX . 1280)
    (:GPS_STATUS_FIX_TYPE_RESERVED1 . 1536)
    (:GPS_STATUS_FIX_TYPE_RESERVED2 . 1792)
    (:GPS_STATUS_FIX_STATUS_FIX_OK . 65536))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'GPS)))
    "Constants for message type 'GPS"
  '((:GPS_STATUS_FIX_TYPE_NO_FIX . 0)
    (:GPS_STATUS_FIX_TYPE_DEAD_RECKONING_ONLY . 256)
    (:GPS_STATUS_FIX_TYPE_2D_FIX . 512)
    (:GPS_STATUS_FIX_TYPE_3D_FIX . 768)
    (:GPS_STATUS_FIX_TYPE_GPS_PLUS_DEAD_RECK . 1024)
    (:GPS_STATUS_FIX_TYPE_TIME_ONLY_FIX . 1280)
    (:GPS_STATUS_FIX_TYPE_RESERVED1 . 1536)
    (:GPS_STATUS_FIX_TYPE_RESERVED2 . 1792)
    (:GPS_STATUS_FIX_STATUS_FIX_OK . 65536))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GPS>) ostream)
  "Serializes a message object of type '<GPS>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'num_sat)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fix_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'fix_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'fix_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'fix_type)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'cno)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'latitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'longitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'altitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'posEcef) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velEcef) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'hMSL))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'hAcc))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vAcc))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'sAcc))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pDop))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GPS>) istream)
  "Deserializes a message object of type '<GPS>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_sat) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fix_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'fix_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'fix_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'fix_type)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cno) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'latitude) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'longitude) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'altitude) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'posEcef) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velEcef) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'hMSL) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'hAcc) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vAcc) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'sAcc) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pDop) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GPS>)))
  "Returns string type for a message object of type '<GPS>"
  "inertial_sense/GPS")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GPS)))
  "Returns string type for a message object of type 'GPS"
  "inertial_sense/GPS")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GPS>)))
  "Returns md5sum for a message object of type '<GPS>"
  "6aa847d654b817ff4bb5ba8c773b2a17")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GPS)))
  "Returns md5sum for a message object of type 'GPS"
  "6aa847d654b817ff4bb5ba8c773b2a17")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GPS>)))
  "Returns full string definition for message of type '<GPS>"
  (cl:format cl:nil "# GPS status flags~%uint32 GPS_STATUS_FIX_TYPE_NO_FIX               = 0~%uint32 GPS_STATUS_FIX_TYPE_DEAD_RECKONING_ONLY  = 256~%uint32 GPS_STATUS_FIX_TYPE_2D_FIX               = 512~%uint32 GPS_STATUS_FIX_TYPE_3D_FIX               = 768~%uint32 GPS_STATUS_FIX_TYPE_GPS_PLUS_DEAD_RECK   = 1024~%uint32 GPS_STATUS_FIX_TYPE_TIME_ONLY_FIX        = 1280~%uint32 GPS_STATUS_FIX_TYPE_RESERVED1            = 1536~%uint32 GPS_STATUS_FIX_TYPE_RESERVED2            = 1792~%~%uint32 GPS_STATUS_FIX_STATUS_FIX_OK             = 65536~%~%Header header~%int8 num_sat 							# Number of satellites used in solution~%uint32 fix_type 						# Fix type, one of STATUS_FIX_TYPE flags~%int32 cno 								# mean carrier noise ratio (dBHz)~%float64 latitude 						# latitude (degrees) ~%float64 longitude						# longitude (degrees)~%float64 altitude						# height above ellipsoid (not MSL) (m)~%geometry_msgs/Vector3 posEcef           # Position (m) in ECEF~%geometry_msgs/Vector3 velEcef       	# Velocity (m/s) in ECEF~%float32 hMSL							# height above MSL~%float32 hAcc							# horizontal accuracy~%float32 vAcc							# vertical accuracy~%float32 sAcc							# speed accuracy (m/s)~%float32 pDop							# Position Dilution of Precision (m)	~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GPS)))
  "Returns full string definition for message of type 'GPS"
  (cl:format cl:nil "# GPS status flags~%uint32 GPS_STATUS_FIX_TYPE_NO_FIX               = 0~%uint32 GPS_STATUS_FIX_TYPE_DEAD_RECKONING_ONLY  = 256~%uint32 GPS_STATUS_FIX_TYPE_2D_FIX               = 512~%uint32 GPS_STATUS_FIX_TYPE_3D_FIX               = 768~%uint32 GPS_STATUS_FIX_TYPE_GPS_PLUS_DEAD_RECK   = 1024~%uint32 GPS_STATUS_FIX_TYPE_TIME_ONLY_FIX        = 1280~%uint32 GPS_STATUS_FIX_TYPE_RESERVED1            = 1536~%uint32 GPS_STATUS_FIX_TYPE_RESERVED2            = 1792~%~%uint32 GPS_STATUS_FIX_STATUS_FIX_OK             = 65536~%~%Header header~%int8 num_sat 							# Number of satellites used in solution~%uint32 fix_type 						# Fix type, one of STATUS_FIX_TYPE flags~%int32 cno 								# mean carrier noise ratio (dBHz)~%float64 latitude 						# latitude (degrees) ~%float64 longitude						# longitude (degrees)~%float64 altitude						# height above ellipsoid (not MSL) (m)~%geometry_msgs/Vector3 posEcef           # Position (m) in ECEF~%geometry_msgs/Vector3 velEcef       	# Velocity (m/s) in ECEF~%float32 hMSL							# height above MSL~%float32 hAcc							# horizontal accuracy~%float32 vAcc							# vertical accuracy~%float32 sAcc							# speed accuracy (m/s)~%float32 pDop							# Position Dilution of Precision (m)	~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GPS>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4
     4
     8
     8
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'posEcef))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velEcef))
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GPS>))
  "Converts a ROS message object to a list"
  (cl:list 'GPS
    (cl:cons ':header (header msg))
    (cl:cons ':num_sat (num_sat msg))
    (cl:cons ':fix_type (fix_type msg))
    (cl:cons ':cno (cno msg))
    (cl:cons ':latitude (latitude msg))
    (cl:cons ':longitude (longitude msg))
    (cl:cons ':altitude (altitude msg))
    (cl:cons ':posEcef (posEcef msg))
    (cl:cons ':velEcef (velEcef msg))
    (cl:cons ':hMSL (hMSL msg))
    (cl:cons ':hAcc (hAcc msg))
    (cl:cons ':vAcc (vAcc msg))
    (cl:cons ':sAcc (sAcc msg))
    (cl:cons ':pDop (pDop msg))
))
