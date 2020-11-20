; Auto-generated. Do not edit!


(cl:in-package inertial_sense-msg)


;//! \htmlinclude RTKInfo.msg.html

(cl:defclass <RTKInfo> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (BaseLLA
    :reader BaseLLA
    :initarg :BaseLLA
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (cycle_slip_count
    :reader cycle_slip_count
    :initarg :cycle_slip_count
    :type cl:integer
    :initform 0)
   (roverObs
    :reader roverObs
    :initarg :roverObs
    :type cl:integer
    :initform 0)
   (baseObs
    :reader baseObs
    :initarg :baseObs
    :type cl:integer
    :initform 0)
   (roverEph
    :reader roverEph
    :initarg :roverEph
    :type cl:integer
    :initform 0)
   (baseEph
    :reader baseEph
    :initarg :baseEph
    :type cl:integer
    :initform 0)
   (baseAntcount
    :reader baseAntcount
    :initarg :baseAntcount
    :type cl:integer
    :initform 0))
)

(cl:defclass RTKInfo (<RTKInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RTKInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RTKInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name inertial_sense-msg:<RTKInfo> is deprecated: use inertial_sense-msg:RTKInfo instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RTKInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:header-val is deprecated.  Use inertial_sense-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'BaseLLA-val :lambda-list '(m))
(cl:defmethod BaseLLA-val ((m <RTKInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:BaseLLA-val is deprecated.  Use inertial_sense-msg:BaseLLA instead.")
  (BaseLLA m))

(cl:ensure-generic-function 'cycle_slip_count-val :lambda-list '(m))
(cl:defmethod cycle_slip_count-val ((m <RTKInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:cycle_slip_count-val is deprecated.  Use inertial_sense-msg:cycle_slip_count instead.")
  (cycle_slip_count m))

(cl:ensure-generic-function 'roverObs-val :lambda-list '(m))
(cl:defmethod roverObs-val ((m <RTKInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:roverObs-val is deprecated.  Use inertial_sense-msg:roverObs instead.")
  (roverObs m))

(cl:ensure-generic-function 'baseObs-val :lambda-list '(m))
(cl:defmethod baseObs-val ((m <RTKInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:baseObs-val is deprecated.  Use inertial_sense-msg:baseObs instead.")
  (baseObs m))

(cl:ensure-generic-function 'roverEph-val :lambda-list '(m))
(cl:defmethod roverEph-val ((m <RTKInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:roverEph-val is deprecated.  Use inertial_sense-msg:roverEph instead.")
  (roverEph m))

(cl:ensure-generic-function 'baseEph-val :lambda-list '(m))
(cl:defmethod baseEph-val ((m <RTKInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:baseEph-val is deprecated.  Use inertial_sense-msg:baseEph instead.")
  (baseEph m))

(cl:ensure-generic-function 'baseAntcount-val :lambda-list '(m))
(cl:defmethod baseAntcount-val ((m <RTKInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:baseAntcount-val is deprecated.  Use inertial_sense-msg:baseAntcount instead.")
  (baseAntcount m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RTKInfo>) ostream)
  "Serializes a message object of type '<RTKInfo>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'BaseLLA))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cycle_slip_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cycle_slip_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'cycle_slip_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'cycle_slip_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'roverObs)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'roverObs)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'roverObs)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'roverObs)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'baseObs)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'baseObs)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'baseObs)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'baseObs)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'roverEph)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'roverEph)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'roverEph)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'roverEph)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'baseEph)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'baseEph)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'baseEph)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'baseEph)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'baseAntcount)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'baseAntcount)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'baseAntcount)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'baseAntcount)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RTKInfo>) istream)
  "Deserializes a message object of type '<RTKInfo>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'BaseLLA) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'BaseLLA)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cycle_slip_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cycle_slip_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'cycle_slip_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'cycle_slip_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'roverObs)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'roverObs)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'roverObs)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'roverObs)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'baseObs)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'baseObs)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'baseObs)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'baseObs)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'roverEph)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'roverEph)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'roverEph)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'roverEph)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'baseEph)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'baseEph)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'baseEph)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'baseEph)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'baseAntcount)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'baseAntcount)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'baseAntcount)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'baseAntcount)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RTKInfo>)))
  "Returns string type for a message object of type '<RTKInfo>"
  "inertial_sense/RTKInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RTKInfo)))
  "Returns string type for a message object of type 'RTKInfo"
  "inertial_sense/RTKInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RTKInfo>)))
  "Returns md5sum for a message object of type '<RTKInfo>"
  "0f06cd1205181677917f42a40817ccb4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RTKInfo)))
  "Returns md5sum for a message object of type 'RTKInfo"
  "0f06cd1205181677917f42a40817ccb4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RTKInfo>)))
  "Returns full string definition for message of type '<RTKInfo>"
  (cl:format cl:nil "Header header~%~%float32[3] BaseLLA 			# base position in lat-lon-altitude (deg, deg, m)~%uint32 cycle_slip_count 	# number of cycle slips detected~%uint32 roverObs				# number of observations from rover (GPS, Glonass, Gallileo, Beidou, Qzs)~%uint32 baseObs				# number of observations from base (GPS, Glonass, Gallileo, Beidou, Qzs)~%uint32 roverEph				# number of ephemeris messages from rover (GPS, Glonass, Gallileo, Beidou, Qzs)~%uint32 baseEph				# number of ephemeris messages from rover (GPS, Glonass, Gallileo, Beidou, Qzs)~%uint32 baseAntcount			# number of base station antenna position measurements~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RTKInfo)))
  "Returns full string definition for message of type 'RTKInfo"
  (cl:format cl:nil "Header header~%~%float32[3] BaseLLA 			# base position in lat-lon-altitude (deg, deg, m)~%uint32 cycle_slip_count 	# number of cycle slips detected~%uint32 roverObs				# number of observations from rover (GPS, Glonass, Gallileo, Beidou, Qzs)~%uint32 baseObs				# number of observations from base (GPS, Glonass, Gallileo, Beidou, Qzs)~%uint32 roverEph				# number of ephemeris messages from rover (GPS, Glonass, Gallileo, Beidou, Qzs)~%uint32 baseEph				# number of ephemeris messages from rover (GPS, Glonass, Gallileo, Beidou, Qzs)~%uint32 baseAntcount			# number of base station antenna position measurements~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RTKInfo>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'BaseLLA) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RTKInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'RTKInfo
    (cl:cons ':header (header msg))
    (cl:cons ':BaseLLA (BaseLLA msg))
    (cl:cons ':cycle_slip_count (cycle_slip_count msg))
    (cl:cons ':roverObs (roverObs msg))
    (cl:cons ':baseObs (baseObs msg))
    (cl:cons ':roverEph (roverEph msg))
    (cl:cons ':baseEph (baseEph msg))
    (cl:cons ':baseAntcount (baseAntcount msg))
))
