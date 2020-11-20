; Auto-generated. Do not edit!


(cl:in-package inertial_sense-msg)


;//! \htmlinclude GNSSObservation.msg.html

(cl:defclass <GNSSObservation> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (time
    :reader time
    :initarg :time
    :type inertial_sense-msg:GTime
    :initform (cl:make-instance 'inertial_sense-msg:GTime))
   (sat
    :reader sat
    :initarg :sat
    :type cl:fixnum
    :initform 0)
   (rcv
    :reader rcv
    :initarg :rcv
    :type cl:fixnum
    :initform 0)
   (SNR
    :reader SNR
    :initarg :SNR
    :type cl:fixnum
    :initform 0)
   (LLI
    :reader LLI
    :initarg :LLI
    :type cl:fixnum
    :initform 0)
   (code
    :reader code
    :initarg :code
    :type cl:fixnum
    :initform 0)
   (qualL
    :reader qualL
    :initarg :qualL
    :type cl:fixnum
    :initform 0)
   (qualP
    :reader qualP
    :initarg :qualP
    :type cl:fixnum
    :initform 0)
   (L
    :reader L
    :initarg :L
    :type cl:float
    :initform 0.0)
   (P
    :reader P
    :initarg :P
    :type cl:float
    :initform 0.0)
   (D
    :reader D
    :initarg :D
    :type cl:float
    :initform 0.0))
)

(cl:defclass GNSSObservation (<GNSSObservation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GNSSObservation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GNSSObservation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name inertial_sense-msg:<GNSSObservation> is deprecated: use inertial_sense-msg:GNSSObservation instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GNSSObservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:header-val is deprecated.  Use inertial_sense-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <GNSSObservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:time-val is deprecated.  Use inertial_sense-msg:time instead.")
  (time m))

(cl:ensure-generic-function 'sat-val :lambda-list '(m))
(cl:defmethod sat-val ((m <GNSSObservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:sat-val is deprecated.  Use inertial_sense-msg:sat instead.")
  (sat m))

(cl:ensure-generic-function 'rcv-val :lambda-list '(m))
(cl:defmethod rcv-val ((m <GNSSObservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:rcv-val is deprecated.  Use inertial_sense-msg:rcv instead.")
  (rcv m))

(cl:ensure-generic-function 'SNR-val :lambda-list '(m))
(cl:defmethod SNR-val ((m <GNSSObservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:SNR-val is deprecated.  Use inertial_sense-msg:SNR instead.")
  (SNR m))

(cl:ensure-generic-function 'LLI-val :lambda-list '(m))
(cl:defmethod LLI-val ((m <GNSSObservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:LLI-val is deprecated.  Use inertial_sense-msg:LLI instead.")
  (LLI m))

(cl:ensure-generic-function 'code-val :lambda-list '(m))
(cl:defmethod code-val ((m <GNSSObservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:code-val is deprecated.  Use inertial_sense-msg:code instead.")
  (code m))

(cl:ensure-generic-function 'qualL-val :lambda-list '(m))
(cl:defmethod qualL-val ((m <GNSSObservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:qualL-val is deprecated.  Use inertial_sense-msg:qualL instead.")
  (qualL m))

(cl:ensure-generic-function 'qualP-val :lambda-list '(m))
(cl:defmethod qualP-val ((m <GNSSObservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:qualP-val is deprecated.  Use inertial_sense-msg:qualP instead.")
  (qualP m))

(cl:ensure-generic-function 'L-val :lambda-list '(m))
(cl:defmethod L-val ((m <GNSSObservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:L-val is deprecated.  Use inertial_sense-msg:L instead.")
  (L m))

(cl:ensure-generic-function 'P-val :lambda-list '(m))
(cl:defmethod P-val ((m <GNSSObservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:P-val is deprecated.  Use inertial_sense-msg:P instead.")
  (P m))

(cl:ensure-generic-function 'D-val :lambda-list '(m))
(cl:defmethod D-val ((m <GNSSObservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:D-val is deprecated.  Use inertial_sense-msg:D instead.")
  (D m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GNSSObservation>) ostream)
  "Serializes a message object of type '<GNSSObservation>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'time) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sat)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rcv)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'SNR)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'LLI)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'code)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'qualL)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'qualP)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'L))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'P))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'D))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GNSSObservation>) istream)
  "Deserializes a message object of type '<GNSSObservation>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'time) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sat)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rcv)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'SNR)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'LLI)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'code)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'qualL)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'qualP)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'L) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'P) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'D) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GNSSObservation>)))
  "Returns string type for a message object of type '<GNSSObservation>"
  "inertial_sense/GNSSObservation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GNSSObservation)))
  "Returns string type for a message object of type 'GNSSObservation"
  "inertial_sense/GNSSObservation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GNSSObservation>)))
  "Returns md5sum for a message object of type '<GNSSObservation>"
  "f652831660ce8b4781ba3cf83655ca76")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GNSSObservation)))
  "Returns md5sum for a message object of type 'GNSSObservation"
  "f652831660ce8b4781ba3cf83655ca76")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GNSSObservation>)))
  "Returns full string definition for message of type '<GNSSObservation>"
  (cl:format cl:nil "std_msgs/Header header~%GTime time              # time of observation~%uint8 sat 		# satellite number~%uint8 rcv 		# receiver number~%uint8 SNR 		# Signal Strength (0.25 dBHz)~%uint8 LLI 		# Loss-of-Lock Indicator (bit1=loss-of-lock, bit2=half-cycle-invalid)~%uint8 code 		# code indicator (BeiDou: CODE_L1I, Other: CODE_L1C )~%uint8 qualL 	# Estimated carrier phase measurement standard deviation (0.004 cycles)~%uint8 qualP 	# Estimated pseudorange measurement standard deviation (0.01 m)~%float64 L      	# observation data carrier-phase (cycle)~%float64 P      	# observation data pseudorange (m)~%float32 D      	# observation data doppler frequency (0.002 Hz)~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: inertial_sense/GTime~%int64 time~%float64 sec~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GNSSObservation)))
  "Returns full string definition for message of type 'GNSSObservation"
  (cl:format cl:nil "std_msgs/Header header~%GTime time              # time of observation~%uint8 sat 		# satellite number~%uint8 rcv 		# receiver number~%uint8 SNR 		# Signal Strength (0.25 dBHz)~%uint8 LLI 		# Loss-of-Lock Indicator (bit1=loss-of-lock, bit2=half-cycle-invalid)~%uint8 code 		# code indicator (BeiDou: CODE_L1I, Other: CODE_L1C )~%uint8 qualL 	# Estimated carrier phase measurement standard deviation (0.004 cycles)~%uint8 qualP 	# Estimated pseudorange measurement standard deviation (0.01 m)~%float64 L      	# observation data carrier-phase (cycle)~%float64 P      	# observation data pseudorange (m)~%float32 D      	# observation data doppler frequency (0.002 Hz)~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: inertial_sense/GTime~%int64 time~%float64 sec~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GNSSObservation>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'time))
     1
     1
     1
     1
     1
     1
     1
     8
     8
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GNSSObservation>))
  "Converts a ROS message object to a list"
  (cl:list 'GNSSObservation
    (cl:cons ':header (header msg))
    (cl:cons ':time (time msg))
    (cl:cons ':sat (sat msg))
    (cl:cons ':rcv (rcv msg))
    (cl:cons ':SNR (SNR msg))
    (cl:cons ':LLI (LLI msg))
    (cl:cons ':code (code msg))
    (cl:cons ':qualL (qualL msg))
    (cl:cons ':qualP (qualP msg))
    (cl:cons ':L (L msg))
    (cl:cons ':P (P msg))
    (cl:cons ':D (D msg))
))
