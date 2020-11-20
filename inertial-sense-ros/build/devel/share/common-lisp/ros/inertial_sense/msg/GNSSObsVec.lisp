; Auto-generated. Do not edit!


(cl:in-package inertial_sense-msg)


;//! \htmlinclude GNSSObsVec.msg.html

(cl:defclass <GNSSObsVec> (roslisp-msg-protocol:ros-message)
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
   (obs
    :reader obs
    :initarg :obs
    :type (cl:vector inertial_sense-msg:GNSSObservation)
   :initform (cl:make-array 0 :element-type 'inertial_sense-msg:GNSSObservation :initial-element (cl:make-instance 'inertial_sense-msg:GNSSObservation))))
)

(cl:defclass GNSSObsVec (<GNSSObsVec>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GNSSObsVec>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GNSSObsVec)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name inertial_sense-msg:<GNSSObsVec> is deprecated: use inertial_sense-msg:GNSSObsVec instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GNSSObsVec>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:header-val is deprecated.  Use inertial_sense-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <GNSSObsVec>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:time-val is deprecated.  Use inertial_sense-msg:time instead.")
  (time m))

(cl:ensure-generic-function 'obs-val :lambda-list '(m))
(cl:defmethod obs-val ((m <GNSSObsVec>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:obs-val is deprecated.  Use inertial_sense-msg:obs instead.")
  (obs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GNSSObsVec>) ostream)
  "Serializes a message object of type '<GNSSObsVec>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'time) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'obs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'obs))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GNSSObsVec>) istream)
  "Deserializes a message object of type '<GNSSObsVec>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'time) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'obs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'obs)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'inertial_sense-msg:GNSSObservation))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GNSSObsVec>)))
  "Returns string type for a message object of type '<GNSSObsVec>"
  "inertial_sense/GNSSObsVec")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GNSSObsVec)))
  "Returns string type for a message object of type 'GNSSObsVec"
  "inertial_sense/GNSSObsVec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GNSSObsVec>)))
  "Returns md5sum for a message object of type '<GNSSObsVec>"
  "d228e847dabfc151b595c858b8d03b94")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GNSSObsVec)))
  "Returns md5sum for a message object of type 'GNSSObsVec"
  "d228e847dabfc151b595c858b8d03b94")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GNSSObsVec>)))
  "Returns full string definition for message of type '<GNSSObsVec>"
  (cl:format cl:nil "std_msgs/Header header~%GTime time              # time of all contained observations (UTC Time w/o Leap Seconds)~%GNSSObservation[] obs   # Vector of observations~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: inertial_sense/GTime~%int64 time~%float64 sec~%================================================================================~%MSG: inertial_sense/GNSSObservation~%std_msgs/Header header~%GTime time              # time of observation~%uint8 sat 		# satellite number~%uint8 rcv 		# receiver number~%uint8 SNR 		# Signal Strength (0.25 dBHz)~%uint8 LLI 		# Loss-of-Lock Indicator (bit1=loss-of-lock, bit2=half-cycle-invalid)~%uint8 code 		# code indicator (BeiDou: CODE_L1I, Other: CODE_L1C )~%uint8 qualL 	# Estimated carrier phase measurement standard deviation (0.004 cycles)~%uint8 qualP 	# Estimated pseudorange measurement standard deviation (0.01 m)~%float64 L      	# observation data carrier-phase (cycle)~%float64 P      	# observation data pseudorange (m)~%float32 D      	# observation data doppler frequency (0.002 Hz)~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GNSSObsVec)))
  "Returns full string definition for message of type 'GNSSObsVec"
  (cl:format cl:nil "std_msgs/Header header~%GTime time              # time of all contained observations (UTC Time w/o Leap Seconds)~%GNSSObservation[] obs   # Vector of observations~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: inertial_sense/GTime~%int64 time~%float64 sec~%================================================================================~%MSG: inertial_sense/GNSSObservation~%std_msgs/Header header~%GTime time              # time of observation~%uint8 sat 		# satellite number~%uint8 rcv 		# receiver number~%uint8 SNR 		# Signal Strength (0.25 dBHz)~%uint8 LLI 		# Loss-of-Lock Indicator (bit1=loss-of-lock, bit2=half-cycle-invalid)~%uint8 code 		# code indicator (BeiDou: CODE_L1I, Other: CODE_L1C )~%uint8 qualL 	# Estimated carrier phase measurement standard deviation (0.004 cycles)~%uint8 qualP 	# Estimated pseudorange measurement standard deviation (0.01 m)~%float64 L      	# observation data carrier-phase (cycle)~%float64 P      	# observation data pseudorange (m)~%float32 D      	# observation data doppler frequency (0.002 Hz)~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GNSSObsVec>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'time))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'obs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GNSSObsVec>))
  "Converts a ROS message object to a list"
  (cl:list 'GNSSObsVec
    (cl:cons ':header (header msg))
    (cl:cons ':time (time msg))
    (cl:cons ':obs (obs msg))
))
