; Auto-generated. Do not edit!


(cl:in-package inertial_sense-msg)


;//! \htmlinclude GPSInfo.msg.html

(cl:defclass <GPSInfo> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (num_sats
    :reader num_sats
    :initarg :num_sats
    :type cl:integer
    :initform 0)
   (sattelite_info
    :reader sattelite_info
    :initarg :sattelite_info
    :type (cl:vector inertial_sense-msg:SatInfo)
   :initform (cl:make-array 50 :element-type 'inertial_sense-msg:SatInfo :initial-element (cl:make-instance 'inertial_sense-msg:SatInfo))))
)

(cl:defclass GPSInfo (<GPSInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GPSInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GPSInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name inertial_sense-msg:<GPSInfo> is deprecated: use inertial_sense-msg:GPSInfo instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GPSInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:header-val is deprecated.  Use inertial_sense-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'num_sats-val :lambda-list '(m))
(cl:defmethod num_sats-val ((m <GPSInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:num_sats-val is deprecated.  Use inertial_sense-msg:num_sats instead.")
  (num_sats m))

(cl:ensure-generic-function 'sattelite_info-val :lambda-list '(m))
(cl:defmethod sattelite_info-val ((m <GPSInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:sattelite_info-val is deprecated.  Use inertial_sense-msg:sattelite_info instead.")
  (sattelite_info m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GPSInfo>) ostream)
  "Serializes a message object of type '<GPSInfo>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_sats)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_sats)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_sats)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_sats)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'sattelite_info))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GPSInfo>) istream)
  "Deserializes a message object of type '<GPSInfo>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_sats)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_sats)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_sats)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_sats)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'sattelite_info) (cl:make-array 50))
  (cl:let ((vals (cl:slot-value msg 'sattelite_info)))
    (cl:dotimes (i 50)
    (cl:setf (cl:aref vals i) (cl:make-instance 'inertial_sense-msg:SatInfo))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GPSInfo>)))
  "Returns string type for a message object of type '<GPSInfo>"
  "inertial_sense/GPSInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GPSInfo)))
  "Returns string type for a message object of type 'GPSInfo"
  "inertial_sense/GPSInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GPSInfo>)))
  "Returns md5sum for a message object of type '<GPSInfo>"
  "a60e054a01708e390d6fe69c6b4a8303")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GPSInfo)))
  "Returns md5sum for a message object of type 'GPSInfo"
  "a60e054a01708e390d6fe69c6b4a8303")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GPSInfo>)))
  "Returns full string definition for message of type '<GPSInfo>"
  (cl:format cl:nil "~%Header header~%uint32 num_sats            		# number of sattelites in the sky~%SatInfo[50] sattelite_info	 	# information about sattelites~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: inertial_sense/SatInfo~%uint32 sat_id # sattelite id~%uint32 cno    # Carrier to noise ratio~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GPSInfo)))
  "Returns full string definition for message of type 'GPSInfo"
  (cl:format cl:nil "~%Header header~%uint32 num_sats            		# number of sattelites in the sky~%SatInfo[50] sattelite_info	 	# information about sattelites~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: inertial_sense/SatInfo~%uint32 sat_id # sattelite id~%uint32 cno    # Carrier to noise ratio~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GPSInfo>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'sattelite_info) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GPSInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'GPSInfo
    (cl:cons ':header (header msg))
    (cl:cons ':num_sats (num_sats msg))
    (cl:cons ':sattelite_info (sattelite_info msg))
))
