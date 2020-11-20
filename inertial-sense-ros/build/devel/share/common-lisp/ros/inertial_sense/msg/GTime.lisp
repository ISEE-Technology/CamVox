; Auto-generated. Do not edit!


(cl:in-package inertial_sense-msg)


;//! \htmlinclude GTime.msg.html

(cl:defclass <GTime> (roslisp-msg-protocol:ros-message)
  ((time
    :reader time
    :initarg :time
    :type cl:integer
    :initform 0)
   (sec
    :reader sec
    :initarg :sec
    :type cl:float
    :initform 0.0))
)

(cl:defclass GTime (<GTime>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GTime>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GTime)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name inertial_sense-msg:<GTime> is deprecated: use inertial_sense-msg:GTime instead.")))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <GTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:time-val is deprecated.  Use inertial_sense-msg:time instead.")
  (time m))

(cl:ensure-generic-function 'sec-val :lambda-list '(m))
(cl:defmethod sec-val ((m <GTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:sec-val is deprecated.  Use inertial_sense-msg:sec instead.")
  (sec m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GTime>) ostream)
  "Serializes a message object of type '<GTime>"
  (cl:let* ((signed (cl:slot-value msg 'time)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'sec))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GTime>) istream)
  "Deserializes a message object of type '<GTime>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'time) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'sec) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GTime>)))
  "Returns string type for a message object of type '<GTime>"
  "inertial_sense/GTime")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GTime)))
  "Returns string type for a message object of type 'GTime"
  "inertial_sense/GTime")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GTime>)))
  "Returns md5sum for a message object of type '<GTime>"
  "f27debe4939acc53511906e7b35f8f58")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GTime)))
  "Returns md5sum for a message object of type 'GTime"
  "f27debe4939acc53511906e7b35f8f58")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GTime>)))
  "Returns full string definition for message of type '<GTime>"
  (cl:format cl:nil "int64 time~%float64 sec~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GTime)))
  "Returns full string definition for message of type 'GTime"
  (cl:format cl:nil "int64 time~%float64 sec~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GTime>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GTime>))
  "Converts a ROS message object to a list"
  (cl:list 'GTime
    (cl:cons ':time (time msg))
    (cl:cons ':sec (sec msg))
))
