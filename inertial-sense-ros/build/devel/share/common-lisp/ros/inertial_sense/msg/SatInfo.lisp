; Auto-generated. Do not edit!


(cl:in-package inertial_sense-msg)


;//! \htmlinclude SatInfo.msg.html

(cl:defclass <SatInfo> (roslisp-msg-protocol:ros-message)
  ((sat_id
    :reader sat_id
    :initarg :sat_id
    :type cl:integer
    :initform 0)
   (cno
    :reader cno
    :initarg :cno
    :type cl:integer
    :initform 0))
)

(cl:defclass SatInfo (<SatInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SatInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SatInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name inertial_sense-msg:<SatInfo> is deprecated: use inertial_sense-msg:SatInfo instead.")))

(cl:ensure-generic-function 'sat_id-val :lambda-list '(m))
(cl:defmethod sat_id-val ((m <SatInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:sat_id-val is deprecated.  Use inertial_sense-msg:sat_id instead.")
  (sat_id m))

(cl:ensure-generic-function 'cno-val :lambda-list '(m))
(cl:defmethod cno-val ((m <SatInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:cno-val is deprecated.  Use inertial_sense-msg:cno instead.")
  (cno m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SatInfo>) ostream)
  "Serializes a message object of type '<SatInfo>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sat_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sat_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sat_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sat_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cno)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cno)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'cno)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'cno)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SatInfo>) istream)
  "Deserializes a message object of type '<SatInfo>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sat_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sat_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sat_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sat_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cno)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cno)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'cno)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'cno)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SatInfo>)))
  "Returns string type for a message object of type '<SatInfo>"
  "inertial_sense/SatInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SatInfo)))
  "Returns string type for a message object of type 'SatInfo"
  "inertial_sense/SatInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SatInfo>)))
  "Returns md5sum for a message object of type '<SatInfo>"
  "f1fb6b174b603bb921293910a7f10d63")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SatInfo)))
  "Returns md5sum for a message object of type 'SatInfo"
  "f1fb6b174b603bb921293910a7f10d63")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SatInfo>)))
  "Returns full string definition for message of type '<SatInfo>"
  (cl:format cl:nil "uint32 sat_id # sattelite id~%uint32 cno    # Carrier to noise ratio~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SatInfo)))
  "Returns full string definition for message of type 'SatInfo"
  (cl:format cl:nil "uint32 sat_id # sattelite id~%uint32 cno    # Carrier to noise ratio~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SatInfo>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SatInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'SatInfo
    (cl:cons ':sat_id (sat_id msg))
    (cl:cons ':cno (cno msg))
))
