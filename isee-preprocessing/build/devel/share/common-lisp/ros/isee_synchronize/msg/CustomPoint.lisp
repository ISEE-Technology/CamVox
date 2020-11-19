; Auto-generated. Do not edit!


(cl:in-package isee_synchronize-msg)


;//! \htmlinclude CustomPoint.msg.html

(cl:defclass <CustomPoint> (roslisp-msg-protocol:ros-message)
  ((offset_time
    :reader offset_time
    :initarg :offset_time
    :type cl:integer
    :initform 0)
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0)
   (reflectivity
    :reader reflectivity
    :initarg :reflectivity
    :type cl:fixnum
    :initform 0)
   (line
    :reader line
    :initarg :line
    :type cl:fixnum
    :initform 0))
)

(cl:defclass CustomPoint (<CustomPoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CustomPoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CustomPoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name isee_synchronize-msg:<CustomPoint> is deprecated: use isee_synchronize-msg:CustomPoint instead.")))

(cl:ensure-generic-function 'offset_time-val :lambda-list '(m))
(cl:defmethod offset_time-val ((m <CustomPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader isee_synchronize-msg:offset_time-val is deprecated.  Use isee_synchronize-msg:offset_time instead.")
  (offset_time m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <CustomPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader isee_synchronize-msg:x-val is deprecated.  Use isee_synchronize-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <CustomPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader isee_synchronize-msg:y-val is deprecated.  Use isee_synchronize-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <CustomPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader isee_synchronize-msg:z-val is deprecated.  Use isee_synchronize-msg:z instead.")
  (z m))

(cl:ensure-generic-function 'reflectivity-val :lambda-list '(m))
(cl:defmethod reflectivity-val ((m <CustomPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader isee_synchronize-msg:reflectivity-val is deprecated.  Use isee_synchronize-msg:reflectivity instead.")
  (reflectivity m))

(cl:ensure-generic-function 'line-val :lambda-list '(m))
(cl:defmethod line-val ((m <CustomPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader isee_synchronize-msg:line-val is deprecated.  Use isee_synchronize-msg:line instead.")
  (line m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CustomPoint>) ostream)
  "Serializes a message object of type '<CustomPoint>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'offset_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'offset_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'offset_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'offset_time)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'reflectivity)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'line)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CustomPoint>) istream)
  "Deserializes a message object of type '<CustomPoint>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'offset_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'offset_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'offset_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'offset_time)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'reflectivity)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'line)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CustomPoint>)))
  "Returns string type for a message object of type '<CustomPoint>"
  "isee_synchronize/CustomPoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CustomPoint)))
  "Returns string type for a message object of type 'CustomPoint"
  "isee_synchronize/CustomPoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CustomPoint>)))
  "Returns md5sum for a message object of type '<CustomPoint>"
  "d852cd86aa9d2b697229026ff07cf452")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CustomPoint)))
  "Returns md5sum for a message object of type 'CustomPoint"
  "d852cd86aa9d2b697229026ff07cf452")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CustomPoint>)))
  "Returns full string definition for message of type '<CustomPoint>"
  (cl:format cl:nil "# Livox costom pointcloud format.~%~%uint32 offset_time      # offset time relative to the base time~%float32 x               # X axis, unit:m~%float32 y               # Y axis, unit:m~%float32 z               # Z axis, unit:m~%uint8 reflectivity      # reflectivity, 0~~255~%uint8 line              # laser number in lidar~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CustomPoint)))
  "Returns full string definition for message of type 'CustomPoint"
  (cl:format cl:nil "# Livox costom pointcloud format.~%~%uint32 offset_time      # offset time relative to the base time~%float32 x               # X axis, unit:m~%float32 y               # Y axis, unit:m~%float32 z               # Z axis, unit:m~%uint8 reflectivity      # reflectivity, 0~~255~%uint8 line              # laser number in lidar~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CustomPoint>))
  (cl:+ 0
     4
     4
     4
     4
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CustomPoint>))
  "Converts a ROS message object to a list"
  (cl:list 'CustomPoint
    (cl:cons ':offset_time (offset_time msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':reflectivity (reflectivity msg))
    (cl:cons ':line (line msg))
))
