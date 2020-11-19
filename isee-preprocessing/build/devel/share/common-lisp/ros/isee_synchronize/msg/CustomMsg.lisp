; Auto-generated. Do not edit!


(cl:in-package isee_synchronize-msg)


;//! \htmlinclude CustomMsg.msg.html

(cl:defclass <CustomMsg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (timebase
    :reader timebase
    :initarg :timebase
    :type cl:integer
    :initform 0)
   (point_num
    :reader point_num
    :initarg :point_num
    :type cl:integer
    :initform 0)
   (lidar_id
    :reader lidar_id
    :initarg :lidar_id
    :type cl:fixnum
    :initform 0)
   (rsvd
    :reader rsvd
    :initarg :rsvd
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (points
    :reader points
    :initarg :points
    :type (cl:vector isee_synchronize-msg:CustomPoint)
   :initform (cl:make-array 0 :element-type 'isee_synchronize-msg:CustomPoint :initial-element (cl:make-instance 'isee_synchronize-msg:CustomPoint))))
)

(cl:defclass CustomMsg (<CustomMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CustomMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CustomMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name isee_synchronize-msg:<CustomMsg> is deprecated: use isee_synchronize-msg:CustomMsg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CustomMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader isee_synchronize-msg:header-val is deprecated.  Use isee_synchronize-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'timebase-val :lambda-list '(m))
(cl:defmethod timebase-val ((m <CustomMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader isee_synchronize-msg:timebase-val is deprecated.  Use isee_synchronize-msg:timebase instead.")
  (timebase m))

(cl:ensure-generic-function 'point_num-val :lambda-list '(m))
(cl:defmethod point_num-val ((m <CustomMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader isee_synchronize-msg:point_num-val is deprecated.  Use isee_synchronize-msg:point_num instead.")
  (point_num m))

(cl:ensure-generic-function 'lidar_id-val :lambda-list '(m))
(cl:defmethod lidar_id-val ((m <CustomMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader isee_synchronize-msg:lidar_id-val is deprecated.  Use isee_synchronize-msg:lidar_id instead.")
  (lidar_id m))

(cl:ensure-generic-function 'rsvd-val :lambda-list '(m))
(cl:defmethod rsvd-val ((m <CustomMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader isee_synchronize-msg:rsvd-val is deprecated.  Use isee_synchronize-msg:rsvd instead.")
  (rsvd m))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <CustomMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader isee_synchronize-msg:points-val is deprecated.  Use isee_synchronize-msg:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CustomMsg>) ostream)
  "Serializes a message object of type '<CustomMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'timebase)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'timebase)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'timebase)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'timebase)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'timebase)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'timebase)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'timebase)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'timebase)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'point_num)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'point_num)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'point_num)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'point_num)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lidar_id)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'rsvd))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CustomMsg>) istream)
  "Deserializes a message object of type '<CustomMsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'timebase)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'timebase)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'timebase)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'timebase)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'timebase)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'timebase)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'timebase)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'timebase)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'point_num)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'point_num)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'point_num)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'point_num)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lidar_id)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'rsvd) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'rsvd)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'isee_synchronize-msg:CustomPoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CustomMsg>)))
  "Returns string type for a message object of type '<CustomMsg>"
  "isee_synchronize/CustomMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CustomMsg)))
  "Returns string type for a message object of type 'CustomMsg"
  "isee_synchronize/CustomMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CustomMsg>)))
  "Returns md5sum for a message object of type '<CustomMsg>"
  "aa196d9cdf97226174bd64be4d696f75")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CustomMsg)))
  "Returns md5sum for a message object of type 'CustomMsg"
  "aa196d9cdf97226174bd64be4d696f75")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CustomMsg>)))
  "Returns full string definition for message of type '<CustomMsg>"
  (cl:format cl:nil "# Livox publish pointcloud msg format.~%~%Header header             # ROS standard message header~%uint64 timebase           # The time of first point~%uint32 point_num          # Total number of pointclouds~%uint8  lidar_id           # Lidar device id number~%uint8[3]  rsvd            # Reserved use~%CustomPoint[] points      # Pointcloud data~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: isee_synchronize/CustomPoint~%# Livox costom pointcloud format.~%~%uint32 offset_time      # offset time relative to the base time~%float32 x               # X axis, unit:m~%float32 y               # Y axis, unit:m~%float32 z               # Z axis, unit:m~%uint8 reflectivity      # reflectivity, 0~~255~%uint8 line              # laser number in lidar~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CustomMsg)))
  "Returns full string definition for message of type 'CustomMsg"
  (cl:format cl:nil "# Livox publish pointcloud msg format.~%~%Header header             # ROS standard message header~%uint64 timebase           # The time of first point~%uint32 point_num          # Total number of pointclouds~%uint8  lidar_id           # Lidar device id number~%uint8[3]  rsvd            # Reserved use~%CustomPoint[] points      # Pointcloud data~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: isee_synchronize/CustomPoint~%# Livox costom pointcloud format.~%~%uint32 offset_time      # offset time relative to the base time~%float32 x               # X axis, unit:m~%float32 y               # Y axis, unit:m~%float32 z               # Z axis, unit:m~%uint8 reflectivity      # reflectivity, 0~~255~%uint8 line              # laser number in lidar~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CustomMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     4
     1
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'rsvd) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CustomMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'CustomMsg
    (cl:cons ':header (header msg))
    (cl:cons ':timebase (timebase msg))
    (cl:cons ':point_num (point_num msg))
    (cl:cons ':lidar_id (lidar_id msg))
    (cl:cons ':rsvd (rsvd msg))
    (cl:cons ':points (points msg))
))
