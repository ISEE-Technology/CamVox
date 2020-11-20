; Auto-generated. Do not edit!


(cl:in-package inertial_sense-srv)


;//! \htmlinclude refLLAUpdate-request.msg.html

(cl:defclass <refLLAUpdate-request> (roslisp-msg-protocol:ros-message)
  ((lla
    :reader lla
    :initarg :lla
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass refLLAUpdate-request (<refLLAUpdate-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <refLLAUpdate-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'refLLAUpdate-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name inertial_sense-srv:<refLLAUpdate-request> is deprecated: use inertial_sense-srv:refLLAUpdate-request instead.")))

(cl:ensure-generic-function 'lla-val :lambda-list '(m))
(cl:defmethod lla-val ((m <refLLAUpdate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-srv:lla-val is deprecated.  Use inertial_sense-srv:lla instead.")
  (lla m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <refLLAUpdate-request>) ostream)
  "Serializes a message object of type '<refLLAUpdate-request>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'lla))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <refLLAUpdate-request>) istream)
  "Deserializes a message object of type '<refLLAUpdate-request>"
  (cl:setf (cl:slot-value msg 'lla) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'lla)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<refLLAUpdate-request>)))
  "Returns string type for a service object of type '<refLLAUpdate-request>"
  "inertial_sense/refLLAUpdateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'refLLAUpdate-request)))
  "Returns string type for a service object of type 'refLLAUpdate-request"
  "inertial_sense/refLLAUpdateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<refLLAUpdate-request>)))
  "Returns md5sum for a message object of type '<refLLAUpdate-request>"
  "b307ff2c781bd2350d5c9875e1ae9b16")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'refLLAUpdate-request)))
  "Returns md5sum for a message object of type 'refLLAUpdate-request"
  "b307ff2c781bd2350d5c9875e1ae9b16")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<refLLAUpdate-request>)))
  "Returns full string definition for message of type '<refLLAUpdate-request>"
  (cl:format cl:nil "float64[3] lla~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'refLLAUpdate-request)))
  "Returns full string definition for message of type 'refLLAUpdate-request"
  (cl:format cl:nil "float64[3] lla~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <refLLAUpdate-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'lla) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <refLLAUpdate-request>))
  "Converts a ROS message object to a list"
  (cl:list 'refLLAUpdate-request
    (cl:cons ':lla (lla msg))
))
;//! \htmlinclude refLLAUpdate-response.msg.html

(cl:defclass <refLLAUpdate-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass refLLAUpdate-response (<refLLAUpdate-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <refLLAUpdate-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'refLLAUpdate-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name inertial_sense-srv:<refLLAUpdate-response> is deprecated: use inertial_sense-srv:refLLAUpdate-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <refLLAUpdate-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-srv:success-val is deprecated.  Use inertial_sense-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <refLLAUpdate-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-srv:message-val is deprecated.  Use inertial_sense-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <refLLAUpdate-response>) ostream)
  "Serializes a message object of type '<refLLAUpdate-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <refLLAUpdate-response>) istream)
  "Deserializes a message object of type '<refLLAUpdate-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<refLLAUpdate-response>)))
  "Returns string type for a service object of type '<refLLAUpdate-response>"
  "inertial_sense/refLLAUpdateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'refLLAUpdate-response)))
  "Returns string type for a service object of type 'refLLAUpdate-response"
  "inertial_sense/refLLAUpdateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<refLLAUpdate-response>)))
  "Returns md5sum for a message object of type '<refLLAUpdate-response>"
  "b307ff2c781bd2350d5c9875e1ae9b16")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'refLLAUpdate-response)))
  "Returns md5sum for a message object of type 'refLLAUpdate-response"
  "b307ff2c781bd2350d5c9875e1ae9b16")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<refLLAUpdate-response>)))
  "Returns full string definition for message of type '<refLLAUpdate-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'refLLAUpdate-response)))
  "Returns full string definition for message of type 'refLLAUpdate-response"
  (cl:format cl:nil "bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <refLLAUpdate-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <refLLAUpdate-response>))
  "Converts a ROS message object to a list"
  (cl:list 'refLLAUpdate-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'refLLAUpdate)))
  'refLLAUpdate-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'refLLAUpdate)))
  'refLLAUpdate-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'refLLAUpdate)))
  "Returns string type for a service object of type '<refLLAUpdate>"
  "inertial_sense/refLLAUpdate")