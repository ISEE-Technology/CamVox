; Auto-generated. Do not edit!


(cl:in-package inertial_sense-srv)


;//! \htmlinclude FirmwareUpdate-request.msg.html

(cl:defclass <FirmwareUpdate-request> (roslisp-msg-protocol:ros-message)
  ((filename
    :reader filename
    :initarg :filename
    :type cl:string
    :initform ""))
)

(cl:defclass FirmwareUpdate-request (<FirmwareUpdate-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FirmwareUpdate-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FirmwareUpdate-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name inertial_sense-srv:<FirmwareUpdate-request> is deprecated: use inertial_sense-srv:FirmwareUpdate-request instead.")))

(cl:ensure-generic-function 'filename-val :lambda-list '(m))
(cl:defmethod filename-val ((m <FirmwareUpdate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-srv:filename-val is deprecated.  Use inertial_sense-srv:filename instead.")
  (filename m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FirmwareUpdate-request>) ostream)
  "Serializes a message object of type '<FirmwareUpdate-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'filename))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'filename))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FirmwareUpdate-request>) istream)
  "Deserializes a message object of type '<FirmwareUpdate-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'filename) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'filename) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FirmwareUpdate-request>)))
  "Returns string type for a service object of type '<FirmwareUpdate-request>"
  "inertial_sense/FirmwareUpdateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FirmwareUpdate-request)))
  "Returns string type for a service object of type 'FirmwareUpdate-request"
  "inertial_sense/FirmwareUpdateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FirmwareUpdate-request>)))
  "Returns md5sum for a message object of type '<FirmwareUpdate-request>"
  "582779c570de719ad3dde68b15a648a8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FirmwareUpdate-request)))
  "Returns md5sum for a message object of type 'FirmwareUpdate-request"
  "582779c570de719ad3dde68b15a648a8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FirmwareUpdate-request>)))
  "Returns full string definition for message of type '<FirmwareUpdate-request>"
  (cl:format cl:nil "string filename~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FirmwareUpdate-request)))
  "Returns full string definition for message of type 'FirmwareUpdate-request"
  (cl:format cl:nil "string filename~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FirmwareUpdate-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'filename))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FirmwareUpdate-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FirmwareUpdate-request
    (cl:cons ':filename (filename msg))
))
;//! \htmlinclude FirmwareUpdate-response.msg.html

(cl:defclass <FirmwareUpdate-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass FirmwareUpdate-response (<FirmwareUpdate-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FirmwareUpdate-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FirmwareUpdate-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name inertial_sense-srv:<FirmwareUpdate-response> is deprecated: use inertial_sense-srv:FirmwareUpdate-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <FirmwareUpdate-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-srv:success-val is deprecated.  Use inertial_sense-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <FirmwareUpdate-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-srv:message-val is deprecated.  Use inertial_sense-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FirmwareUpdate-response>) ostream)
  "Serializes a message object of type '<FirmwareUpdate-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FirmwareUpdate-response>) istream)
  "Deserializes a message object of type '<FirmwareUpdate-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FirmwareUpdate-response>)))
  "Returns string type for a service object of type '<FirmwareUpdate-response>"
  "inertial_sense/FirmwareUpdateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FirmwareUpdate-response)))
  "Returns string type for a service object of type 'FirmwareUpdate-response"
  "inertial_sense/FirmwareUpdateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FirmwareUpdate-response>)))
  "Returns md5sum for a message object of type '<FirmwareUpdate-response>"
  "582779c570de719ad3dde68b15a648a8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FirmwareUpdate-response)))
  "Returns md5sum for a message object of type 'FirmwareUpdate-response"
  "582779c570de719ad3dde68b15a648a8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FirmwareUpdate-response>)))
  "Returns full string definition for message of type '<FirmwareUpdate-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FirmwareUpdate-response)))
  "Returns full string definition for message of type 'FirmwareUpdate-response"
  (cl:format cl:nil "bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FirmwareUpdate-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FirmwareUpdate-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FirmwareUpdate-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FirmwareUpdate)))
  'FirmwareUpdate-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FirmwareUpdate)))
  'FirmwareUpdate-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FirmwareUpdate)))
  "Returns string type for a service object of type '<FirmwareUpdate>"
  "inertial_sense/FirmwareUpdate")