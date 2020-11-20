; Auto-generated. Do not edit!


(cl:in-package inertial_sense-msg)


;//! \htmlinclude GlonassEphemeris.msg.html

(cl:defclass <GlonassEphemeris> (roslisp-msg-protocol:ros-message)
  ((sat
    :reader sat
    :initarg :sat
    :type cl:integer
    :initform 0)
   (iode
    :reader iode
    :initarg :iode
    :type cl:integer
    :initform 0)
   (frq
    :reader frq
    :initarg :frq
    :type cl:integer
    :initform 0)
   (svh
    :reader svh
    :initarg :svh
    :type cl:integer
    :initform 0)
   (sva
    :reader sva
    :initarg :sva
    :type cl:integer
    :initform 0)
   (age
    :reader age
    :initarg :age
    :type cl:integer
    :initform 0)
   (toe
    :reader toe
    :initarg :toe
    :type inertial_sense-msg:GTime
    :initform (cl:make-instance 'inertial_sense-msg:GTime))
   (tof
    :reader tof
    :initarg :tof
    :type inertial_sense-msg:GTime
    :initform (cl:make-instance 'inertial_sense-msg:GTime))
   (pos
    :reader pos
    :initarg :pos
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (vel
    :reader vel
    :initarg :vel
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (acc
    :reader acc
    :initarg :acc
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (taun
    :reader taun
    :initarg :taun
    :type cl:float
    :initform 0.0)
   (gamn
    :reader gamn
    :initarg :gamn
    :type cl:float
    :initform 0.0)
   (dtaun
    :reader dtaun
    :initarg :dtaun
    :type cl:float
    :initform 0.0))
)

(cl:defclass GlonassEphemeris (<GlonassEphemeris>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GlonassEphemeris>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GlonassEphemeris)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name inertial_sense-msg:<GlonassEphemeris> is deprecated: use inertial_sense-msg:GlonassEphemeris instead.")))

(cl:ensure-generic-function 'sat-val :lambda-list '(m))
(cl:defmethod sat-val ((m <GlonassEphemeris>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:sat-val is deprecated.  Use inertial_sense-msg:sat instead.")
  (sat m))

(cl:ensure-generic-function 'iode-val :lambda-list '(m))
(cl:defmethod iode-val ((m <GlonassEphemeris>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:iode-val is deprecated.  Use inertial_sense-msg:iode instead.")
  (iode m))

(cl:ensure-generic-function 'frq-val :lambda-list '(m))
(cl:defmethod frq-val ((m <GlonassEphemeris>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:frq-val is deprecated.  Use inertial_sense-msg:frq instead.")
  (frq m))

(cl:ensure-generic-function 'svh-val :lambda-list '(m))
(cl:defmethod svh-val ((m <GlonassEphemeris>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:svh-val is deprecated.  Use inertial_sense-msg:svh instead.")
  (svh m))

(cl:ensure-generic-function 'sva-val :lambda-list '(m))
(cl:defmethod sva-val ((m <GlonassEphemeris>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:sva-val is deprecated.  Use inertial_sense-msg:sva instead.")
  (sva m))

(cl:ensure-generic-function 'age-val :lambda-list '(m))
(cl:defmethod age-val ((m <GlonassEphemeris>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:age-val is deprecated.  Use inertial_sense-msg:age instead.")
  (age m))

(cl:ensure-generic-function 'toe-val :lambda-list '(m))
(cl:defmethod toe-val ((m <GlonassEphemeris>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:toe-val is deprecated.  Use inertial_sense-msg:toe instead.")
  (toe m))

(cl:ensure-generic-function 'tof-val :lambda-list '(m))
(cl:defmethod tof-val ((m <GlonassEphemeris>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:tof-val is deprecated.  Use inertial_sense-msg:tof instead.")
  (tof m))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <GlonassEphemeris>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:pos-val is deprecated.  Use inertial_sense-msg:pos instead.")
  (pos m))

(cl:ensure-generic-function 'vel-val :lambda-list '(m))
(cl:defmethod vel-val ((m <GlonassEphemeris>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:vel-val is deprecated.  Use inertial_sense-msg:vel instead.")
  (vel m))

(cl:ensure-generic-function 'acc-val :lambda-list '(m))
(cl:defmethod acc-val ((m <GlonassEphemeris>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:acc-val is deprecated.  Use inertial_sense-msg:acc instead.")
  (acc m))

(cl:ensure-generic-function 'taun-val :lambda-list '(m))
(cl:defmethod taun-val ((m <GlonassEphemeris>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:taun-val is deprecated.  Use inertial_sense-msg:taun instead.")
  (taun m))

(cl:ensure-generic-function 'gamn-val :lambda-list '(m))
(cl:defmethod gamn-val ((m <GlonassEphemeris>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:gamn-val is deprecated.  Use inertial_sense-msg:gamn instead.")
  (gamn m))

(cl:ensure-generic-function 'dtaun-val :lambda-list '(m))
(cl:defmethod dtaun-val ((m <GlonassEphemeris>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader inertial_sense-msg:dtaun-val is deprecated.  Use inertial_sense-msg:dtaun instead.")
  (dtaun m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GlonassEphemeris>) ostream)
  "Serializes a message object of type '<GlonassEphemeris>"
  (cl:let* ((signed (cl:slot-value msg 'sat)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'iode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'frq)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'svh)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'sva)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'age)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'toe) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'tof) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'pos))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'vel))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'acc))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'taun))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'gamn))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'dtaun))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GlonassEphemeris>) istream)
  "Deserializes a message object of type '<GlonassEphemeris>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sat) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'iode) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'frq) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'svh) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sva) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'age) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'toe) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'tof) istream)
  (cl:setf (cl:slot-value msg 'pos) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'pos)))
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
  (cl:setf (cl:slot-value msg 'vel) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'vel)))
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
  (cl:setf (cl:slot-value msg 'acc) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'acc)))
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'taun) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gamn) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dtaun) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GlonassEphemeris>)))
  "Returns string type for a message object of type '<GlonassEphemeris>"
  "inertial_sense/GlonassEphemeris")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GlonassEphemeris)))
  "Returns string type for a message object of type 'GlonassEphemeris"
  "inertial_sense/GlonassEphemeris")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GlonassEphemeris>)))
  "Returns md5sum for a message object of type '<GlonassEphemeris>"
  "d6e50080404485ddd899caa0ddef8be5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GlonassEphemeris)))
  "Returns md5sum for a message object of type 'GlonassEphemeris"
  "d6e50080404485ddd899caa0ddef8be5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GlonassEphemeris>)))
  "Returns full string definition for message of type '<GlonassEphemeris>"
  (cl:format cl:nil "int32 sat 	# satellite number ~%int32 iode 	# IODE (0-6 bit of tb field) ~%int32 frq 	# satellite frequency number ~%int32 svh 	# satellite health ~%int32 sva 	# satellite accuracy ~%int32 age 	# satellite age of operation ~%GTime toe 	# epoch of epherides (gpst) ~%GTime tof 	# message frame time (gpst) ~%float64[3] pos 	# satellite position (ecef) (m) ~%float64[3] vel 	# satellite velocity (ecef) (m/s) ~%float64[3] acc 	# satellite acceleration (ecef) (m/s^2) ~%float64 taun 	# SV clock bias (s) ~%float64 gamn 	# relative freq bias ~%float64 dtaun 	# delay between L1 and L2 (s) ~%================================================================================~%MSG: inertial_sense/GTime~%int64 time~%float64 sec~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GlonassEphemeris)))
  "Returns full string definition for message of type 'GlonassEphemeris"
  (cl:format cl:nil "int32 sat 	# satellite number ~%int32 iode 	# IODE (0-6 bit of tb field) ~%int32 frq 	# satellite frequency number ~%int32 svh 	# satellite health ~%int32 sva 	# satellite accuracy ~%int32 age 	# satellite age of operation ~%GTime toe 	# epoch of epherides (gpst) ~%GTime tof 	# message frame time (gpst) ~%float64[3] pos 	# satellite position (ecef) (m) ~%float64[3] vel 	# satellite velocity (ecef) (m/s) ~%float64[3] acc 	# satellite acceleration (ecef) (m/s^2) ~%float64 taun 	# SV clock bias (s) ~%float64 gamn 	# relative freq bias ~%float64 dtaun 	# delay between L1 and L2 (s) ~%================================================================================~%MSG: inertial_sense/GTime~%int64 time~%float64 sec~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GlonassEphemeris>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'toe))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'tof))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'vel) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'acc) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GlonassEphemeris>))
  "Converts a ROS message object to a list"
  (cl:list 'GlonassEphemeris
    (cl:cons ':sat (sat msg))
    (cl:cons ':iode (iode msg))
    (cl:cons ':frq (frq msg))
    (cl:cons ':svh (svh msg))
    (cl:cons ':sva (sva msg))
    (cl:cons ':age (age msg))
    (cl:cons ':toe (toe msg))
    (cl:cons ':tof (tof msg))
    (cl:cons ':pos (pos msg))
    (cl:cons ':vel (vel msg))
    (cl:cons ':acc (acc msg))
    (cl:cons ':taun (taun msg))
    (cl:cons ':gamn (gamn msg))
    (cl:cons ':dtaun (dtaun msg))
))
