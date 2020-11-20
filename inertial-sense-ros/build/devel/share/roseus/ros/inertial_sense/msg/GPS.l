;; Auto-generated. Do not edit!


(when (boundp 'inertial_sense::GPS)
  (if (not (find-package "INERTIAL_SENSE"))
    (make-package "INERTIAL_SENSE"))
  (shadow 'GPS (find-package "INERTIAL_SENSE")))
(unless (find-package "INERTIAL_SENSE::GPS")
  (make-package "INERTIAL_SENSE::GPS"))

(in-package "ROS")
;;//! \htmlinclude GPS.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*GPS_STATUS_FIX_TYPE_NO_FIX*" (find-package "INERTIAL_SENSE::GPS"))
(shadow '*GPS_STATUS_FIX_TYPE_NO_FIX* (find-package "INERTIAL_SENSE::GPS"))
(defconstant inertial_sense::GPS::*GPS_STATUS_FIX_TYPE_NO_FIX* 0)
(intern "*GPS_STATUS_FIX_TYPE_DEAD_RECKONING_ONLY*" (find-package "INERTIAL_SENSE::GPS"))
(shadow '*GPS_STATUS_FIX_TYPE_DEAD_RECKONING_ONLY* (find-package "INERTIAL_SENSE::GPS"))
(defconstant inertial_sense::GPS::*GPS_STATUS_FIX_TYPE_DEAD_RECKONING_ONLY* 256)
(intern "*GPS_STATUS_FIX_TYPE_2D_FIX*" (find-package "INERTIAL_SENSE::GPS"))
(shadow '*GPS_STATUS_FIX_TYPE_2D_FIX* (find-package "INERTIAL_SENSE::GPS"))
(defconstant inertial_sense::GPS::*GPS_STATUS_FIX_TYPE_2D_FIX* 512)
(intern "*GPS_STATUS_FIX_TYPE_3D_FIX*" (find-package "INERTIAL_SENSE::GPS"))
(shadow '*GPS_STATUS_FIX_TYPE_3D_FIX* (find-package "INERTIAL_SENSE::GPS"))
(defconstant inertial_sense::GPS::*GPS_STATUS_FIX_TYPE_3D_FIX* 768)
(intern "*GPS_STATUS_FIX_TYPE_GPS_PLUS_DEAD_RECK*" (find-package "INERTIAL_SENSE::GPS"))
(shadow '*GPS_STATUS_FIX_TYPE_GPS_PLUS_DEAD_RECK* (find-package "INERTIAL_SENSE::GPS"))
(defconstant inertial_sense::GPS::*GPS_STATUS_FIX_TYPE_GPS_PLUS_DEAD_RECK* 1024)
(intern "*GPS_STATUS_FIX_TYPE_TIME_ONLY_FIX*" (find-package "INERTIAL_SENSE::GPS"))
(shadow '*GPS_STATUS_FIX_TYPE_TIME_ONLY_FIX* (find-package "INERTIAL_SENSE::GPS"))
(defconstant inertial_sense::GPS::*GPS_STATUS_FIX_TYPE_TIME_ONLY_FIX* 1280)
(intern "*GPS_STATUS_FIX_TYPE_RESERVED1*" (find-package "INERTIAL_SENSE::GPS"))
(shadow '*GPS_STATUS_FIX_TYPE_RESERVED1* (find-package "INERTIAL_SENSE::GPS"))
(defconstant inertial_sense::GPS::*GPS_STATUS_FIX_TYPE_RESERVED1* 1536)
(intern "*GPS_STATUS_FIX_TYPE_RESERVED2*" (find-package "INERTIAL_SENSE::GPS"))
(shadow '*GPS_STATUS_FIX_TYPE_RESERVED2* (find-package "INERTIAL_SENSE::GPS"))
(defconstant inertial_sense::GPS::*GPS_STATUS_FIX_TYPE_RESERVED2* 1792)
(intern "*GPS_STATUS_FIX_STATUS_FIX_OK*" (find-package "INERTIAL_SENSE::GPS"))
(shadow '*GPS_STATUS_FIX_STATUS_FIX_OK* (find-package "INERTIAL_SENSE::GPS"))
(defconstant inertial_sense::GPS::*GPS_STATUS_FIX_STATUS_FIX_OK* 65536)
(defclass inertial_sense::GPS
  :super ros::object
  :slots (_header _num_sat _fix_type _cno _latitude _longitude _altitude _posEcef _velEcef _hMSL _hAcc _vAcc _sAcc _pDop ))

(defmethod inertial_sense::GPS
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:num_sat __num_sat) 0)
    ((:fix_type __fix_type) 0)
    ((:cno __cno) 0)
    ((:latitude __latitude) 0.0)
    ((:longitude __longitude) 0.0)
    ((:altitude __altitude) 0.0)
    ((:posEcef __posEcef) (instance geometry_msgs::Vector3 :init))
    ((:velEcef __velEcef) (instance geometry_msgs::Vector3 :init))
    ((:hMSL __hMSL) 0.0)
    ((:hAcc __hAcc) 0.0)
    ((:vAcc __vAcc) 0.0)
    ((:sAcc __sAcc) 0.0)
    ((:pDop __pDop) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _num_sat (round __num_sat))
   (setq _fix_type (round __fix_type))
   (setq _cno (round __cno))
   (setq _latitude (float __latitude))
   (setq _longitude (float __longitude))
   (setq _altitude (float __altitude))
   (setq _posEcef __posEcef)
   (setq _velEcef __velEcef)
   (setq _hMSL (float __hMSL))
   (setq _hAcc (float __hAcc))
   (setq _vAcc (float __vAcc))
   (setq _sAcc (float __sAcc))
   (setq _pDop (float __pDop))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:num_sat
   (&optional __num_sat)
   (if __num_sat (setq _num_sat __num_sat)) _num_sat)
  (:fix_type
   (&optional __fix_type)
   (if __fix_type (setq _fix_type __fix_type)) _fix_type)
  (:cno
   (&optional __cno)
   (if __cno (setq _cno __cno)) _cno)
  (:latitude
   (&optional __latitude)
   (if __latitude (setq _latitude __latitude)) _latitude)
  (:longitude
   (&optional __longitude)
   (if __longitude (setq _longitude __longitude)) _longitude)
  (:altitude
   (&optional __altitude)
   (if __altitude (setq _altitude __altitude)) _altitude)
  (:posEcef
   (&rest __posEcef)
   (if (keywordp (car __posEcef))
       (send* _posEcef __posEcef)
     (progn
       (if __posEcef (setq _posEcef (car __posEcef)))
       _posEcef)))
  (:velEcef
   (&rest __velEcef)
   (if (keywordp (car __velEcef))
       (send* _velEcef __velEcef)
     (progn
       (if __velEcef (setq _velEcef (car __velEcef)))
       _velEcef)))
  (:hMSL
   (&optional __hMSL)
   (if __hMSL (setq _hMSL __hMSL)) _hMSL)
  (:hAcc
   (&optional __hAcc)
   (if __hAcc (setq _hAcc __hAcc)) _hAcc)
  (:vAcc
   (&optional __vAcc)
   (if __vAcc (setq _vAcc __vAcc)) _vAcc)
  (:sAcc
   (&optional __sAcc)
   (if __sAcc (setq _sAcc __sAcc)) _sAcc)
  (:pDop
   (&optional __pDop)
   (if __pDop (setq _pDop __pDop)) _pDop)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int8 _num_sat
    1
    ;; uint32 _fix_type
    4
    ;; int32 _cno
    4
    ;; float64 _latitude
    8
    ;; float64 _longitude
    8
    ;; float64 _altitude
    8
    ;; geometry_msgs/Vector3 _posEcef
    (send _posEcef :serialization-length)
    ;; geometry_msgs/Vector3 _velEcef
    (send _velEcef :serialization-length)
    ;; float32 _hMSL
    4
    ;; float32 _hAcc
    4
    ;; float32 _vAcc
    4
    ;; float32 _sAcc
    4
    ;; float32 _pDop
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int8 _num_sat
       (write-byte _num_sat s)
     ;; uint32 _fix_type
       (write-long _fix_type s)
     ;; int32 _cno
       (write-long _cno s)
     ;; float64 _latitude
       (sys::poke _latitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _longitude
       (sys::poke _longitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _altitude
       (sys::poke _altitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/Vector3 _posEcef
       (send _posEcef :serialize s)
     ;; geometry_msgs/Vector3 _velEcef
       (send _velEcef :serialize s)
     ;; float32 _hMSL
       (sys::poke _hMSL (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _hAcc
       (sys::poke _hAcc (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vAcc
       (sys::poke _vAcc (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _sAcc
       (sys::poke _sAcc (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pDop
       (sys::poke _pDop (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int8 _num_sat
     (setq _num_sat (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _num_sat 127) (setq _num_sat (- _num_sat 256)))
   ;; uint32 _fix_type
     (setq _fix_type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _cno
     (setq _cno (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _latitude
     (setq _latitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _longitude
     (setq _longitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _altitude
     (setq _altitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/Vector3 _posEcef
     (send _posEcef :deserialize buf ptr-) (incf ptr- (send _posEcef :serialization-length))
   ;; geometry_msgs/Vector3 _velEcef
     (send _velEcef :deserialize buf ptr-) (incf ptr- (send _velEcef :serialization-length))
   ;; float32 _hMSL
     (setq _hMSL (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _hAcc
     (setq _hAcc (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vAcc
     (setq _vAcc (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _sAcc
     (setq _sAcc (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pDop
     (setq _pDop (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get inertial_sense::GPS :md5sum-) "6aa847d654b817ff4bb5ba8c773b2a17")
(setf (get inertial_sense::GPS :datatype-) "inertial_sense/GPS")
(setf (get inertial_sense::GPS :definition-)
      "# GPS status flags
uint32 GPS_STATUS_FIX_TYPE_NO_FIX               = 0
uint32 GPS_STATUS_FIX_TYPE_DEAD_RECKONING_ONLY  = 256
uint32 GPS_STATUS_FIX_TYPE_2D_FIX               = 512
uint32 GPS_STATUS_FIX_TYPE_3D_FIX               = 768
uint32 GPS_STATUS_FIX_TYPE_GPS_PLUS_DEAD_RECK   = 1024
uint32 GPS_STATUS_FIX_TYPE_TIME_ONLY_FIX        = 1280
uint32 GPS_STATUS_FIX_TYPE_RESERVED1            = 1536
uint32 GPS_STATUS_FIX_TYPE_RESERVED2            = 1792

uint32 GPS_STATUS_FIX_STATUS_FIX_OK             = 65536

Header header
int8 num_sat 							# Number of satellites used in solution
uint32 fix_type 						# Fix type, one of STATUS_FIX_TYPE flags
int32 cno 								# mean carrier noise ratio (dBHz)
float64 latitude 						# latitude (degrees) 
float64 longitude						# longitude (degrees)
float64 altitude						# height above ellipsoid (not MSL) (m)
geometry_msgs/Vector3 posEcef           # Position (m) in ECEF
geometry_msgs/Vector3 velEcef       	# Velocity (m/s) in ECEF
float32 hMSL							# height above MSL
float32 hAcc							# horizontal accuracy
float32 vAcc							# vertical accuracy
float32 sAcc							# speed accuracy (m/s)
float32 pDop							# Position Dilution of Precision (m)	


================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :inertial_sense/GPS "6aa847d654b817ff4bb5ba8c773b2a17")


