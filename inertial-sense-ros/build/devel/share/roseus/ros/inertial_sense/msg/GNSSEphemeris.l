;; Auto-generated. Do not edit!


(when (boundp 'inertial_sense::GNSSEphemeris)
  (if (not (find-package "INERTIAL_SENSE"))
    (make-package "INERTIAL_SENSE"))
  (shadow 'GNSSEphemeris (find-package "INERTIAL_SENSE")))
(unless (find-package "INERTIAL_SENSE::GNSSEPHEMERIS")
  (make-package "INERTIAL_SENSE::GNSSEPHEMERIS"))

(in-package "ROS")
;;//! \htmlinclude GNSSEphemeris.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass inertial_sense::GNSSEphemeris
  :super ros::object
  :slots (_header _sat _iode _iodc _sva _svh _week _code _flag _toe _toc _ttr _A _e _i0 _OMG0 _omg _M0 _deln _OMGd _idot _crc _crs _cuc _cus _cic _cis _toes _fit _f0 _f1 _f2 _tgd _Adot _ndot ))

(defmethod inertial_sense::GNSSEphemeris
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:sat __sat) 0)
    ((:iode __iode) 0)
    ((:iodc __iodc) 0)
    ((:sva __sva) 0)
    ((:svh __svh) 0)
    ((:week __week) 0)
    ((:code __code) 0)
    ((:flag __flag) 0)
    ((:toe __toe) (instance inertial_sense::GTime :init))
    ((:toc __toc) (instance inertial_sense::GTime :init))
    ((:ttr __ttr) (instance inertial_sense::GTime :init))
    ((:A __A) 0.0)
    ((:e __e) 0.0)
    ((:i0 __i0) 0.0)
    ((:OMG0 __OMG0) 0.0)
    ((:omg __omg) 0.0)
    ((:M0 __M0) 0.0)
    ((:deln __deln) 0.0)
    ((:OMGd __OMGd) 0.0)
    ((:idot __idot) 0.0)
    ((:crc __crc) 0.0)
    ((:crs __crs) 0.0)
    ((:cuc __cuc) 0.0)
    ((:cus __cus) 0.0)
    ((:cic __cic) 0.0)
    ((:cis __cis) 0.0)
    ((:toes __toes) 0.0)
    ((:fit __fit) 0.0)
    ((:f0 __f0) 0.0)
    ((:f1 __f1) 0.0)
    ((:f2 __f2) 0.0)
    ((:tgd __tgd) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:Adot __Adot) 0.0)
    ((:ndot __ndot) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _sat (round __sat))
   (setq _iode (round __iode))
   (setq _iodc (round __iodc))
   (setq _sva (round __sva))
   (setq _svh (round __svh))
   (setq _week (round __week))
   (setq _code (round __code))
   (setq _flag (round __flag))
   (setq _toe __toe)
   (setq _toc __toc)
   (setq _ttr __ttr)
   (setq _A (float __A))
   (setq _e (float __e))
   (setq _i0 (float __i0))
   (setq _OMG0 (float __OMG0))
   (setq _omg (float __omg))
   (setq _M0 (float __M0))
   (setq _deln (float __deln))
   (setq _OMGd (float __OMGd))
   (setq _idot (float __idot))
   (setq _crc (float __crc))
   (setq _crs (float __crs))
   (setq _cuc (float __cuc))
   (setq _cus (float __cus))
   (setq _cic (float __cic))
   (setq _cis (float __cis))
   (setq _toes (float __toes))
   (setq _fit (float __fit))
   (setq _f0 (float __f0))
   (setq _f1 (float __f1))
   (setq _f2 (float __f2))
   (setq _tgd __tgd)
   (setq _Adot (float __Adot))
   (setq _ndot (float __ndot))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:sat
   (&optional __sat)
   (if __sat (setq _sat __sat)) _sat)
  (:iode
   (&optional __iode)
   (if __iode (setq _iode __iode)) _iode)
  (:iodc
   (&optional __iodc)
   (if __iodc (setq _iodc __iodc)) _iodc)
  (:sva
   (&optional __sva)
   (if __sva (setq _sva __sva)) _sva)
  (:svh
   (&optional __svh)
   (if __svh (setq _svh __svh)) _svh)
  (:week
   (&optional __week)
   (if __week (setq _week __week)) _week)
  (:code
   (&optional __code)
   (if __code (setq _code __code)) _code)
  (:flag
   (&optional __flag)
   (if __flag (setq _flag __flag)) _flag)
  (:toe
   (&rest __toe)
   (if (keywordp (car __toe))
       (send* _toe __toe)
     (progn
       (if __toe (setq _toe (car __toe)))
       _toe)))
  (:toc
   (&rest __toc)
   (if (keywordp (car __toc))
       (send* _toc __toc)
     (progn
       (if __toc (setq _toc (car __toc)))
       _toc)))
  (:ttr
   (&rest __ttr)
   (if (keywordp (car __ttr))
       (send* _ttr __ttr)
     (progn
       (if __ttr (setq _ttr (car __ttr)))
       _ttr)))
  (:A
   (&optional __A)
   (if __A (setq _A __A)) _A)
  (:e
   (&optional __e)
   (if __e (setq _e __e)) _e)
  (:i0
   (&optional __i0)
   (if __i0 (setq _i0 __i0)) _i0)
  (:OMG0
   (&optional __OMG0)
   (if __OMG0 (setq _OMG0 __OMG0)) _OMG0)
  (:omg
   (&optional __omg)
   (if __omg (setq _omg __omg)) _omg)
  (:M0
   (&optional __M0)
   (if __M0 (setq _M0 __M0)) _M0)
  (:deln
   (&optional __deln)
   (if __deln (setq _deln __deln)) _deln)
  (:OMGd
   (&optional __OMGd)
   (if __OMGd (setq _OMGd __OMGd)) _OMGd)
  (:idot
   (&optional __idot)
   (if __idot (setq _idot __idot)) _idot)
  (:crc
   (&optional __crc)
   (if __crc (setq _crc __crc)) _crc)
  (:crs
   (&optional __crs)
   (if __crs (setq _crs __crs)) _crs)
  (:cuc
   (&optional __cuc)
   (if __cuc (setq _cuc __cuc)) _cuc)
  (:cus
   (&optional __cus)
   (if __cus (setq _cus __cus)) _cus)
  (:cic
   (&optional __cic)
   (if __cic (setq _cic __cic)) _cic)
  (:cis
   (&optional __cis)
   (if __cis (setq _cis __cis)) _cis)
  (:toes
   (&optional __toes)
   (if __toes (setq _toes __toes)) _toes)
  (:fit
   (&optional __fit)
   (if __fit (setq _fit __fit)) _fit)
  (:f0
   (&optional __f0)
   (if __f0 (setq _f0 __f0)) _f0)
  (:f1
   (&optional __f1)
   (if __f1 (setq _f1 __f1)) _f1)
  (:f2
   (&optional __f2)
   (if __f2 (setq _f2 __f2)) _f2)
  (:tgd
   (&optional __tgd)
   (if __tgd (setq _tgd __tgd)) _tgd)
  (:Adot
   (&optional __Adot)
   (if __Adot (setq _Adot __Adot)) _Adot)
  (:ndot
   (&optional __ndot)
   (if __ndot (setq _ndot __ndot)) _ndot)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _sat
    4
    ;; int32 _iode
    4
    ;; int32 _iodc
    4
    ;; int32 _sva
    4
    ;; int32 _svh
    4
    ;; int32 _week
    4
    ;; int32 _code
    4
    ;; int32 _flag
    4
    ;; inertial_sense/GTime _toe
    (send _toe :serialization-length)
    ;; inertial_sense/GTime _toc
    (send _toc :serialization-length)
    ;; inertial_sense/GTime _ttr
    (send _ttr :serialization-length)
    ;; float64 _A
    8
    ;; float64 _e
    8
    ;; float64 _i0
    8
    ;; float64 _OMG0
    8
    ;; float64 _omg
    8
    ;; float64 _M0
    8
    ;; float64 _deln
    8
    ;; float64 _OMGd
    8
    ;; float64 _idot
    8
    ;; float64 _crc
    8
    ;; float64 _crs
    8
    ;; float64 _cuc
    8
    ;; float64 _cus
    8
    ;; float64 _cic
    8
    ;; float64 _cis
    8
    ;; float64 _toes
    8
    ;; float64 _fit
    8
    ;; float64 _f0
    8
    ;; float64 _f1
    8
    ;; float64 _f2
    8
    ;; float64[4] _tgd
    (* 8    4)
    ;; float64 _Adot
    8
    ;; float64 _ndot
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _sat
       (write-long _sat s)
     ;; int32 _iode
       (write-long _iode s)
     ;; int32 _iodc
       (write-long _iodc s)
     ;; int32 _sva
       (write-long _sva s)
     ;; int32 _svh
       (write-long _svh s)
     ;; int32 _week
       (write-long _week s)
     ;; int32 _code
       (write-long _code s)
     ;; int32 _flag
       (write-long _flag s)
     ;; inertial_sense/GTime _toe
       (send _toe :serialize s)
     ;; inertial_sense/GTime _toc
       (send _toc :serialize s)
     ;; inertial_sense/GTime _ttr
       (send _ttr :serialize s)
     ;; float64 _A
       (sys::poke _A (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _e
       (sys::poke _e (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _i0
       (sys::poke _i0 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _OMG0
       (sys::poke _OMG0 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _omg
       (sys::poke _omg (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _M0
       (sys::poke _M0 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _deln
       (sys::poke _deln (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _OMGd
       (sys::poke _OMGd (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _idot
       (sys::poke _idot (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _crc
       (sys::poke _crc (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _crs
       (sys::poke _crs (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _cuc
       (sys::poke _cuc (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _cus
       (sys::poke _cus (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _cic
       (sys::poke _cic (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _cis
       (sys::poke _cis (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _toes
       (sys::poke _toes (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _fit
       (sys::poke _fit (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _f0
       (sys::poke _f0 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _f1
       (sys::poke _f1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _f2
       (sys::poke _f2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[4] _tgd
     (dotimes (i 4)
       (sys::poke (elt _tgd i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _Adot
       (sys::poke _Adot (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ndot
       (sys::poke _ndot (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _sat
     (setq _sat (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _iode
     (setq _iode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _iodc
     (setq _iodc (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _sva
     (setq _sva (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _svh
     (setq _svh (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _week
     (setq _week (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _code
     (setq _code (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _flag
     (setq _flag (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; inertial_sense/GTime _toe
     (send _toe :deserialize buf ptr-) (incf ptr- (send _toe :serialization-length))
   ;; inertial_sense/GTime _toc
     (send _toc :deserialize buf ptr-) (incf ptr- (send _toc :serialization-length))
   ;; inertial_sense/GTime _ttr
     (send _ttr :deserialize buf ptr-) (incf ptr- (send _ttr :serialization-length))
   ;; float64 _A
     (setq _A (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _e
     (setq _e (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _i0
     (setq _i0 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _OMG0
     (setq _OMG0 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _omg
     (setq _omg (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _M0
     (setq _M0 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _deln
     (setq _deln (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _OMGd
     (setq _OMGd (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _idot
     (setq _idot (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _crc
     (setq _crc (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _crs
     (setq _crs (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _cuc
     (setq _cuc (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _cus
     (setq _cus (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _cic
     (setq _cic (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _cis
     (setq _cis (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _toes
     (setq _toes (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _fit
     (setq _fit (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _f0
     (setq _f0 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _f1
     (setq _f1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _f2
     (setq _f2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[4] _tgd
   (dotimes (i (length _tgd))
     (setf (elt _tgd i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64 _Adot
     (setq _Adot (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ndot
     (setq _ndot (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get inertial_sense::GNSSEphemeris :md5sum-) "b950a03f405d085580c4de95aeee72ef")
(setf (get inertial_sense::GNSSEphemeris :datatype-) "inertial_sense/GNSSEphemeris")
(setf (get inertial_sense::GNSSEphemeris :definition-)
      "Header header
int32 sat 		# satellite number
int32 iode 		# IODE Issue of Data, Ephemeris (ephemeris version)
int32 iodc 		# IODC Issue of Data, Clock (clock version)
int32 sva 		# SV accuracy (URA index) IRN-IS-200H p.97            
int32 svh 		# SV health GPS/QZS (0:ok)            
int32 week 		# GPS/QZS: gps week, GAL: galileo week
int32 code 		# GPS/QZS: code on L2 * (00=Invalid, 01 = P Code ON, 11 = C/A code ON, 11 = Invalid) * GAL/CMP: data sources
int32 flag 		# GPS/QZS: L2 P data flag (indicates that the NAV data stream was commanded OFF on the P-code of the in-phase component of the L2 channel) *  CMP: nav type
GTime toe 		# Toe
GTime toc 		# clock data reference time (s) (20.3.4.5)
GTime ttr 		# T_trans
float64 A 		# Semi-Major Axis m
float64 e 		# Eccentricity (no units) 
float64 i0 		# Inclination Angle at Reference Time (rad)
float64 OMG0 	# Longitude of Ascending Node of Orbit Plane at Weekly Epoch (rad)
float64 omg 	# Argument of Perigee (rad)
float64 M0 		# Mean Anomaly at Reference Time (rad)
float64 deln 	# Mean Motion Difference From Computed Value (rad)
float64 OMGd 	# Rate of Right Ascension (rad/s)
float64 idot 	# Rate of Inclination Angle (rad/s)
float64 crc 	# Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius
float64 crs 	# Amplitude of the Sine Harmonic Correction Term to the Orbit Radius (m)
float64 cuc 	# Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude (rad) 
float64 cus 	# Amplitude of the Sine Harmonic Correction Term to the Argument of Latitude (rad)
float64 cic 	# Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination (rad)
float64 cis 	# Amplitude of the Sine Harmonic Correction Term to the Angle of Inclination (rad)
float64 toes 	# Reference Time Ephemeris in week (s)
float64 fit 	# fit interval (h) (0: 4 hours, 1:greater than 4 hours)
float64 f0 		# SV clock parameters - af0
float64 f1 		# SV clock parameters - af1
float64 f2 		# SV clock parameters - af2
float64[4] tgd 	# group delay parameters: GPS/QZS:tgd[0]=TGD (IRN-IS-200H p.103) * GAL:tgd[0]=BGD E5a/E1,tgd[1]=BGD E5b/E1	* CMP :tgd[0]=BGD1,tgd[1]=BGD2
float64 Adot 	# Adot for CNAV
float64 ndot 	# ndot for CNAV
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
MSG: inertial_sense/GTime
int64 time
float64 sec
")



(provide :inertial_sense/GNSSEphemeris "b950a03f405d085580c4de95aeee72ef")


