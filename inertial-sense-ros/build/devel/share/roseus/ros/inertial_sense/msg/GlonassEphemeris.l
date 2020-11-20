;; Auto-generated. Do not edit!


(when (boundp 'inertial_sense::GlonassEphemeris)
  (if (not (find-package "INERTIAL_SENSE"))
    (make-package "INERTIAL_SENSE"))
  (shadow 'GlonassEphemeris (find-package "INERTIAL_SENSE")))
(unless (find-package "INERTIAL_SENSE::GLONASSEPHEMERIS")
  (make-package "INERTIAL_SENSE::GLONASSEPHEMERIS"))

(in-package "ROS")
;;//! \htmlinclude GlonassEphemeris.msg.html


(defclass inertial_sense::GlonassEphemeris
  :super ros::object
  :slots (_sat _iode _frq _svh _sva _age _toe _tof _pos _vel _acc _taun _gamn _dtaun ))

(defmethod inertial_sense::GlonassEphemeris
  (:init
   (&key
    ((:sat __sat) 0)
    ((:iode __iode) 0)
    ((:frq __frq) 0)
    ((:svh __svh) 0)
    ((:sva __sva) 0)
    ((:age __age) 0)
    ((:toe __toe) (instance inertial_sense::GTime :init))
    ((:tof __tof) (instance inertial_sense::GTime :init))
    ((:pos __pos) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:vel __vel) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:acc __acc) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:taun __taun) 0.0)
    ((:gamn __gamn) 0.0)
    ((:dtaun __dtaun) 0.0)
    )
   (send-super :init)
   (setq _sat (round __sat))
   (setq _iode (round __iode))
   (setq _frq (round __frq))
   (setq _svh (round __svh))
   (setq _sva (round __sva))
   (setq _age (round __age))
   (setq _toe __toe)
   (setq _tof __tof)
   (setq _pos __pos)
   (setq _vel __vel)
   (setq _acc __acc)
   (setq _taun (float __taun))
   (setq _gamn (float __gamn))
   (setq _dtaun (float __dtaun))
   self)
  (:sat
   (&optional __sat)
   (if __sat (setq _sat __sat)) _sat)
  (:iode
   (&optional __iode)
   (if __iode (setq _iode __iode)) _iode)
  (:frq
   (&optional __frq)
   (if __frq (setq _frq __frq)) _frq)
  (:svh
   (&optional __svh)
   (if __svh (setq _svh __svh)) _svh)
  (:sva
   (&optional __sva)
   (if __sva (setq _sva __sva)) _sva)
  (:age
   (&optional __age)
   (if __age (setq _age __age)) _age)
  (:toe
   (&rest __toe)
   (if (keywordp (car __toe))
       (send* _toe __toe)
     (progn
       (if __toe (setq _toe (car __toe)))
       _toe)))
  (:tof
   (&rest __tof)
   (if (keywordp (car __tof))
       (send* _tof __tof)
     (progn
       (if __tof (setq _tof (car __tof)))
       _tof)))
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:vel
   (&optional __vel)
   (if __vel (setq _vel __vel)) _vel)
  (:acc
   (&optional __acc)
   (if __acc (setq _acc __acc)) _acc)
  (:taun
   (&optional __taun)
   (if __taun (setq _taun __taun)) _taun)
  (:gamn
   (&optional __gamn)
   (if __gamn (setq _gamn __gamn)) _gamn)
  (:dtaun
   (&optional __dtaun)
   (if __dtaun (setq _dtaun __dtaun)) _dtaun)
  (:serialization-length
   ()
   (+
    ;; int32 _sat
    4
    ;; int32 _iode
    4
    ;; int32 _frq
    4
    ;; int32 _svh
    4
    ;; int32 _sva
    4
    ;; int32 _age
    4
    ;; inertial_sense/GTime _toe
    (send _toe :serialization-length)
    ;; inertial_sense/GTime _tof
    (send _tof :serialization-length)
    ;; float64[3] _pos
    (* 8    3)
    ;; float64[3] _vel
    (* 8    3)
    ;; float64[3] _acc
    (* 8    3)
    ;; float64 _taun
    8
    ;; float64 _gamn
    8
    ;; float64 _dtaun
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _sat
       (write-long _sat s)
     ;; int32 _iode
       (write-long _iode s)
     ;; int32 _frq
       (write-long _frq s)
     ;; int32 _svh
       (write-long _svh s)
     ;; int32 _sva
       (write-long _sva s)
     ;; int32 _age
       (write-long _age s)
     ;; inertial_sense/GTime _toe
       (send _toe :serialize s)
     ;; inertial_sense/GTime _tof
       (send _tof :serialize s)
     ;; float64[3] _pos
     (dotimes (i 3)
       (sys::poke (elt _pos i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _vel
     (dotimes (i 3)
       (sys::poke (elt _vel i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _acc
     (dotimes (i 3)
       (sys::poke (elt _acc i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _taun
       (sys::poke _taun (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _gamn
       (sys::poke _gamn (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _dtaun
       (sys::poke _dtaun (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _sat
     (setq _sat (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _iode
     (setq _iode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _frq
     (setq _frq (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _svh
     (setq _svh (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _sva
     (setq _sva (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _age
     (setq _age (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; inertial_sense/GTime _toe
     (send _toe :deserialize buf ptr-) (incf ptr- (send _toe :serialization-length))
   ;; inertial_sense/GTime _tof
     (send _tof :deserialize buf ptr-) (incf ptr- (send _tof :serialization-length))
   ;; float64[3] _pos
   (dotimes (i (length _pos))
     (setf (elt _pos i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _vel
   (dotimes (i (length _vel))
     (setf (elt _vel i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _acc
   (dotimes (i (length _acc))
     (setf (elt _acc i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64 _taun
     (setq _taun (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _gamn
     (setq _gamn (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _dtaun
     (setq _dtaun (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get inertial_sense::GlonassEphemeris :md5sum-) "d6e50080404485ddd899caa0ddef8be5")
(setf (get inertial_sense::GlonassEphemeris :datatype-) "inertial_sense/GlonassEphemeris")
(setf (get inertial_sense::GlonassEphemeris :definition-)
      "int32 sat 	# satellite number 
int32 iode 	# IODE (0-6 bit of tb field) 
int32 frq 	# satellite frequency number 
int32 svh 	# satellite health 
int32 sva 	# satellite accuracy 
int32 age 	# satellite age of operation 
GTime toe 	# epoch of epherides (gpst) 
GTime tof 	# message frame time (gpst) 
float64[3] pos 	# satellite position (ecef) (m) 
float64[3] vel 	# satellite velocity (ecef) (m/s) 
float64[3] acc 	# satellite acceleration (ecef) (m/s^2) 
float64 taun 	# SV clock bias (s) 
float64 gamn 	# relative freq bias 
float64 dtaun 	# delay between L1 and L2 (s) 
================================================================================
MSG: inertial_sense/GTime
int64 time
float64 sec
")



(provide :inertial_sense/GlonassEphemeris "d6e50080404485ddd899caa0ddef8be5")


