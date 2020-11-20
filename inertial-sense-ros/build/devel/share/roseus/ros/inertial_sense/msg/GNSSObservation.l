;; Auto-generated. Do not edit!


(when (boundp 'inertial_sense::GNSSObservation)
  (if (not (find-package "INERTIAL_SENSE"))
    (make-package "INERTIAL_SENSE"))
  (shadow 'GNSSObservation (find-package "INERTIAL_SENSE")))
(unless (find-package "INERTIAL_SENSE::GNSSOBSERVATION")
  (make-package "INERTIAL_SENSE::GNSSOBSERVATION"))

(in-package "ROS")
;;//! \htmlinclude GNSSObservation.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass inertial_sense::GNSSObservation
  :super ros::object
  :slots (_header _time _sat _rcv _SNR _LLI _code _qualL _qualP _L _P _D ))

(defmethod inertial_sense::GNSSObservation
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:time __time) (instance inertial_sense::GTime :init))
    ((:sat __sat) 0)
    ((:rcv __rcv) 0)
    ((:SNR __SNR) 0)
    ((:LLI __LLI) 0)
    ((:code __code) 0)
    ((:qualL __qualL) 0)
    ((:qualP __qualP) 0)
    ((:L __L) 0.0)
    ((:P __P) 0.0)
    ((:D __D) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _time __time)
   (setq _sat (round __sat))
   (setq _rcv (round __rcv))
   (setq _SNR (round __SNR))
   (setq _LLI (round __LLI))
   (setq _code (round __code))
   (setq _qualL (round __qualL))
   (setq _qualP (round __qualP))
   (setq _L (float __L))
   (setq _P (float __P))
   (setq _D (float __D))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:time
   (&rest __time)
   (if (keywordp (car __time))
       (send* _time __time)
     (progn
       (if __time (setq _time (car __time)))
       _time)))
  (:sat
   (&optional __sat)
   (if __sat (setq _sat __sat)) _sat)
  (:rcv
   (&optional __rcv)
   (if __rcv (setq _rcv __rcv)) _rcv)
  (:SNR
   (&optional __SNR)
   (if __SNR (setq _SNR __SNR)) _SNR)
  (:LLI
   (&optional __LLI)
   (if __LLI (setq _LLI __LLI)) _LLI)
  (:code
   (&optional __code)
   (if __code (setq _code __code)) _code)
  (:qualL
   (&optional __qualL)
   (if __qualL (setq _qualL __qualL)) _qualL)
  (:qualP
   (&optional __qualP)
   (if __qualP (setq _qualP __qualP)) _qualP)
  (:L
   (&optional __L)
   (if __L (setq _L __L)) _L)
  (:P
   (&optional __P)
   (if __P (setq _P __P)) _P)
  (:D
   (&optional __D)
   (if __D (setq _D __D)) _D)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; inertial_sense/GTime _time
    (send _time :serialization-length)
    ;; uint8 _sat
    1
    ;; uint8 _rcv
    1
    ;; uint8 _SNR
    1
    ;; uint8 _LLI
    1
    ;; uint8 _code
    1
    ;; uint8 _qualL
    1
    ;; uint8 _qualP
    1
    ;; float64 _L
    8
    ;; float64 _P
    8
    ;; float32 _D
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; inertial_sense/GTime _time
       (send _time :serialize s)
     ;; uint8 _sat
       (write-byte _sat s)
     ;; uint8 _rcv
       (write-byte _rcv s)
     ;; uint8 _SNR
       (write-byte _SNR s)
     ;; uint8 _LLI
       (write-byte _LLI s)
     ;; uint8 _code
       (write-byte _code s)
     ;; uint8 _qualL
       (write-byte _qualL s)
     ;; uint8 _qualP
       (write-byte _qualP s)
     ;; float64 _L
       (sys::poke _L (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _P
       (sys::poke _P (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _D
       (sys::poke _D (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; inertial_sense/GTime _time
     (send _time :deserialize buf ptr-) (incf ptr- (send _time :serialization-length))
   ;; uint8 _sat
     (setq _sat (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _rcv
     (setq _rcv (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _SNR
     (setq _SNR (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _LLI
     (setq _LLI (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _code
     (setq _code (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _qualL
     (setq _qualL (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _qualP
     (setq _qualP (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float64 _L
     (setq _L (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _P
     (setq _P (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _D
     (setq _D (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get inertial_sense::GNSSObservation :md5sum-) "f652831660ce8b4781ba3cf83655ca76")
(setf (get inertial_sense::GNSSObservation :datatype-) "inertial_sense/GNSSObservation")
(setf (get inertial_sense::GNSSObservation :definition-)
      "std_msgs/Header header
GTime time              # time of observation
uint8 sat 		# satellite number
uint8 rcv 		# receiver number
uint8 SNR 		# Signal Strength (0.25 dBHz)
uint8 LLI 		# Loss-of-Lock Indicator (bit1=loss-of-lock, bit2=half-cycle-invalid)
uint8 code 		# code indicator (BeiDou: CODE_L1I, Other: CODE_L1C )
uint8 qualL 	# Estimated carrier phase measurement standard deviation (0.004 cycles)
uint8 qualP 	# Estimated pseudorange measurement standard deviation (0.01 m)
float64 L      	# observation data carrier-phase (cycle)
float64 P      	# observation data pseudorange (m)
float32 D      	# observation data doppler frequency (0.002 Hz)
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



(provide :inertial_sense/GNSSObservation "f652831660ce8b4781ba3cf83655ca76")


