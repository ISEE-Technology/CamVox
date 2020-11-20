;; Auto-generated. Do not edit!


(when (boundp 'inertial_sense::GNSSObsVec)
  (if (not (find-package "INERTIAL_SENSE"))
    (make-package "INERTIAL_SENSE"))
  (shadow 'GNSSObsVec (find-package "INERTIAL_SENSE")))
(unless (find-package "INERTIAL_SENSE::GNSSOBSVEC")
  (make-package "INERTIAL_SENSE::GNSSOBSVEC"))

(in-package "ROS")
;;//! \htmlinclude GNSSObsVec.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass inertial_sense::GNSSObsVec
  :super ros::object
  :slots (_header _time _obs ))

(defmethod inertial_sense::GNSSObsVec
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:time __time) (instance inertial_sense::GTime :init))
    ((:obs __obs) (let (r) (dotimes (i 0) (push (instance inertial_sense::GNSSObservation :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _time __time)
   (setq _obs __obs)
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
  (:obs
   (&rest __obs)
   (if (keywordp (car __obs))
       (send* _obs __obs)
     (progn
       (if __obs (setq _obs (car __obs)))
       _obs)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; inertial_sense/GTime _time
    (send _time :serialization-length)
    ;; inertial_sense/GNSSObservation[] _obs
    (apply #'+ (send-all _obs :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; inertial_sense/GTime _time
       (send _time :serialize s)
     ;; inertial_sense/GNSSObservation[] _obs
     (write-long (length _obs) s)
     (dolist (elem _obs)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; inertial_sense/GTime _time
     (send _time :deserialize buf ptr-) (incf ptr- (send _time :serialization-length))
   ;; inertial_sense/GNSSObservation[] _obs
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _obs (let (r) (dotimes (i n) (push (instance inertial_sense::GNSSObservation :init) r)) r))
     (dolist (elem- _obs)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get inertial_sense::GNSSObsVec :md5sum-) "d228e847dabfc151b595c858b8d03b94")
(setf (get inertial_sense::GNSSObsVec :datatype-) "inertial_sense/GNSSObsVec")
(setf (get inertial_sense::GNSSObsVec :definition-)
      "std_msgs/Header header
GTime time              # time of all contained observations (UTC Time w/o Leap Seconds)
GNSSObservation[] obs   # Vector of observations
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
================================================================================
MSG: inertial_sense/GNSSObservation
std_msgs/Header header
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
")



(provide :inertial_sense/GNSSObsVec "d228e847dabfc151b595c858b8d03b94")


