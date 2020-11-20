;; Auto-generated. Do not edit!


(when (boundp 'inertial_sense::RTKInfo)
  (if (not (find-package "INERTIAL_SENSE"))
    (make-package "INERTIAL_SENSE"))
  (shadow 'RTKInfo (find-package "INERTIAL_SENSE")))
(unless (find-package "INERTIAL_SENSE::RTKINFO")
  (make-package "INERTIAL_SENSE::RTKINFO"))

(in-package "ROS")
;;//! \htmlinclude RTKInfo.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass inertial_sense::RTKInfo
  :super ros::object
  :slots (_header _BaseLLA _cycle_slip_count _roverObs _baseObs _roverEph _baseEph _baseAntcount ))

(defmethod inertial_sense::RTKInfo
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:BaseLLA __BaseLLA) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:cycle_slip_count __cycle_slip_count) 0)
    ((:roverObs __roverObs) 0)
    ((:baseObs __baseObs) 0)
    ((:roverEph __roverEph) 0)
    ((:baseEph __baseEph) 0)
    ((:baseAntcount __baseAntcount) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _BaseLLA __BaseLLA)
   (setq _cycle_slip_count (round __cycle_slip_count))
   (setq _roverObs (round __roverObs))
   (setq _baseObs (round __baseObs))
   (setq _roverEph (round __roverEph))
   (setq _baseEph (round __baseEph))
   (setq _baseAntcount (round __baseAntcount))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:BaseLLA
   (&optional __BaseLLA)
   (if __BaseLLA (setq _BaseLLA __BaseLLA)) _BaseLLA)
  (:cycle_slip_count
   (&optional __cycle_slip_count)
   (if __cycle_slip_count (setq _cycle_slip_count __cycle_slip_count)) _cycle_slip_count)
  (:roverObs
   (&optional __roverObs)
   (if __roverObs (setq _roverObs __roverObs)) _roverObs)
  (:baseObs
   (&optional __baseObs)
   (if __baseObs (setq _baseObs __baseObs)) _baseObs)
  (:roverEph
   (&optional __roverEph)
   (if __roverEph (setq _roverEph __roverEph)) _roverEph)
  (:baseEph
   (&optional __baseEph)
   (if __baseEph (setq _baseEph __baseEph)) _baseEph)
  (:baseAntcount
   (&optional __baseAntcount)
   (if __baseAntcount (setq _baseAntcount __baseAntcount)) _baseAntcount)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32[3] _BaseLLA
    (* 4    3)
    ;; uint32 _cycle_slip_count
    4
    ;; uint32 _roverObs
    4
    ;; uint32 _baseObs
    4
    ;; uint32 _roverEph
    4
    ;; uint32 _baseEph
    4
    ;; uint32 _baseAntcount
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32[3] _BaseLLA
     (dotimes (i 3)
       (sys::poke (elt _BaseLLA i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; uint32 _cycle_slip_count
       (write-long _cycle_slip_count s)
     ;; uint32 _roverObs
       (write-long _roverObs s)
     ;; uint32 _baseObs
       (write-long _baseObs s)
     ;; uint32 _roverEph
       (write-long _roverEph s)
     ;; uint32 _baseEph
       (write-long _baseEph s)
     ;; uint32 _baseAntcount
       (write-long _baseAntcount s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32[3] _BaseLLA
   (dotimes (i (length _BaseLLA))
     (setf (elt _BaseLLA i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; uint32 _cycle_slip_count
     (setq _cycle_slip_count (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _roverObs
     (setq _roverObs (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _baseObs
     (setq _baseObs (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _roverEph
     (setq _roverEph (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _baseEph
     (setq _baseEph (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _baseAntcount
     (setq _baseAntcount (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get inertial_sense::RTKInfo :md5sum-) "0f06cd1205181677917f42a40817ccb4")
(setf (get inertial_sense::RTKInfo :datatype-) "inertial_sense/RTKInfo")
(setf (get inertial_sense::RTKInfo :definition-)
      "Header header

float32[3] BaseLLA 			# base position in lat-lon-altitude (deg, deg, m)
uint32 cycle_slip_count 	# number of cycle slips detected
uint32 roverObs				# number of observations from rover (GPS, Glonass, Gallileo, Beidou, Qzs)
uint32 baseObs				# number of observations from base (GPS, Glonass, Gallileo, Beidou, Qzs)
uint32 roverEph				# number of ephemeris messages from rover (GPS, Glonass, Gallileo, Beidou, Qzs)
uint32 baseEph				# number of ephemeris messages from rover (GPS, Glonass, Gallileo, Beidou, Qzs)
uint32 baseAntcount			# number of base station antenna position measurements


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

")



(provide :inertial_sense/RTKInfo "0f06cd1205181677917f42a40817ccb4")


