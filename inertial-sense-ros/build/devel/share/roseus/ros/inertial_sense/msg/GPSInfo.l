;; Auto-generated. Do not edit!


(when (boundp 'inertial_sense::GPSInfo)
  (if (not (find-package "INERTIAL_SENSE"))
    (make-package "INERTIAL_SENSE"))
  (shadow 'GPSInfo (find-package "INERTIAL_SENSE")))
(unless (find-package "INERTIAL_SENSE::GPSINFO")
  (make-package "INERTIAL_SENSE::GPSINFO"))

(in-package "ROS")
;;//! \htmlinclude GPSInfo.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass inertial_sense::GPSInfo
  :super ros::object
  :slots (_header _num_sats _sattelite_info ))

(defmethod inertial_sense::GPSInfo
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:num_sats __num_sats) 0)
    ((:sattelite_info __sattelite_info) (let (r) (dotimes (i 50) (push (instance inertial_sense::SatInfo :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _num_sats (round __num_sats))
   (setq _sattelite_info __sattelite_info)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:num_sats
   (&optional __num_sats)
   (if __num_sats (setq _num_sats __num_sats)) _num_sats)
  (:sattelite_info
   (&rest __sattelite_info)
   (if (keywordp (car __sattelite_info))
       (send* _sattelite_info __sattelite_info)
     (progn
       (if __sattelite_info (setq _sattelite_info (car __sattelite_info)))
       _sattelite_info)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _num_sats
    4
    ;; inertial_sense/SatInfo[50] _sattelite_info
    (apply #'+ (send-all _sattelite_info :serialization-length))
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _num_sats
       (write-long _num_sats s)
     ;; inertial_sense/SatInfo[50] _sattelite_info
     (dolist (elem _sattelite_info)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _num_sats
     (setq _num_sats (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; inertial_sense/SatInfo[50] _sattelite_info
   (dotimes (i 50)
     (send (elt _sattelite_info i) :deserialize buf ptr-) (incf ptr- (send (elt _sattelite_info i) :serialization-length))
     )
   ;;
   self)
  )

(setf (get inertial_sense::GPSInfo :md5sum-) "a60e054a01708e390d6fe69c6b4a8303")
(setf (get inertial_sense::GPSInfo :datatype-) "inertial_sense/GPSInfo")
(setf (get inertial_sense::GPSInfo :definition-)
      "
Header header
uint32 num_sats            		# number of sattelites in the sky
SatInfo[50] sattelite_info	 	# information about sattelites
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
MSG: inertial_sense/SatInfo
uint32 sat_id # sattelite id
uint32 cno    # Carrier to noise ratio
")



(provide :inertial_sense/GPSInfo "a60e054a01708e390d6fe69c6b4a8303")


