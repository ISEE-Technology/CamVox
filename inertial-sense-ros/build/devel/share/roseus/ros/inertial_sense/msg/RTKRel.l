;; Auto-generated. Do not edit!


(when (boundp 'inertial_sense::RTKRel)
  (if (not (find-package "INERTIAL_SENSE"))
    (make-package "INERTIAL_SENSE"))
  (shadow 'RTKRel (find-package "INERTIAL_SENSE")))
(unless (find-package "INERTIAL_SENSE::RTKREL")
  (make-package "INERTIAL_SENSE::RTKREL"))

(in-package "ROS")
;;//! \htmlinclude RTKRel.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass inertial_sense::RTKRel
  :super ros::object
  :slots (_header _differential_age _ar_ratio _vector_base_to_rover _distance_base_to_rover _heading_base_to_rover ))

(defmethod inertial_sense::RTKRel
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:differential_age __differential_age) 0.0)
    ((:ar_ratio __ar_ratio) 0.0)
    ((:vector_base_to_rover __vector_base_to_rover) (instance geometry_msgs::Vector3 :init))
    ((:distance_base_to_rover __distance_base_to_rover) 0.0)
    ((:heading_base_to_rover __heading_base_to_rover) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _differential_age (float __differential_age))
   (setq _ar_ratio (float __ar_ratio))
   (setq _vector_base_to_rover __vector_base_to_rover)
   (setq _distance_base_to_rover (float __distance_base_to_rover))
   (setq _heading_base_to_rover (float __heading_base_to_rover))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:differential_age
   (&optional __differential_age)
   (if __differential_age (setq _differential_age __differential_age)) _differential_age)
  (:ar_ratio
   (&optional __ar_ratio)
   (if __ar_ratio (setq _ar_ratio __ar_ratio)) _ar_ratio)
  (:vector_base_to_rover
   (&rest __vector_base_to_rover)
   (if (keywordp (car __vector_base_to_rover))
       (send* _vector_base_to_rover __vector_base_to_rover)
     (progn
       (if __vector_base_to_rover (setq _vector_base_to_rover (car __vector_base_to_rover)))
       _vector_base_to_rover)))
  (:distance_base_to_rover
   (&optional __distance_base_to_rover)
   (if __distance_base_to_rover (setq _distance_base_to_rover __distance_base_to_rover)) _distance_base_to_rover)
  (:heading_base_to_rover
   (&optional __heading_base_to_rover)
   (if __heading_base_to_rover (setq _heading_base_to_rover __heading_base_to_rover)) _heading_base_to_rover)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _differential_age
    4
    ;; float32 _ar_ratio
    4
    ;; geometry_msgs/Vector3 _vector_base_to_rover
    (send _vector_base_to_rover :serialization-length)
    ;; float32 _distance_base_to_rover
    4
    ;; float32 _heading_base_to_rover
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _differential_age
       (sys::poke _differential_age (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ar_ratio
       (sys::poke _ar_ratio (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; geometry_msgs/Vector3 _vector_base_to_rover
       (send _vector_base_to_rover :serialize s)
     ;; float32 _distance_base_to_rover
       (sys::poke _distance_base_to_rover (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _heading_base_to_rover
       (sys::poke _heading_base_to_rover (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _differential_age
     (setq _differential_age (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ar_ratio
     (setq _ar_ratio (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; geometry_msgs/Vector3 _vector_base_to_rover
     (send _vector_base_to_rover :deserialize buf ptr-) (incf ptr- (send _vector_base_to_rover :serialization-length))
   ;; float32 _distance_base_to_rover
     (setq _distance_base_to_rover (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _heading_base_to_rover
     (setq _heading_base_to_rover (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get inertial_sense::RTKRel :md5sum-) "ff67521259c93bfc5ebd55747655bff6")
(setf (get inertial_sense::RTKRel :datatype-) "inertial_sense/RTKRel")
(setf (get inertial_sense::RTKRel :definition-)
      "Header header
float32 differential_age 				# Age of differential (seconds)
float32 ar_ratio 						# Ambiguity resolution ratio factor for validation
geometry_msgs/Vector3 vector_base_to_rover 	# Vector to base (m) in ECEF - If Compassing enabled, this is the 3-vector from antenna 2 to antenna 1
float32 distance_base_to_rover 				# Distance to Base (m)
float32 heading_base_to_rover 				# Angle from north to vectorToBase in local tangent plane. (rad)
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



(provide :inertial_sense/RTKRel "ff67521259c93bfc5ebd55747655bff6")


