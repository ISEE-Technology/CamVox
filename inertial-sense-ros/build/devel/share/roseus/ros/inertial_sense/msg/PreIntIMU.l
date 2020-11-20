;; Auto-generated. Do not edit!


(when (boundp 'inertial_sense::PreIntIMU)
  (if (not (find-package "INERTIAL_SENSE"))
    (make-package "INERTIAL_SENSE"))
  (shadow 'PreIntIMU (find-package "INERTIAL_SENSE")))
(unless (find-package "INERTIAL_SENSE::PREINTIMU")
  (make-package "INERTIAL_SENSE::PREINTIMU"))

(in-package "ROS")
;;//! \htmlinclude PreIntIMU.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass inertial_sense::PreIntIMU
  :super ros::object
  :slots (_header _dtheta _dvel _dt ))

(defmethod inertial_sense::PreIntIMU
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:dtheta __dtheta) (instance geometry_msgs::Vector3 :init))
    ((:dvel __dvel) (instance geometry_msgs::Vector3 :init))
    ((:dt __dt) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _dtheta __dtheta)
   (setq _dvel __dvel)
   (setq _dt (float __dt))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:dtheta
   (&rest __dtheta)
   (if (keywordp (car __dtheta))
       (send* _dtheta __dtheta)
     (progn
       (if __dtheta (setq _dtheta (car __dtheta)))
       _dtheta)))
  (:dvel
   (&rest __dvel)
   (if (keywordp (car __dvel))
       (send* _dvel __dvel)
     (progn
       (if __dvel (setq _dvel (car __dvel)))
       _dvel)))
  (:dt
   (&optional __dt)
   (if __dt (setq _dt __dt)) _dt)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Vector3 _dtheta
    (send _dtheta :serialization-length)
    ;; geometry_msgs/Vector3 _dvel
    (send _dvel :serialization-length)
    ;; float64 _dt
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Vector3 _dtheta
       (send _dtheta :serialize s)
     ;; geometry_msgs/Vector3 _dvel
       (send _dvel :serialize s)
     ;; float64 _dt
       (sys::poke _dt (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Vector3 _dtheta
     (send _dtheta :deserialize buf ptr-) (incf ptr- (send _dtheta :serialization-length))
   ;; geometry_msgs/Vector3 _dvel
     (send _dvel :deserialize buf ptr-) (incf ptr- (send _dvel :serialization-length))
   ;; float64 _dt
     (setq _dt (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get inertial_sense::PreIntIMU :md5sum-) "8cedd64d41ec2b2b45424a92ffe74e5a")
(setf (get inertial_sense::PreIntIMU :datatype-) "inertial_sense/PreIntIMU")
(setf (get inertial_sense::PreIntIMU :definition-)
      "Header header
geometry_msgs/Vector3 dtheta 	# change in angle over time period (rodriguez vector)
geometry_msgs/Vector3 dvel		# change in velocity over time period (m/s)
float64 dt 						# length of time period (s)
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



(provide :inertial_sense/PreIntIMU "8cedd64d41ec2b2b45424a92ffe74e5a")


