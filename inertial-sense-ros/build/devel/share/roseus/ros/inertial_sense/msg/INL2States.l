;; Auto-generated. Do not edit!


(when (boundp 'inertial_sense::INL2States)
  (if (not (find-package "INERTIAL_SENSE"))
    (make-package "INERTIAL_SENSE"))
  (shadow 'INL2States (find-package "INERTIAL_SENSE")))
(unless (find-package "INERTIAL_SENSE::INL2STATES")
  (make-package "INERTIAL_SENSE::INL2STATES"))

(in-package "ROS")
;;//! \htmlinclude INL2States.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass inertial_sense::INL2States
  :super ros::object
  :slots (_header _quatEcef _velEcef _posEcef _gyroBias _accelBias _baroBias _magDec _magInc ))

(defmethod inertial_sense::INL2States
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:quatEcef __quatEcef) (instance geometry_msgs::Quaternion :init))
    ((:velEcef __velEcef) (instance geometry_msgs::Vector3 :init))
    ((:posEcef __posEcef) (instance geometry_msgs::Vector3 :init))
    ((:gyroBias __gyroBias) (instance geometry_msgs::Vector3 :init))
    ((:accelBias __accelBias) (instance geometry_msgs::Vector3 :init))
    ((:baroBias __baroBias) 0.0)
    ((:magDec __magDec) 0.0)
    ((:magInc __magInc) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _quatEcef __quatEcef)
   (setq _velEcef __velEcef)
   (setq _posEcef __posEcef)
   (setq _gyroBias __gyroBias)
   (setq _accelBias __accelBias)
   (setq _baroBias (float __baroBias))
   (setq _magDec (float __magDec))
   (setq _magInc (float __magInc))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:quatEcef
   (&rest __quatEcef)
   (if (keywordp (car __quatEcef))
       (send* _quatEcef __quatEcef)
     (progn
       (if __quatEcef (setq _quatEcef (car __quatEcef)))
       _quatEcef)))
  (:velEcef
   (&rest __velEcef)
   (if (keywordp (car __velEcef))
       (send* _velEcef __velEcef)
     (progn
       (if __velEcef (setq _velEcef (car __velEcef)))
       _velEcef)))
  (:posEcef
   (&rest __posEcef)
   (if (keywordp (car __posEcef))
       (send* _posEcef __posEcef)
     (progn
       (if __posEcef (setq _posEcef (car __posEcef)))
       _posEcef)))
  (:gyroBias
   (&rest __gyroBias)
   (if (keywordp (car __gyroBias))
       (send* _gyroBias __gyroBias)
     (progn
       (if __gyroBias (setq _gyroBias (car __gyroBias)))
       _gyroBias)))
  (:accelBias
   (&rest __accelBias)
   (if (keywordp (car __accelBias))
       (send* _accelBias __accelBias)
     (progn
       (if __accelBias (setq _accelBias (car __accelBias)))
       _accelBias)))
  (:baroBias
   (&optional __baroBias)
   (if __baroBias (setq _baroBias __baroBias)) _baroBias)
  (:magDec
   (&optional __magDec)
   (if __magDec (setq _magDec __magDec)) _magDec)
  (:magInc
   (&optional __magInc)
   (if __magInc (setq _magInc __magInc)) _magInc)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Quaternion _quatEcef
    (send _quatEcef :serialization-length)
    ;; geometry_msgs/Vector3 _velEcef
    (send _velEcef :serialization-length)
    ;; geometry_msgs/Vector3 _posEcef
    (send _posEcef :serialization-length)
    ;; geometry_msgs/Vector3 _gyroBias
    (send _gyroBias :serialization-length)
    ;; geometry_msgs/Vector3 _accelBias
    (send _accelBias :serialization-length)
    ;; float32 _baroBias
    4
    ;; float32 _magDec
    4
    ;; float32 _magInc
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Quaternion _quatEcef
       (send _quatEcef :serialize s)
     ;; geometry_msgs/Vector3 _velEcef
       (send _velEcef :serialize s)
     ;; geometry_msgs/Vector3 _posEcef
       (send _posEcef :serialize s)
     ;; geometry_msgs/Vector3 _gyroBias
       (send _gyroBias :serialize s)
     ;; geometry_msgs/Vector3 _accelBias
       (send _accelBias :serialize s)
     ;; float32 _baroBias
       (sys::poke _baroBias (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _magDec
       (sys::poke _magDec (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _magInc
       (sys::poke _magInc (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Quaternion _quatEcef
     (send _quatEcef :deserialize buf ptr-) (incf ptr- (send _quatEcef :serialization-length))
   ;; geometry_msgs/Vector3 _velEcef
     (send _velEcef :deserialize buf ptr-) (incf ptr- (send _velEcef :serialization-length))
   ;; geometry_msgs/Vector3 _posEcef
     (send _posEcef :deserialize buf ptr-) (incf ptr- (send _posEcef :serialization-length))
   ;; geometry_msgs/Vector3 _gyroBias
     (send _gyroBias :deserialize buf ptr-) (incf ptr- (send _gyroBias :serialization-length))
   ;; geometry_msgs/Vector3 _accelBias
     (send _accelBias :deserialize buf ptr-) (incf ptr- (send _accelBias :serialization-length))
   ;; float32 _baroBias
     (setq _baroBias (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _magDec
     (setq _magDec (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _magInc
     (setq _magInc (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get inertial_sense::INL2States :md5sum-) "06de6b8d1957718b78007390d5c6fc67")
(setf (get inertial_sense::INL2States :datatype-) "inertial_sense/INL2States")
(setf (get inertial_sense::INL2States :definition-)
      "Header header                       # GPS time of week (since Sunday morning) in seconds
geometry_msgs/Quaternion quatEcef   # Quaternion body rotation with respect to ECEF
geometry_msgs/Vector3 velEcef       # (m/s) Velocity in ECEF frame
geometry_msgs/Vector3 posEcef       # (m) Position in ECEF frame
geometry_msgs/Vector3 gyroBias      # (rad/s) Gyro bias
geometry_msgs/Vector3 accelBias     # (m/s^2) Accelerometer bias
float32 baroBias                    # (m) Barometer bias
float32 magDec                      # (rad) Magnetic declination
float32 magInc                      # (rad) Magnetic inclination

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
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

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



(provide :inertial_sense/INL2States "06de6b8d1957718b78007390d5c6fc67")


