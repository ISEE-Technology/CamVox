;; Auto-generated. Do not edit!


(when (boundp 'isee_synchronize::CustomPoint)
  (if (not (find-package "ISEE_SYNCHRONIZE"))
    (make-package "ISEE_SYNCHRONIZE"))
  (shadow 'CustomPoint (find-package "ISEE_SYNCHRONIZE")))
(unless (find-package "ISEE_SYNCHRONIZE::CUSTOMPOINT")
  (make-package "ISEE_SYNCHRONIZE::CUSTOMPOINT"))

(in-package "ROS")
;;//! \htmlinclude CustomPoint.msg.html


(defclass isee_synchronize::CustomPoint
  :super ros::object
  :slots (_offset_time _x _y _z _reflectivity _line ))

(defmethod isee_synchronize::CustomPoint
  (:init
   (&key
    ((:offset_time __offset_time) 0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:reflectivity __reflectivity) 0)
    ((:line __line) 0)
    )
   (send-super :init)
   (setq _offset_time (round __offset_time))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _reflectivity (round __reflectivity))
   (setq _line (round __line))
   self)
  (:offset_time
   (&optional __offset_time)
   (if __offset_time (setq _offset_time __offset_time)) _offset_time)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:reflectivity
   (&optional __reflectivity)
   (if __reflectivity (setq _reflectivity __reflectivity)) _reflectivity)
  (:line
   (&optional __line)
   (if __line (setq _line __line)) _line)
  (:serialization-length
   ()
   (+
    ;; uint32 _offset_time
    4
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _z
    4
    ;; uint8 _reflectivity
    1
    ;; uint8 _line
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _offset_time
       (write-long _offset_time s)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z
       (sys::poke _z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _reflectivity
       (write-byte _reflectivity s)
     ;; uint8 _line
       (write-byte _line s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _offset_time
     (setq _offset_time (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _reflectivity
     (setq _reflectivity (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _line
     (setq _line (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get isee_synchronize::CustomPoint :md5sum-) "d852cd86aa9d2b697229026ff07cf452")
(setf (get isee_synchronize::CustomPoint :datatype-) "isee_synchronize/CustomPoint")
(setf (get isee_synchronize::CustomPoint :definition-)
      "# Livox costom pointcloud format.

uint32 offset_time      # offset time relative to the base time
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8 reflectivity      # reflectivity, 0~255
uint8 line              # laser number in lidar


")



(provide :isee_synchronize/CustomPoint "d852cd86aa9d2b697229026ff07cf452")


