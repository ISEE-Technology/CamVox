;; Auto-generated. Do not edit!


(when (boundp 'inertial_sense::SatInfo)
  (if (not (find-package "INERTIAL_SENSE"))
    (make-package "INERTIAL_SENSE"))
  (shadow 'SatInfo (find-package "INERTIAL_SENSE")))
(unless (find-package "INERTIAL_SENSE::SATINFO")
  (make-package "INERTIAL_SENSE::SATINFO"))

(in-package "ROS")
;;//! \htmlinclude SatInfo.msg.html


(defclass inertial_sense::SatInfo
  :super ros::object
  :slots (_sat_id _cno ))

(defmethod inertial_sense::SatInfo
  (:init
   (&key
    ((:sat_id __sat_id) 0)
    ((:cno __cno) 0)
    )
   (send-super :init)
   (setq _sat_id (round __sat_id))
   (setq _cno (round __cno))
   self)
  (:sat_id
   (&optional __sat_id)
   (if __sat_id (setq _sat_id __sat_id)) _sat_id)
  (:cno
   (&optional __cno)
   (if __cno (setq _cno __cno)) _cno)
  (:serialization-length
   ()
   (+
    ;; uint32 _sat_id
    4
    ;; uint32 _cno
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _sat_id
       (write-long _sat_id s)
     ;; uint32 _cno
       (write-long _cno s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _sat_id
     (setq _sat_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _cno
     (setq _cno (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get inertial_sense::SatInfo :md5sum-) "f1fb6b174b603bb921293910a7f10d63")
(setf (get inertial_sense::SatInfo :datatype-) "inertial_sense/SatInfo")
(setf (get inertial_sense::SatInfo :definition-)
      "uint32 sat_id # sattelite id
uint32 cno    # Carrier to noise ratio
")



(provide :inertial_sense/SatInfo "f1fb6b174b603bb921293910a7f10d63")


