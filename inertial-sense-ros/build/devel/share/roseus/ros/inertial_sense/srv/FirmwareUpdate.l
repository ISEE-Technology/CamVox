;; Auto-generated. Do not edit!


(when (boundp 'inertial_sense::FirmwareUpdate)
  (if (not (find-package "INERTIAL_SENSE"))
    (make-package "INERTIAL_SENSE"))
  (shadow 'FirmwareUpdate (find-package "INERTIAL_SENSE")))
(unless (find-package "INERTIAL_SENSE::FIRMWAREUPDATE")
  (make-package "INERTIAL_SENSE::FIRMWAREUPDATE"))
(unless (find-package "INERTIAL_SENSE::FIRMWAREUPDATEREQUEST")
  (make-package "INERTIAL_SENSE::FIRMWAREUPDATEREQUEST"))
(unless (find-package "INERTIAL_SENSE::FIRMWAREUPDATERESPONSE")
  (make-package "INERTIAL_SENSE::FIRMWAREUPDATERESPONSE"))

(in-package "ROS")





(defclass inertial_sense::FirmwareUpdateRequest
  :super ros::object
  :slots (_filename ))

(defmethod inertial_sense::FirmwareUpdateRequest
  (:init
   (&key
    ((:filename __filename) "")
    )
   (send-super :init)
   (setq _filename (string __filename))
   self)
  (:filename
   (&optional __filename)
   (if __filename (setq _filename __filename)) _filename)
  (:serialization-length
   ()
   (+
    ;; string _filename
    4 (length _filename)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _filename
       (write-long (length _filename) s) (princ _filename s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _filename
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _filename (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass inertial_sense::FirmwareUpdateResponse
  :super ros::object
  :slots (_success _message ))

(defmethod inertial_sense::FirmwareUpdateResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass inertial_sense::FirmwareUpdate
  :super ros::object
  :slots ())

(setf (get inertial_sense::FirmwareUpdate :md5sum-) "582779c570de719ad3dde68b15a648a8")
(setf (get inertial_sense::FirmwareUpdate :datatype-) "inertial_sense/FirmwareUpdate")
(setf (get inertial_sense::FirmwareUpdate :request) inertial_sense::FirmwareUpdateRequest)
(setf (get inertial_sense::FirmwareUpdate :response) inertial_sense::FirmwareUpdateResponse)

(defmethod inertial_sense::FirmwareUpdateRequest
  (:response () (instance inertial_sense::FirmwareUpdateResponse :init)))

(setf (get inertial_sense::FirmwareUpdateRequest :md5sum-) "582779c570de719ad3dde68b15a648a8")
(setf (get inertial_sense::FirmwareUpdateRequest :datatype-) "inertial_sense/FirmwareUpdateRequest")
(setf (get inertial_sense::FirmwareUpdateRequest :definition-)
      "string filename
---
bool success
string message
")

(setf (get inertial_sense::FirmwareUpdateResponse :md5sum-) "582779c570de719ad3dde68b15a648a8")
(setf (get inertial_sense::FirmwareUpdateResponse :datatype-) "inertial_sense/FirmwareUpdateResponse")
(setf (get inertial_sense::FirmwareUpdateResponse :definition-)
      "string filename
---
bool success
string message
")



(provide :inertial_sense/FirmwareUpdate "582779c570de719ad3dde68b15a648a8")


