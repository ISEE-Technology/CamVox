;; Auto-generated. Do not edit!


(when (boundp 'isee_synchronize::FirmwareUpdate)
  (if (not (find-package "ISEE_SYNCHRONIZE"))
    (make-package "ISEE_SYNCHRONIZE"))
  (shadow 'FirmwareUpdate (find-package "ISEE_SYNCHRONIZE")))
(unless (find-package "ISEE_SYNCHRONIZE::FIRMWAREUPDATE")
  (make-package "ISEE_SYNCHRONIZE::FIRMWAREUPDATE"))
(unless (find-package "ISEE_SYNCHRONIZE::FIRMWAREUPDATEREQUEST")
  (make-package "ISEE_SYNCHRONIZE::FIRMWAREUPDATEREQUEST"))
(unless (find-package "ISEE_SYNCHRONIZE::FIRMWAREUPDATERESPONSE")
  (make-package "ISEE_SYNCHRONIZE::FIRMWAREUPDATERESPONSE"))

(in-package "ROS")





(defclass isee_synchronize::FirmwareUpdateRequest
  :super ros::object
  :slots (_filename ))

(defmethod isee_synchronize::FirmwareUpdateRequest
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

(defclass isee_synchronize::FirmwareUpdateResponse
  :super ros::object
  :slots (_success _message ))

(defmethod isee_synchronize::FirmwareUpdateResponse
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

(defclass isee_synchronize::FirmwareUpdate
  :super ros::object
  :slots ())

(setf (get isee_synchronize::FirmwareUpdate :md5sum-) "582779c570de719ad3dde68b15a648a8")
(setf (get isee_synchronize::FirmwareUpdate :datatype-) "isee_synchronize/FirmwareUpdate")
(setf (get isee_synchronize::FirmwareUpdate :request) isee_synchronize::FirmwareUpdateRequest)
(setf (get isee_synchronize::FirmwareUpdate :response) isee_synchronize::FirmwareUpdateResponse)

(defmethod isee_synchronize::FirmwareUpdateRequest
  (:response () (instance isee_synchronize::FirmwareUpdateResponse :init)))

(setf (get isee_synchronize::FirmwareUpdateRequest :md5sum-) "582779c570de719ad3dde68b15a648a8")
(setf (get isee_synchronize::FirmwareUpdateRequest :datatype-) "isee_synchronize/FirmwareUpdateRequest")
(setf (get isee_synchronize::FirmwareUpdateRequest :definition-)
      "string filename
---
bool success
string message
")

(setf (get isee_synchronize::FirmwareUpdateResponse :md5sum-) "582779c570de719ad3dde68b15a648a8")
(setf (get isee_synchronize::FirmwareUpdateResponse :datatype-) "isee_synchronize/FirmwareUpdateResponse")
(setf (get isee_synchronize::FirmwareUpdateResponse :definition-)
      "string filename
---
bool success
string message
")



(provide :isee_synchronize/FirmwareUpdate "582779c570de719ad3dde68b15a648a8")


