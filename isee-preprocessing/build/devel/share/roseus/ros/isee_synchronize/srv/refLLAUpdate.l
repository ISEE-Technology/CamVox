;; Auto-generated. Do not edit!


(when (boundp 'isee_synchronize::refLLAUpdate)
  (if (not (find-package "ISEE_SYNCHRONIZE"))
    (make-package "ISEE_SYNCHRONIZE"))
  (shadow 'refLLAUpdate (find-package "ISEE_SYNCHRONIZE")))
(unless (find-package "ISEE_SYNCHRONIZE::REFLLAUPDATE")
  (make-package "ISEE_SYNCHRONIZE::REFLLAUPDATE"))
(unless (find-package "ISEE_SYNCHRONIZE::REFLLAUPDATEREQUEST")
  (make-package "ISEE_SYNCHRONIZE::REFLLAUPDATEREQUEST"))
(unless (find-package "ISEE_SYNCHRONIZE::REFLLAUPDATERESPONSE")
  (make-package "ISEE_SYNCHRONIZE::REFLLAUPDATERESPONSE"))

(in-package "ROS")





(defclass isee_synchronize::refLLAUpdateRequest
  :super ros::object
  :slots (_lla ))

(defmethod isee_synchronize::refLLAUpdateRequest
  (:init
   (&key
    ((:lla __lla) (make-array 3 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _lla __lla)
   self)
  (:lla
   (&optional __lla)
   (if __lla (setq _lla __lla)) _lla)
  (:serialization-length
   ()
   (+
    ;; float64[3] _lla
    (* 8    3)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[3] _lla
     (dotimes (i 3)
       (sys::poke (elt _lla i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[3] _lla
   (dotimes (i (length _lla))
     (setf (elt _lla i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(defclass isee_synchronize::refLLAUpdateResponse
  :super ros::object
  :slots (_success _message ))

(defmethod isee_synchronize::refLLAUpdateResponse
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

(defclass isee_synchronize::refLLAUpdate
  :super ros::object
  :slots ())

(setf (get isee_synchronize::refLLAUpdate :md5sum-) "b307ff2c781bd2350d5c9875e1ae9b16")
(setf (get isee_synchronize::refLLAUpdate :datatype-) "isee_synchronize/refLLAUpdate")
(setf (get isee_synchronize::refLLAUpdate :request) isee_synchronize::refLLAUpdateRequest)
(setf (get isee_synchronize::refLLAUpdate :response) isee_synchronize::refLLAUpdateResponse)

(defmethod isee_synchronize::refLLAUpdateRequest
  (:response () (instance isee_synchronize::refLLAUpdateResponse :init)))

(setf (get isee_synchronize::refLLAUpdateRequest :md5sum-) "b307ff2c781bd2350d5c9875e1ae9b16")
(setf (get isee_synchronize::refLLAUpdateRequest :datatype-) "isee_synchronize/refLLAUpdateRequest")
(setf (get isee_synchronize::refLLAUpdateRequest :definition-)
      "float64[3] lla
---
bool success
string message
")

(setf (get isee_synchronize::refLLAUpdateResponse :md5sum-) "b307ff2c781bd2350d5c9875e1ae9b16")
(setf (get isee_synchronize::refLLAUpdateResponse :datatype-) "isee_synchronize/refLLAUpdateResponse")
(setf (get isee_synchronize::refLLAUpdateResponse :definition-)
      "float64[3] lla
---
bool success
string message
")



(provide :isee_synchronize/refLLAUpdate "b307ff2c781bd2350d5c9875e1ae9b16")


