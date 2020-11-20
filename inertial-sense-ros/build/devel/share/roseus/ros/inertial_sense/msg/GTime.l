;; Auto-generated. Do not edit!


(when (boundp 'inertial_sense::GTime)
  (if (not (find-package "INERTIAL_SENSE"))
    (make-package "INERTIAL_SENSE"))
  (shadow 'GTime (find-package "INERTIAL_SENSE")))
(unless (find-package "INERTIAL_SENSE::GTIME")
  (make-package "INERTIAL_SENSE::GTIME"))

(in-package "ROS")
;;//! \htmlinclude GTime.msg.html


(defclass inertial_sense::GTime
  :super ros::object
  :slots (_time _sec ))

(defmethod inertial_sense::GTime
  (:init
   (&key
    ((:time __time) 0)
    ((:sec __sec) 0.0)
    )
   (send-super :init)
   (setq _time (round __time))
   (setq _sec (float __sec))
   self)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:sec
   (&optional __sec)
   (if __sec (setq _sec __sec)) _sec)
  (:serialization-length
   ()
   (+
    ;; int64 _time
    8
    ;; float64 _sec
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _time
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _time (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _time) (= (length (_time . bv)) 2)) ;; bignum
              (write-long (ash (elt (_time . bv) 0) 0) s)
              (write-long (ash (elt (_time . bv) 1) -1) s))
             ((and (class _time) (= (length (_time . bv)) 1)) ;; big1
              (write-long (elt (_time . bv) 0) s)
              (write-long (if (>= _time 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _time s)(write-long (if (>= _time 0) 0 #xffffffff) s)))
     ;; float64 _sec
       (sys::poke _sec (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _time
#+(or :alpha :irix6 :x86_64)
      (setf _time (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _time (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64 _sec
     (setq _sec (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get inertial_sense::GTime :md5sum-) "f27debe4939acc53511906e7b35f8f58")
(setf (get inertial_sense::GTime :datatype-) "inertial_sense/GTime")
(setf (get inertial_sense::GTime :definition-)
      "int64 time
float64 sec
")



(provide :inertial_sense/GTime "f27debe4939acc53511906e7b35f8f58")


