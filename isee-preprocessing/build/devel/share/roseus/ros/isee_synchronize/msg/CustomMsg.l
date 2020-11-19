;; Auto-generated. Do not edit!


(when (boundp 'isee_synchronize::CustomMsg)
  (if (not (find-package "ISEE_SYNCHRONIZE"))
    (make-package "ISEE_SYNCHRONIZE"))
  (shadow 'CustomMsg (find-package "ISEE_SYNCHRONIZE")))
(unless (find-package "ISEE_SYNCHRONIZE::CUSTOMMSG")
  (make-package "ISEE_SYNCHRONIZE::CUSTOMMSG"))

(in-package "ROS")
;;//! \htmlinclude CustomMsg.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass isee_synchronize::CustomMsg
  :super ros::object
  :slots (_header _timebase _point_num _lidar_id _rsvd _points ))

(defmethod isee_synchronize::CustomMsg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:timebase __timebase) 0)
    ((:point_num __point_num) 0)
    ((:lidar_id __lidar_id) 0)
    ((:rsvd __rsvd) (make-array 3 :initial-element 0 :element-type :char))
    ((:points __points) (let (r) (dotimes (i 0) (push (instance isee_synchronize::CustomPoint :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _timebase (round __timebase))
   (setq _point_num (round __point_num))
   (setq _lidar_id (round __lidar_id))
   (setq _rsvd __rsvd)
   (setq _points __points)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:timebase
   (&optional __timebase)
   (if __timebase (setq _timebase __timebase)) _timebase)
  (:point_num
   (&optional __point_num)
   (if __point_num (setq _point_num __point_num)) _point_num)
  (:lidar_id
   (&optional __lidar_id)
   (if __lidar_id (setq _lidar_id __lidar_id)) _lidar_id)
  (:rsvd
   (&optional __rsvd)
   (if __rsvd (setq _rsvd __rsvd)) _rsvd)
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint64 _timebase
    8
    ;; uint32 _point_num
    4
    ;; uint8 _lidar_id
    1
    ;; uint8[3] _rsvd
    (* 1    3)
    ;; isee_synchronize/CustomPoint[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint64 _timebase
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _timebase (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _timebase) (= (length (_timebase . bv)) 2)) ;; bignum
              (write-long (ash (elt (_timebase . bv) 0) 0) s)
              (write-long (ash (elt (_timebase . bv) 1) -1) s))
             ((and (class _timebase) (= (length (_timebase . bv)) 1)) ;; big1
              (write-long (elt (_timebase . bv) 0) s)
              (write-long (if (>= _timebase 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _timebase s)(write-long (if (>= _timebase 0) 0 #xffffffff) s)))
     ;; uint32 _point_num
       (write-long _point_num s)
     ;; uint8 _lidar_id
       (write-byte _lidar_id s)
     ;; uint8[3] _rsvd
     (princ _rsvd s)
     ;; isee_synchronize/CustomPoint[] _points
     (write-long (length _points) s)
     (dolist (elem _points)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint64 _timebase
     
#+(or :alpha :irix6 :x86_64)
      (setf _timebase (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _timebase (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; uint32 _point_num
     (setq _point_num (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _lidar_id
     (setq _lidar_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8[3] _rsvd
   (setq _rsvd (make-array 3 :element-type :char))
   (replace _rsvd buf :start2 ptr-) (incf ptr- 3)
   ;; isee_synchronize/CustomPoint[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance isee_synchronize::CustomPoint :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get isee_synchronize::CustomMsg :md5sum-) "aa196d9cdf97226174bd64be4d696f75")
(setf (get isee_synchronize::CustomMsg :datatype-) "isee_synchronize/CustomMsg")
(setf (get isee_synchronize::CustomMsg :definition-)
      "# Livox publish pointcloud msg format.

Header header             # ROS standard message header
uint64 timebase           # The time of first point
uint32 point_num          # Total number of pointclouds
uint8  lidar_id           # Lidar device id number
uint8[3]  rsvd            # Reserved use
CustomPoint[] points      # Pointcloud data


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
MSG: isee_synchronize/CustomPoint
# Livox costom pointcloud format.

uint32 offset_time      # offset time relative to the base time
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8 reflectivity      # reflectivity, 0~255
uint8 line              # laser number in lidar


")



(provide :isee_synchronize/CustomMsg "aa196d9cdf97226174bd64be4d696f75")


