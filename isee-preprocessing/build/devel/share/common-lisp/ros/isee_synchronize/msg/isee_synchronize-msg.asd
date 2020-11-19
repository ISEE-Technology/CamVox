
(cl:in-package :asdf)

(defsystem "isee_synchronize-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CustomMsg" :depends-on ("_package_CustomMsg"))
    (:file "_package_CustomMsg" :depends-on ("_package"))
    (:file "CustomPoint" :depends-on ("_package_CustomPoint"))
    (:file "_package_CustomPoint" :depends-on ("_package"))
    (:file "GPS" :depends-on ("_package_GPS"))
    (:file "_package_GPS" :depends-on ("_package"))
  ))