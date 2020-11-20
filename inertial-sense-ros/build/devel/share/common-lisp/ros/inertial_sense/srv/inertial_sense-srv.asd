
(cl:in-package :asdf)

(defsystem "inertial_sense-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "FirmwareUpdate" :depends-on ("_package_FirmwareUpdate"))
    (:file "_package_FirmwareUpdate" :depends-on ("_package"))
    (:file "refLLAUpdate" :depends-on ("_package_refLLAUpdate"))
    (:file "_package_refLLAUpdate" :depends-on ("_package"))
  ))