
(cl:in-package :asdf)

(defsystem "srcp2_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "BatteryStateMsg" :depends-on ("_package_BatteryStateMsg"))
    (:file "_package_BatteryStateMsg" :depends-on ("_package"))
    (:file "ScoreMsg" :depends-on ("_package_ScoreMsg"))
    (:file "_package_ScoreMsg" :depends-on ("_package"))
    (:file "VolSensorMsg" :depends-on ("_package_VolSensorMsg"))
    (:file "_package_VolSensorMsg" :depends-on ("_package"))
  ))