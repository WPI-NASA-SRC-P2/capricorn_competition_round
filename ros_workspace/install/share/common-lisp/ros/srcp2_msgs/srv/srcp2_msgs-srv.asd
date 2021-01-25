
(cl:in-package :asdf)

(defsystem "srcp2_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "BrakeRoverSrv" :depends-on ("_package_BrakeRoverSrv"))
    (:file "_package_BrakeRoverSrv" :depends-on ("_package"))
    (:file "ChargingStationSrv" :depends-on ("_package_ChargingStationSrv"))
    (:file "_package_ChargingStationSrv" :depends-on ("_package"))
    (:file "LocalizationSrv" :depends-on ("_package_LocalizationSrv"))
    (:file "_package_LocalizationSrv" :depends-on ("_package"))
    (:file "StartStopSrv" :depends-on ("_package_StartStopSrv"))
    (:file "_package_StartStopSrv" :depends-on ("_package"))
    (:file "ToggleLightSrv" :depends-on ("_package_ToggleLightSrv"))
    (:file "_package_ToggleLightSrv" :depends-on ("_package"))
  ))