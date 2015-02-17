
(cl:in-package :asdf)

(defsystem "vision_comm-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CaptureImage" :depends-on ("_package_CaptureImage"))
    (:file "_package_CaptureImage" :depends-on ("_package"))
    (:file "GetInfo" :depends-on ("_package_GetInfo"))
    (:file "_package_GetInfo" :depends-on ("_package"))
    (:file "PingVision" :depends-on ("_package_PingVision"))
    (:file "_package_PingVision" :depends-on ("_package"))
    (:file "DropDetect" :depends-on ("_package_DropDetect"))
    (:file "_package_DropDetect" :depends-on ("_package"))
    (:file "CalibrateVision" :depends-on ("_package_CalibrateVision"))
    (:file "_package_CalibrateVision" :depends-on ("_package"))
    (:file "PlaceDetect" :depends-on ("_package_PlaceDetect"))
    (:file "_package_PlaceDetect" :depends-on ("_package"))
    (:file "InsertDetect" :depends-on ("_package_InsertDetect"))
    (:file "_package_InsertDetect" :depends-on ("_package"))
  ))