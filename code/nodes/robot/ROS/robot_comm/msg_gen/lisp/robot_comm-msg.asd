
(cl:in-package :asdf)

(defsystem "robot_comm-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "robot_CartesianLog" :depends-on ("_package_robot_CartesianLog"))
    (:file "_package_robot_CartesianLog" :depends-on ("_package"))
    (:file "robot_ForceLog" :depends-on ("_package_robot_ForceLog"))
    (:file "_package_robot_ForceLog" :depends-on ("_package"))
    (:file "robot_JointsLog" :depends-on ("_package_robot_JointsLog"))
    (:file "_package_robot_JointsLog" :depends-on ("_package"))
  ))