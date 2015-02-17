
(cl:in-package :asdf)

(defsystem "hand_comm-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "hand_ForcesLog" :depends-on ("_package_hand_ForcesLog"))
    (:file "_package_hand_ForcesLog" :depends-on ("_package"))
    (:file "hand_RawForcesLog" :depends-on ("_package_hand_RawForcesLog"))
    (:file "_package_hand_RawForcesLog" :depends-on ("_package"))
    (:file "hand_AnglesLog" :depends-on ("_package_hand_AnglesLog"))
    (:file "_package_hand_AnglesLog" :depends-on ("_package"))
    (:file "hand_EncodersLog" :depends-on ("_package_hand_EncodersLog"))
    (:file "_package_hand_EncodersLog" :depends-on ("_package"))
  ))