
(cl:in-package :asdf)

(defsystem "hand_comm-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "hand_SetEncoder" :depends-on ("_package_hand_SetEncoder"))
    (:file "_package_hand_SetEncoder" :depends-on ("_package"))
    (:file "hand_SetForce" :depends-on ("_package_hand_SetForce"))
    (:file "_package_hand_SetForce" :depends-on ("_package"))
    (:file "hand_IsMoving" :depends-on ("_package_hand_IsMoving"))
    (:file "_package_hand_IsMoving" :depends-on ("_package"))
    (:file "hand_DADA" :depends-on ("_package_hand_DADA"))
    (:file "_package_hand_DADA" :depends-on ("_package"))
    (:file "hand_GetAngles" :depends-on ("_package_hand_GetAngles"))
    (:file "_package_hand_GetAngles" :depends-on ("_package"))
    (:file "hand_GetRawForces" :depends-on ("_package_hand_GetRawForces"))
    (:file "_package_hand_GetRawForces" :depends-on ("_package"))
    (:file "hand_Ping" :depends-on ("_package_hand_Ping"))
    (:file "_package_hand_Ping" :depends-on ("_package"))
    (:file "hand_SetRest" :depends-on ("_package_hand_SetRest"))
    (:file "_package_hand_SetRest" :depends-on ("_package"))
    (:file "hand_SetSpeed" :depends-on ("_package_hand_SetSpeed"))
    (:file "_package_hand_SetSpeed" :depends-on ("_package"))
    (:file "hand_Calibrate" :depends-on ("_package_hand_Calibrate"))
    (:file "_package_hand_Calibrate" :depends-on ("_package"))
    (:file "hand_WaitRest" :depends-on ("_package_hand_WaitRest"))
    (:file "_package_hand_WaitRest" :depends-on ("_package"))
    (:file "hand_SetAngle" :depends-on ("_package_hand_SetAngle"))
    (:file "_package_hand_SetAngle" :depends-on ("_package"))
    (:file "hand_GetEncoders" :depends-on ("_package_hand_GetEncoders"))
    (:file "_package_hand_GetEncoders" :depends-on ("_package"))
    (:file "hand_GetForces" :depends-on ("_package_hand_GetForces"))
    (:file "_package_hand_GetForces" :depends-on ("_package"))
  ))