
(cl:in-package :asdf)

(defsystem "logger_comm-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "logger_SystemLog" :depends-on ("_package_logger_SystemLog"))
    (:file "_package_logger_SystemLog" :depends-on ("_package"))
  ))