
(cl:in-package :asdf)

(defsystem "logger_comm-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "logger_Copy" :depends-on ("_package_logger_Copy"))
    (:file "_package_logger_Copy" :depends-on ("_package"))
    (:file "logger_Append" :depends-on ("_package_logger_Append"))
    (:file "_package_logger_Append" :depends-on ("_package"))
    (:file "logger_Create" :depends-on ("_package_logger_Create"))
    (:file "_package_logger_Create" :depends-on ("_package"))
    (:file "logger_Start" :depends-on ("_package_logger_Start"))
    (:file "_package_logger_Start" :depends-on ("_package"))
    (:file "logger_Stop" :depends-on ("_package_logger_Stop"))
    (:file "_package_logger_Stop" :depends-on ("_package"))
  ))