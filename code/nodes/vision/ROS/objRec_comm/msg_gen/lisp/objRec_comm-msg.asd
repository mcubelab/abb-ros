
(cl:in-package :asdf)

(defsystem "objRec_comm-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "objRec_ObjPos" :depends-on ("_package_objRec_ObjPos"))
    (:file "_package_objRec_ObjPos" :depends-on ("_package"))
  ))