
(cl:in-package :asdf)

(defsystem "dynamic-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Cap" :depends-on ("_package_Cap"))
    (:file "_package_Cap" :depends-on ("_package"))
  ))