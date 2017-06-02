
(cl:in-package :asdf)

(defsystem "python_service_test-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PanoService" :depends-on ("_package_PanoService"))
    (:file "_package_PanoService" :depends-on ("_package"))
  ))