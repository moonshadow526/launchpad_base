
(cl:in-package :asdf)

(defsystem "bucketbot_driver-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Dockir" :depends-on ("_package_Dockir"))
    (:file "_package_Dockir" :depends-on ("_package"))
    (:file "CliffEvent" :depends-on ("_package_CliffEvent"))
    (:file "_package_CliffEvent" :depends-on ("_package"))
  ))