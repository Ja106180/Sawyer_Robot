
(cl:in-package :asdf)

(defsystem "arm_follow-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "PersonKeypoints" :depends-on ("_package_PersonKeypoints"))
    (:file "_package_PersonKeypoints" :depends-on ("_package"))
    (:file "PersonsKeypoints" :depends-on ("_package_PersonsKeypoints"))
    (:file "_package_PersonsKeypoints" :depends-on ("_package"))
  ))