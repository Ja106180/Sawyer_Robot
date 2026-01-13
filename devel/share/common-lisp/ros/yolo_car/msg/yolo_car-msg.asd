
(cl:in-package :asdf)

(defsystem "yolo_car-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ObjectDetection" :depends-on ("_package_ObjectDetection"))
    (:file "_package_ObjectDetection" :depends-on ("_package"))
    (:file "ObjectDetections" :depends-on ("_package_ObjectDetections"))
    (:file "_package_ObjectDetections" :depends-on ("_package"))
  ))