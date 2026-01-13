
(cl:in-package :asdf)

(defsystem "my_car_yolo-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ImuProcessed" :depends-on ("_package_ImuProcessed"))
    (:file "_package_ImuProcessed" :depends-on ("_package"))
    (:file "ObjectDetection" :depends-on ("_package_ObjectDetection"))
    (:file "_package_ObjectDetection" :depends-on ("_package"))
    (:file "ObjectDetections" :depends-on ("_package_ObjectDetections"))
    (:file "_package_ObjectDetections" :depends-on ("_package"))
    (:file "WheelEncoders" :depends-on ("_package_WheelEncoders"))
    (:file "_package_WheelEncoders" :depends-on ("_package"))
  ))