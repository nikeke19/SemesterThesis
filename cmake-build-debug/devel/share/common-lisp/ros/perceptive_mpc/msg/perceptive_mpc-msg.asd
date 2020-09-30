
(cl:in-package :asdf)

(defsystem "perceptive_mpc-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "WrenchPoseStamped" :depends-on ("_package_WrenchPoseStamped"))
    (:file "_package_WrenchPoseStamped" :depends-on ("_package"))
    (:file "WrenchPoseTrajectory" :depends-on ("_package_WrenchPoseTrajectory"))
    (:file "_package_WrenchPoseTrajectory" :depends-on ("_package"))
  ))