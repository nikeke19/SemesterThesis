;; Auto-generated. Do not edit!


(when (boundp 'perceptive_mpc::WrenchPoseTrajectory)
  (if (not (find-package "PERCEPTIVE_MPC"))
    (make-package "PERCEPTIVE_MPC"))
  (shadow 'WrenchPoseTrajectory (find-package "PERCEPTIVE_MPC")))
(unless (find-package "PERCEPTIVE_MPC::WRENCHPOSETRAJECTORY")
  (make-package "PERCEPTIVE_MPC::WRENCHPOSETRAJECTORY"))

(in-package "ROS")
;;//! \htmlinclude WrenchPoseTrajectory.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass perceptive_mpc::WrenchPoseTrajectory
  :super ros::object
  :slots (_header _posesWrenches ))

(defmethod perceptive_mpc::WrenchPoseTrajectory
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:posesWrenches __posesWrenches) (let (r) (dotimes (i 0) (push (instance perceptive_mpc::WrenchPoseStamped :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _posesWrenches __posesWrenches)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:posesWrenches
   (&rest __posesWrenches)
   (if (keywordp (car __posesWrenches))
       (send* _posesWrenches __posesWrenches)
     (progn
       (if __posesWrenches (setq _posesWrenches (car __posesWrenches)))
       _posesWrenches)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; perceptive_mpc/WrenchPoseStamped[] _posesWrenches
    (apply #'+ (send-all _posesWrenches :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; perceptive_mpc/WrenchPoseStamped[] _posesWrenches
     (write-long (length _posesWrenches) s)
     (dolist (elem _posesWrenches)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; perceptive_mpc/WrenchPoseStamped[] _posesWrenches
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _posesWrenches (let (r) (dotimes (i n) (push (instance perceptive_mpc::WrenchPoseStamped :init) r)) r))
     (dolist (elem- _posesWrenches)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get perceptive_mpc::WrenchPoseTrajectory :md5sum-) "c310e1938cfabae9ab5df412a81f16ea")
(setf (get perceptive_mpc::WrenchPoseTrajectory :datatype-) "perceptive_mpc/WrenchPoseTrajectory")
(setf (get perceptive_mpc::WrenchPoseTrajectory :definition-)
      "Header header
perceptive_mpc/WrenchPoseStamped[] posesWrenches

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: perceptive_mpc/WrenchPoseStamped
Header header
geometry_msgs/Wrench wrench
geometry_msgs/Pose pose

================================================================================
MSG: geometry_msgs/Wrench
# This represents force in free space, separated into
# its linear and angular parts.
Vector3  force
Vector3  torque

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :perceptive_mpc/WrenchPoseTrajectory "c310e1938cfabae9ab5df412a81f16ea")


