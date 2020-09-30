; Auto-generated. Do not edit!


(cl:in-package perceptive_mpc-msg)


;//! \htmlinclude WrenchPoseStamped.msg.html

(cl:defclass <WrenchPoseStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (wrench
    :reader wrench
    :initarg :wrench
    :type geometry_msgs-msg:Wrench
    :initform (cl:make-instance 'geometry_msgs-msg:Wrench))
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass WrenchPoseStamped (<WrenchPoseStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WrenchPoseStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WrenchPoseStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name perceptive_mpc-msg:<WrenchPoseStamped> is deprecated: use perceptive_mpc-msg:WrenchPoseStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <WrenchPoseStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader perceptive_mpc-msg:header-val is deprecated.  Use perceptive_mpc-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'wrench-val :lambda-list '(m))
(cl:defmethod wrench-val ((m <WrenchPoseStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader perceptive_mpc-msg:wrench-val is deprecated.  Use perceptive_mpc-msg:wrench instead.")
  (wrench m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <WrenchPoseStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader perceptive_mpc-msg:pose-val is deprecated.  Use perceptive_mpc-msg:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WrenchPoseStamped>) ostream)
  "Serializes a message object of type '<WrenchPoseStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'wrench) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WrenchPoseStamped>) istream)
  "Deserializes a message object of type '<WrenchPoseStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'wrench) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WrenchPoseStamped>)))
  "Returns string type for a message object of type '<WrenchPoseStamped>"
  "perceptive_mpc/WrenchPoseStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WrenchPoseStamped)))
  "Returns string type for a message object of type 'WrenchPoseStamped"
  "perceptive_mpc/WrenchPoseStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WrenchPoseStamped>)))
  "Returns md5sum for a message object of type '<WrenchPoseStamped>"
  "0494f16dc46a5eb2705de2f9317dfa03")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WrenchPoseStamped)))
  "Returns md5sum for a message object of type 'WrenchPoseStamped"
  "0494f16dc46a5eb2705de2f9317dfa03")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WrenchPoseStamped>)))
  "Returns full string definition for message of type '<WrenchPoseStamped>"
  (cl:format cl:nil "Header header~%geometry_msgs/Wrench wrench~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WrenchPoseStamped)))
  "Returns full string definition for message of type 'WrenchPoseStamped"
  (cl:format cl:nil "Header header~%geometry_msgs/Wrench wrench~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WrenchPoseStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'wrench))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WrenchPoseStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'WrenchPoseStamped
    (cl:cons ':header (header msg))
    (cl:cons ':wrench (wrench msg))
    (cl:cons ':pose (pose msg))
))
