; Auto-generated. Do not edit!


(cl:in-package perceptive_mpc-msg)


;//! \htmlinclude WrenchPoseTrajectory.msg.html

(cl:defclass <WrenchPoseTrajectory> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (posesWrenches
    :reader posesWrenches
    :initarg :posesWrenches
    :type (cl:vector perceptive_mpc-msg:WrenchPoseStamped)
   :initform (cl:make-array 0 :element-type 'perceptive_mpc-msg:WrenchPoseStamped :initial-element (cl:make-instance 'perceptive_mpc-msg:WrenchPoseStamped))))
)

(cl:defclass WrenchPoseTrajectory (<WrenchPoseTrajectory>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WrenchPoseTrajectory>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WrenchPoseTrajectory)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name perceptive_mpc-msg:<WrenchPoseTrajectory> is deprecated: use perceptive_mpc-msg:WrenchPoseTrajectory instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <WrenchPoseTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader perceptive_mpc-msg:header-val is deprecated.  Use perceptive_mpc-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'posesWrenches-val :lambda-list '(m))
(cl:defmethod posesWrenches-val ((m <WrenchPoseTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader perceptive_mpc-msg:posesWrenches-val is deprecated.  Use perceptive_mpc-msg:posesWrenches instead.")
  (posesWrenches m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WrenchPoseTrajectory>) ostream)
  "Serializes a message object of type '<WrenchPoseTrajectory>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'posesWrenches))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'posesWrenches))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WrenchPoseTrajectory>) istream)
  "Deserializes a message object of type '<WrenchPoseTrajectory>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'posesWrenches) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'posesWrenches)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'perceptive_mpc-msg:WrenchPoseStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WrenchPoseTrajectory>)))
  "Returns string type for a message object of type '<WrenchPoseTrajectory>"
  "perceptive_mpc/WrenchPoseTrajectory")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WrenchPoseTrajectory)))
  "Returns string type for a message object of type 'WrenchPoseTrajectory"
  "perceptive_mpc/WrenchPoseTrajectory")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WrenchPoseTrajectory>)))
  "Returns md5sum for a message object of type '<WrenchPoseTrajectory>"
  "c310e1938cfabae9ab5df412a81f16ea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WrenchPoseTrajectory)))
  "Returns md5sum for a message object of type 'WrenchPoseTrajectory"
  "c310e1938cfabae9ab5df412a81f16ea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WrenchPoseTrajectory>)))
  "Returns full string definition for message of type '<WrenchPoseTrajectory>"
  (cl:format cl:nil "Header header~%perceptive_mpc/WrenchPoseStamped[] posesWrenches~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: perceptive_mpc/WrenchPoseStamped~%Header header~%geometry_msgs/Wrench wrench~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WrenchPoseTrajectory)))
  "Returns full string definition for message of type 'WrenchPoseTrajectory"
  (cl:format cl:nil "Header header~%perceptive_mpc/WrenchPoseStamped[] posesWrenches~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: perceptive_mpc/WrenchPoseStamped~%Header header~%geometry_msgs/Wrench wrench~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WrenchPoseTrajectory>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'posesWrenches) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WrenchPoseTrajectory>))
  "Converts a ROS message object to a list"
  (cl:list 'WrenchPoseTrajectory
    (cl:cons ':header (header msg))
    (cl:cons ':posesWrenches (posesWrenches msg))
))
