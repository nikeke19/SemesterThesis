// Auto-generated. Do not edit!

// (in-package perceptive_mpc.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let WrenchPoseStamped = require('./WrenchPoseStamped.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class WrenchPoseTrajectory {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.posesWrenches = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('posesWrenches')) {
        this.posesWrenches = initObj.posesWrenches
      }
      else {
        this.posesWrenches = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WrenchPoseTrajectory
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [posesWrenches]
    // Serialize the length for message field [posesWrenches]
    bufferOffset = _serializer.uint32(obj.posesWrenches.length, buffer, bufferOffset);
    obj.posesWrenches.forEach((val) => {
      bufferOffset = WrenchPoseStamped.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WrenchPoseTrajectory
    let len;
    let data = new WrenchPoseTrajectory(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [posesWrenches]
    // Deserialize array length for message field [posesWrenches]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.posesWrenches = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.posesWrenches[i] = WrenchPoseStamped.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.posesWrenches.forEach((val) => {
      length += WrenchPoseStamped.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'perceptive_mpc/WrenchPoseTrajectory';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c310e1938cfabae9ab5df412a81f16ea';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WrenchPoseTrajectory(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.posesWrenches !== undefined) {
      resolved.posesWrenches = new Array(msg.posesWrenches.length);
      for (let i = 0; i < resolved.posesWrenches.length; ++i) {
        resolved.posesWrenches[i] = WrenchPoseStamped.Resolve(msg.posesWrenches[i]);
      }
    }
    else {
      resolved.posesWrenches = []
    }

    return resolved;
    }
};

module.exports = WrenchPoseTrajectory;
