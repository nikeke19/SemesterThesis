# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "perceptive_mpc: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iperceptive_mpc:/home/nick/mpc_ws/src/perceptive_mpc/msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(perceptive_mpc_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/nick/mpc_ws/src/perceptive_mpc/msg/WrenchPoseStamped.msg" NAME_WE)
add_custom_target(_perceptive_mpc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "perceptive_mpc" "/home/nick/mpc_ws/src/perceptive_mpc/msg/WrenchPoseStamped.msg" "geometry_msgs/Vector3:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Point:geometry_msgs/Wrench:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/nick/mpc_ws/src/perceptive_mpc/msg/WrenchPoseTrajectory.msg" NAME_WE)
add_custom_target(_perceptive_mpc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "perceptive_mpc" "/home/nick/mpc_ws/src/perceptive_mpc/msg/WrenchPoseTrajectory.msg" "geometry_msgs/Vector3:perceptive_mpc/WrenchPoseStamped:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Wrench:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(perceptive_mpc
  "/home/nick/mpc_ws/src/perceptive_mpc/msg/WrenchPoseTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/nick/mpc_ws/src/perceptive_mpc/msg/WrenchPoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/perceptive_mpc
)
_generate_msg_cpp(perceptive_mpc
  "/home/nick/mpc_ws/src/perceptive_mpc/msg/WrenchPoseStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/perceptive_mpc
)

### Generating Services

### Generating Module File
_generate_module_cpp(perceptive_mpc
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/perceptive_mpc
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(perceptive_mpc_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(perceptive_mpc_generate_messages perceptive_mpc_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nick/mpc_ws/src/perceptive_mpc/msg/WrenchPoseStamped.msg" NAME_WE)
add_dependencies(perceptive_mpc_generate_messages_cpp _perceptive_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nick/mpc_ws/src/perceptive_mpc/msg/WrenchPoseTrajectory.msg" NAME_WE)
add_dependencies(perceptive_mpc_generate_messages_cpp _perceptive_mpc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(perceptive_mpc_gencpp)
add_dependencies(perceptive_mpc_gencpp perceptive_mpc_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS perceptive_mpc_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(perceptive_mpc
  "/home/nick/mpc_ws/src/perceptive_mpc/msg/WrenchPoseTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/nick/mpc_ws/src/perceptive_mpc/msg/WrenchPoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/perceptive_mpc
)
_generate_msg_eus(perceptive_mpc
  "/home/nick/mpc_ws/src/perceptive_mpc/msg/WrenchPoseStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/perceptive_mpc
)

### Generating Services

### Generating Module File
_generate_module_eus(perceptive_mpc
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/perceptive_mpc
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(perceptive_mpc_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(perceptive_mpc_generate_messages perceptive_mpc_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nick/mpc_ws/src/perceptive_mpc/msg/WrenchPoseStamped.msg" NAME_WE)
add_dependencies(perceptive_mpc_generate_messages_eus _perceptive_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nick/mpc_ws/src/perceptive_mpc/msg/WrenchPoseTrajectory.msg" NAME_WE)
add_dependencies(perceptive_mpc_generate_messages_eus _perceptive_mpc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(perceptive_mpc_geneus)
add_dependencies(perceptive_mpc_geneus perceptive_mpc_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS perceptive_mpc_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(perceptive_mpc
  "/home/nick/mpc_ws/src/perceptive_mpc/msg/WrenchPoseTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/nick/mpc_ws/src/perceptive_mpc/msg/WrenchPoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/perceptive_mpc
)
_generate_msg_lisp(perceptive_mpc
  "/home/nick/mpc_ws/src/perceptive_mpc/msg/WrenchPoseStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/perceptive_mpc
)

### Generating Services

### Generating Module File
_generate_module_lisp(perceptive_mpc
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/perceptive_mpc
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(perceptive_mpc_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(perceptive_mpc_generate_messages perceptive_mpc_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nick/mpc_ws/src/perceptive_mpc/msg/WrenchPoseStamped.msg" NAME_WE)
add_dependencies(perceptive_mpc_generate_messages_lisp _perceptive_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nick/mpc_ws/src/perceptive_mpc/msg/WrenchPoseTrajectory.msg" NAME_WE)
add_dependencies(perceptive_mpc_generate_messages_lisp _perceptive_mpc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(perceptive_mpc_genlisp)
add_dependencies(perceptive_mpc_genlisp perceptive_mpc_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS perceptive_mpc_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(perceptive_mpc
  "/home/nick/mpc_ws/src/perceptive_mpc/msg/WrenchPoseTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/nick/mpc_ws/src/perceptive_mpc/msg/WrenchPoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/perceptive_mpc
)
_generate_msg_nodejs(perceptive_mpc
  "/home/nick/mpc_ws/src/perceptive_mpc/msg/WrenchPoseStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/perceptive_mpc
)

### Generating Services

### Generating Module File
_generate_module_nodejs(perceptive_mpc
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/perceptive_mpc
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(perceptive_mpc_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(perceptive_mpc_generate_messages perceptive_mpc_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nick/mpc_ws/src/perceptive_mpc/msg/WrenchPoseStamped.msg" NAME_WE)
add_dependencies(perceptive_mpc_generate_messages_nodejs _perceptive_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nick/mpc_ws/src/perceptive_mpc/msg/WrenchPoseTrajectory.msg" NAME_WE)
add_dependencies(perceptive_mpc_generate_messages_nodejs _perceptive_mpc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(perceptive_mpc_gennodejs)
add_dependencies(perceptive_mpc_gennodejs perceptive_mpc_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS perceptive_mpc_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(perceptive_mpc
  "/home/nick/mpc_ws/src/perceptive_mpc/msg/WrenchPoseTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/nick/mpc_ws/src/perceptive_mpc/msg/WrenchPoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/perceptive_mpc
)
_generate_msg_py(perceptive_mpc
  "/home/nick/mpc_ws/src/perceptive_mpc/msg/WrenchPoseStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/perceptive_mpc
)

### Generating Services

### Generating Module File
_generate_module_py(perceptive_mpc
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/perceptive_mpc
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(perceptive_mpc_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(perceptive_mpc_generate_messages perceptive_mpc_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nick/mpc_ws/src/perceptive_mpc/msg/WrenchPoseStamped.msg" NAME_WE)
add_dependencies(perceptive_mpc_generate_messages_py _perceptive_mpc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nick/mpc_ws/src/perceptive_mpc/msg/WrenchPoseTrajectory.msg" NAME_WE)
add_dependencies(perceptive_mpc_generate_messages_py _perceptive_mpc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(perceptive_mpc_genpy)
add_dependencies(perceptive_mpc_genpy perceptive_mpc_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS perceptive_mpc_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/perceptive_mpc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/perceptive_mpc
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(perceptive_mpc_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/perceptive_mpc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/perceptive_mpc
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(perceptive_mpc_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/perceptive_mpc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/perceptive_mpc
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(perceptive_mpc_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/perceptive_mpc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/perceptive_mpc
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(perceptive_mpc_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/perceptive_mpc)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/perceptive_mpc\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/perceptive_mpc
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(perceptive_mpc_generate_messages_py geometry_msgs_generate_messages_py)
endif()
