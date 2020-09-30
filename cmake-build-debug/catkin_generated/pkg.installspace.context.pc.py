# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include".split(';') if "${prefix}/include;/usr/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roslib;tf;geometry_msgs;ocs2_comm_interfaces;ocs2_core;ocs2_robotic_tools;ocs2_ddp;kindr;kindr_ros;voxblox;voxblox_ros;message_runtime".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lperceptive_mpc;/usr/lib/x86_64-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so".split(';') if "-lperceptive_mpc;/usr/lib/x86_64-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so" != "" else []
PROJECT_NAME = "perceptive_mpc"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
