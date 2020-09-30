//
// Created by nick on 29/09/2020.
//

#include <ros/ros.h>
#include "ompl_planner/OmplPlanner.h"

using namespace perceptive_mpc;

int main(int argc, char **argv) {
    ros::init(argc,argv, "ompl_planner");
    ROS_INFO("Print hi");
    ros::NodeHandle nodeHandle;
    OmplPlanner planner(nodeHandle);

    ros::spin();
    return 0;
}