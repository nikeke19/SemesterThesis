//
// Created by nick on 06/11/2020.
//

#include <ros/ros.h>
//#include "ompl_planner/OmplPlanner.h"
#include "ompl_planner/WriteOcGrid.h"

using namespace perceptive_mpc;

int main(int argc, char **argv) {
    ros::init(argc,argv, "write_oc_grid");
    ROS_INFO("Starting write_oc_grid node");
    ros::NodeHandle nodeHandle;
    WriteOcGrid writer(nodeHandle);

    ros::spin();
    return 0;
}