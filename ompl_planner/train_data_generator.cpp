#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "mabi_msgs/PlanTrajectoryAction.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "train_data_generator");

    actionlib::SimpleActionClient<mabi_msgs::PlanTrajectoryAction> ac("plan_trajectory", true);
    ac.waitForServer();

    // Making sure that map is loaded
    ros::Duration(30).sleep();

    // Obtaining occupancy grid
    // todo

    //Setting goal
    mabi_msgs::PlanTrajectoryGoal goal;

    //Executing Plan
    ac.sendGoal(goal);

}
