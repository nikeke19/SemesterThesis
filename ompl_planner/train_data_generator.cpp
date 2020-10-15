#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "mabi_msgs/PlanTrajectoryAction.h"
#include "mabi_msgs/WriteOcGrid.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "train_data_generator");
    ROS_INFO("Started train data generator");
    ros::NodeHandle nh;
    actionlib::SimpleActionClient<mabi_msgs::PlanTrajectoryAction> ac("plan_trajectory", true);
    ac.waitForServer();

    // Making sure that map is loaded
    ros::Duration(60).sleep();
    ROS_INFO("Woke up from sleep");

    // Obtaining occupancy grid
    ros::ServiceClient client = nh.serviceClient<mabi_msgs::WriteOcGrid>("write_oc_grid_to_file");
    mabi_msgs::WriteOcGrid srv;
    srv.request.name = "test_from_external";
    srv.request.resolution = 0.2;
    if(client.call(srv))
        ROS_INFO("OC Grid created succesfully");
    else
        ROS_INFO("OC Grid not created, try again");

    //Setting goal
    mabi_msgs::PlanTrajectoryGoal goal;
    goal.goal_pose.pose.position.x = 0.94;
    goal.goal_pose.pose.position.y = -0.45;
    goal.goal_pose.pose.position.z = 0.84;
    goal.goal_pose.pose.orientation.w = -0.7;
    goal.goal_pose.pose.orientation.x = 0.7;
    goal.goal_pose.pose.orientation.y = 0.0;
    goal.goal_pose.pose.orientation.z = 0.0;

    goal.file_name = "test_from_external";
    goal.current_state.joint_state.position.resize(6);
    for(int i = 0; i < 6; i++)
        goal.current_state.joint_state.position[i] = 0;

    goal.current_state.pose_base.x = -1;
    goal.current_state.pose_base.y = 0;
    goal.current_state.pose_base.theta = 0.4;

    //Executing Plan
    ac.sendGoal(goal);
    ROS_INFO("Finished data writting");

}
