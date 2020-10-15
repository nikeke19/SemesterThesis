#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "mabi_msgs/PlanTrajectoryAction.h"
#include "mabi_msgs/WriteOcGrid.h"
#include <random>

const double PI = 3.14156;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "train_data_generator");
    ROS_INFO("Started train data generator");
    ros::NodeHandle nh;
    ros::NodeHandle pNh("~");
    XmlRpc::XmlRpcValue trajectories;

    // Check if everything is in the right format
    if (pNh.hasParam("goal_points")) {
        pNh.getParam("goal_points", trajectories);
        if(trajectories.size()==0) {
            ROS_ERROR("No goal points in data_gen.yaml");
            return 0;
        }
        else {
            for(int i = 0; i < trajectories.size(); i++) {
                if(trajectories[i].size() != 3) {
                    ROS_ERROR_STREAM("Wrong amount of coordinates. 3 were expected and given are" << trajectories.size());
                    ROS_ERROR_STREAM("Check " << i <<"th line of your data_gen.yaml file");
                    return 0;
                }
            }
        }
    }
    else {
        ROS_ERROR("Could not find goal points in param");
        return 0;
    }

    actionlib::SimpleActionClient<mabi_msgs::PlanTrajectoryAction> ac("plan_trajectory", true);
    ac.waitForServer();

    // Making sure that map is loaded
    ros::Duration(60).sleep();
    ROS_INFO("Woke up from sleep");

    // Obtaining occupancy grid
    ros::ServiceClient client = nh.serviceClient<mabi_msgs::WriteOcGrid>("write_oc_grid_to_file");
    mabi_msgs::WriteOcGrid srv;
    srv.request.name = "world_1";
    srv.request.resolution = 0.2;
    if(client.call(srv))
        ROS_INFO("OC Grid created succesfully");
    else
        ROS_INFO("OC Grid not created, try again");

    // Default values
    mabi_msgs::PlanTrajectoryGoal goal;
    goal.current_state.joint_state.position.resize(6);
    goal.goal_pose.pose.orientation.w = -0.7;
    goal.goal_pose.pose.orientation.x = 0.7;
    goal.goal_pose.pose.orientation.y = 0.0;
    goal.goal_pose.pose.orientation.z = 0.0;
    int n_random_start_configurations = 10;

    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution_x(-2.0, 2.0);
    std::uniform_real_distribution<double> distribution_y(-2.0, 2.0);
    std::uniform_real_distribution<double> distribution_yaw(-PI, PI);
    std::uniform_real_distribution<double> distribution_q(-PI, PI);

    int n_trajectories = trajectories.size();
    for(int i = 0; i < n_trajectories; i++) {
        //Setting goal
        goal.goal_pose.pose.position.x = trajectories[i][0];
        goal.goal_pose.pose.position.y = trajectories[i][1];
        goal.goal_pose.pose.position.z = trajectories[i][2];

        // Setting n_random_starts positions and planning to goal i
        for(int k = 0, count = 0; k < n_random_start_configurations && count < n_random_start_configurations * 3; count ++) {
            std::stringstream file_name_stream;
            file_name_stream << "world_" << std::to_string(1) << "_goal_" << std::to_string(i) << "_start_" << std::to_string(k);
            goal.file_name = file_name_stream.str();
            goal.current_state.pose_base.x = distribution_x(generator);
            goal.current_state.pose_base.y = distribution_y(generator);
            goal.current_state.pose_base.theta = distribution_yaw(generator);
            for(int j = 0; j < 6; j++) {
                goal.current_state.joint_state.position[j] = distribution_q(generator);
            }
            ac.sendGoal(goal);
            ROS_INFO("Sended goal, waiting for result");
            ac.waitForResult();
            ROS_INFO("Got result");
            if(ac.getResult()->trajectory_found) {
                k++;
                ROS_INFO("Trajectory created");
            }
            else {
                ROS_INFO("Trajectory not created");
            }


        }

    }

    //Setting goal
    goal.goal_pose.pose.position.x = 0.94;
    goal.goal_pose.pose.position.y = -0.45;
    goal.goal_pose.pose.position.z = 0.84;
    goal.goal_pose.pose.orientation.w = -0.7;
    goal.goal_pose.pose.orientation.x = 0.7;
    goal.goal_pose.pose.orientation.y = 0.0;
    goal.goal_pose.pose.orientation.z = 0.0;

    goal.file_name = "test_from_external";

    for(int i = 0; i < 6; i++)
        goal.current_state.joint_state.position[i] = 0;

    goal.current_state.pose_base.x = -1;
    goal.current_state.pose_base.y = 0;
    goal.current_state.pose_base.theta = 0.4;

    //Executing Plan
    ac.sendGoal(goal);
    ROS_INFO("Finished data writting");

}
