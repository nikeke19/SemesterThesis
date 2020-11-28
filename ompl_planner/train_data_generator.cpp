#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "mabi_msgs/PlanTrajectoryAction.h"
#include "mabi_msgs/WriteOcGrid.h"
#include <random>

const double PI = 3.14156;
const int SUCCESFULL = 1;
const int UNSUCCESFUL = 0;
const int NO_VALID_START = -1;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "train_data_generator");
    ROS_INFO("Started train data generator");
    ros::NodeHandle nh;
    ros::NodeHandle pNh("~");
    XmlRpc::XmlRpcValue goalPoints;
    std::string world;
    pNh.getParam("world", world);
    ROS_INFO_STREAM("world is " << world);

    // Check if everything is in the right format
    if (pNh.hasParam("goal_points")) {
        pNh.getParam("goal_points", goalPoints);
        if(goalPoints.size() == 0) {
            ROS_ERROR("No goal points in data_gen.yaml");
            return 0;
        }
        else {
            for(int i = 0; i < goalPoints.size(); i++) {
                if(goalPoints[i].size() != 3) {
                    ROS_ERROR_STREAM("Wrong amount of coordinates. 3 were expected and given are" << goalPoints[i].size());
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
    //ros::Duration(30).sleep();
    ROS_INFO("Woke up from sleep");

    // Obtaining occupancy grid
//    ros::ServiceClient client = nh.serviceClient<mabi_msgs::WriteOcGrid>("write_oc_grid_to_file");
//    mabi_msgs::WriteOcGrid srv;
//    srv.request.name = world;
//    srv.request.resolution = 0.2;
//    if(client.call(srv))
//        ROS_INFO("OC Grid created succesfully");
//    else
//        ROS_INFO("OC Grid not created, try again");

    // Default values
    mabi_msgs::PlanTrajectoryGoal goal;
    goal.current_state.joint_state.position.resize(6);
    goal.goal_pose.pose.orientation.w = -0.7;
    goal.goal_pose.pose.orientation.x = 0.7;
    goal.goal_pose.pose.orientation.y = 0.0;
    goal.goal_pose.pose.orientation.z = 0.0;
    int n_random_start_configurations = 20;

    std::default_random_engine generator;
    std::normal_distribution<double> distribution_q1(0, PI/3);
    std::normal_distribution<double> distribution_q2(-PI/2, 0.356);
    std::normal_distribution<double> distribution_q3_1(PI/2, 0.2);
    std::normal_distribution<double> distribution_q3_2(-PI/2, 0.2);
    std::normal_distribution<double> distribution_q4(0, PI/3);
    std::normal_distribution<double> distribution_q5(0, 0.66);
    std::normal_distribution<double> distribution_q6(0, PI/3);
    std::normal_distribution<double> distribution_position(0, 0.1);
    std::uniform_real_distribution<double> distribution_x(-5, 5.0); // only for world 1, else -5,5
    std::uniform_real_distribution<double> distribution_y(-5.0, 5.0);
    std::uniform_real_distribution<double> distribution_yaw(-PI, PI);

    int n_trajectories = goalPoints.size();
    for(int i = 0; i < n_trajectories; i++) { //todo was zero but changed to 13
        //Setting goal
        goal.goal_pose.pose.position.x = goalPoints[i][0];
        goal.goal_pose.pose.position.y = goalPoints[i][1];
        goal.goal_pose.pose.position.z = goalPoints[i][2];

        // Setting n_random_starts positions and planning to goal i
        for(int k = 0, count = 0; k < n_random_start_configurations && count < n_random_start_configurations * 2; count ++) {
            std::stringstream file_name_stream;
            file_name_stream << world  << "_goal_" << std::to_string(i) << "_start_" << std::to_string(k);
            goal.file_name = file_name_stream.str();
            goal.current_state.pose_base.x = distribution_x(generator);
            goal.current_state.pose_base.y = distribution_y(generator);
            goal.current_state.pose_base.theta = distribution_yaw(generator);

            goal.current_state.joint_state.position[0] = distribution_q1(generator);
            goal.current_state.joint_state.position[1] = distribution_q2(generator);
            // Flip coin if use gaussian centered around +PI/2 or -PI/2
            if(rand() % 2 + 1 == 1)
                goal.current_state.joint_state.position[2] = distribution_q3_1(generator);
            else
                goal.current_state.joint_state.position[2] = distribution_q3_2(generator);
            goal.current_state.joint_state.position[3] = distribution_q4(generator);
            goal.current_state.joint_state.position[4] = distribution_q5(generator);
            goal.current_state.joint_state.position[5] = distribution_q6(generator);

            ac.sendGoal(goal);
            ac.waitForResult();
            if(ac.getResult()->trajectory_found == SUCCESFULL) {
                k++;
                ROS_INFO("Trajectory created");
            }
            else if(ac.getResult()->trajectory_found == NO_VALID_START) {
                ROS_WARN("No valid start");
                count --;
            }
            else {
                ROS_WARN("Trajectory not created");
            }
        }

    }

    ROS_INFO("Finished data writting");

}
