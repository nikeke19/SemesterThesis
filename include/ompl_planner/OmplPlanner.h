//
// Created by nick on 29/09/2020.
//

#ifndef PERCEPTIVE_MPC_OMPLPLANNER_H
#define PERCEPTIVE_MPC_OMPLPLANNER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h> // For input from interactive

#include <Eigen/Dense>
#include "kindr/poses/HomogeneousTransformation.hpp"

#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/Goal.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>

#include "ompl_planner/EndEffectorGoal.h"
#include "StateSpace.h"
#include "perceptive_mpc/Definitions.h"

#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h>

const float PI = 3.14159265359;

namespace perceptive_mpc{

struct Settings {
    Settings() {
        minArmPositionLimits << -5.23599, -3.14159, -2.47837, -5.23599, -2.04204, -3.14159;
        maxArmPositionLimits << 5.23599, 3.14159, 2.47837, 5.23599, 2.04204, 3.14159;
        velocityLimits << 1.0, 0.5, 0.5, 0.5, 0.5, 0.7, 1.0, 1.0;
    }
    Eigen::Vector2d minBasePositionLimit{-2.5, -2.5};
    Eigen::Vector2d maxBasePositionLimit{2.5, 2.5};

    Eigen::Matrix<double, 6, 1> minArmPositionLimits;
    Eigen::Matrix<double, 6, 1> maxArmPositionLimits;
    Eigen::Matrix<double, Definitions::INPUT_DIM_, 1> velocityLimits;

    Eigen::Matrix4d transformBase_X_ArmMount = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d transformWrist2_X_Endeffector = Eigen::Matrix4d::Identity();

    double maxPlanningTime = 10;
    double positionTolerance = 0.1;
    double orientationTolerance = 0.1;

//    std::shared_ptr<VoxbloxCostConfig> voxbloxCostConfig = nullptr; //todo make this active?
};

class OmplPlanner {
public:
    explicit OmplPlanner(const ros::NodeHandle& nodeHandle);
    OmplPlanner() = default;
    ~OmplPlanner()= default;

    ompl::base::GoalPtr convertPoseToOmplGoal(const kindr::HomTransformQuatD& goal_pose);
    void planTrajectory(const Eigen::Matrix<float, Definitions::STATE_DIM_, 1>& start_position, const kindr::HomTransformQuatD& goal_pose);

    struct CurrentState {
        Eigen::Vector2d position2DBase;
        double yaw_base;
        Eigen::Matrix<double,6,1> jointAngles;
    };
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_desired_end_effector_pose_subscriber_;
    ros::Publisher pub_arm_state_;

    Settings settings_;
    CurrentState current_state_;

    void initializeState();
    void cbDesiredEndEffectorPose(const geometry_msgs::PoseStampedConstPtr& msgPtr);
    void publishArmState();

};

}
#endif //PERCEPTIVE_MPC_OMPLPLANNER_H
