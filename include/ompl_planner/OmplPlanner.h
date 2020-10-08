//
// Created by nick on 29/09/2020.
//

#ifndef PERCEPTIVE_MPC_OMPLPLANNER_H
#define PERCEPTIVE_MPC_OMPLPLANNER_H

// ROS stuff
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h> // For input from interactive
#include <Eigen/Dense>
#include "kindr/poses/HomogeneousTransformation.hpp"
#include <kindr_ros/kindr_ros.hpp>

// Custom Classes
#include "EndEffectorGoal.h"
#include "StateSpace.h"
#include "perceptive_mpc/Definitions.h"
#include "perceptive_mpc/costs/PointsOnRobot.h"  //See if it helps
#include "VoxbloxStateValidityChecker.h"
#include "ocs2_core/Dimensions.h"
#include "perceptive_mpc/EsdfCachingServer.hpp"

//OMPL Setup
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/Goal.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>

// OMPL Planners
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/informedtrees/ABITstar.h>

// test if it helps:
#include <ocs2_core/automatic_differentiation/CppAdInterface.h>

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

    double maxPlanningTime = 30;
    double positionTolerance = 0.3; //todo was 0.1
    double orientationTolerance = 100;

    std::shared_ptr<VoxbloxCostConfig> voxbloxCostConfig = nullptr;
};



class OmplPlanner {
public:
    using DIMENSIONS = ocs2::Dimensions<Definitions::STATE_DIM_, Definitions::INPUT_DIM_>;
    using state_vector_t = typename DIMENSIONS::state_vector_t;
    using input_vector_t = typename DIMENSIONS::input_vector_t;

    explicit OmplPlanner(const ros::NodeHandle& nodeHandle);
    OmplPlanner() = default;
    ~OmplPlanner()= default;

    ompl::base::GoalPtr convertPoseToOmplGoal(const kindr::HomTransformQuatD& goal_pose);
    void planTrajectory(const kindr::HomTransformQuatD& goal_pose);

    struct CurrentState {
        Eigen::Vector2d position2DBase;
        double yaw_base;
        Eigen::Matrix<double,6,1> jointAngles;
    };

    struct PlannerOutput {
        std::vector<double> times;
        std::vector<state_vector_t> states;
        std::vector<input_vector_t> inputs;
    };
private:
    ros::NodeHandle nh_;
    ros::Subscriber subDesiredEndEffectorPoseSubscriber_;

    // To publish to RVIZ Simulation
    tf::TransformBroadcaster tfOdomBroadcaster_;
    geometry_msgs::TransformStamped odomTrans_;
    sensor_msgs::JointState jointState_;
    ros::Publisher pubArmState_;
    ros::Rate r_;

    Settings settings_;
    CurrentState currentState_;
    KinematicInterfaceConfig kinematicInterfaceConfig_;

    // Solution to planning problem
    PlannerOutput solutionTrajectory_;
    std::vector<CurrentState> test;

    void initializeState();
    void cbDesiredEndEffectorPose(const geometry_msgs::PoseStampedConstPtr& msgPtr);
    void publishSolutionTrajectory(const std::vector<CurrentState>& solutionTrajectory);
    std::shared_ptr<VoxbloxCostConfig> setUpVoxbloxCostConfig();
    void testLoop();
    void initializeKinematicInterfaceConfig();


};

}
#endif //PERCEPTIVE_MPC_OMPLPLANNER_H
