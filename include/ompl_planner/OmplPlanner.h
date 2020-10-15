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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "mabi_msgs/PlanTrajectoryAction.h"
#include "mabi_msgs/WriteOcGrid.h"
#include <actionlib/server/simple_action_server.h>


// Custom Classes
#include "EndEffectorGoal.h"
#include "StateSpace.h"
#include "perceptive_mpc/Definitions.h"
#include "perceptive_mpc/costs/PointsOnRobot.h"  //See if it helps
#include "VoxbloxStateValidityChecker.h"
#include "ocs2_core/Dimensions.h"
#include "perceptive_mpc/EsdfCachingServer.hpp"
#include "voxblox_msgs/FilePath.h"

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

    Eigen::Vector2d minBasePositionLimit{-5.0, -5.0};
    Eigen::Vector2d maxBasePositionLimit{5.0, 5.0};
    Eigen::Vector2d minMaxHeight{0.0, 2.5};

    Eigen::Matrix<double, 6, 1> minArmPositionLimits;
    Eigen::Matrix<double, 6, 1> maxArmPositionLimits;
    Eigen::Matrix<double, Definitions::INPUT_DIM_, 1> velocityLimits;

    Eigen::Matrix4d transformBase_X_ArmMount = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d transformWrist2_X_Endeffector = Eigen::Matrix4d::Identity();

    double maxPlanningTime = 20;
    double positionTolerance = 0.3;
    double orientationTolerance = 100;

    std::shared_ptr<VoxbloxCostConfig> voxbloxCostConfig = nullptr;
};

struct CurrentState {
    Eigen::Vector2d position2DBase;
    double yaw_base;
    Eigen::Matrix<double,6,1> jointAngles;
};


class OmplPlanner {
public:
    using DIMENSIONS = ocs2::Dimensions<Definitions::STATE_DIM_, Definitions::INPUT_DIM_>;
    using state_vector_t = typename DIMENSIONS::state_vector_t;
    using input_vector_t = typename DIMENSIONS::input_vector_t;
    typedef ocs2::SystemObservation<perceptive_mpc::STATE_DIM_, perceptive_mpc::INPUT_DIM_> Observation;


    explicit OmplPlanner(const ros::NodeHandle& nodeHandle);
    OmplPlanner() = default;
    ~OmplPlanner()= default;

private:
    ros::NodeHandle nh_;


    // To publish to RVIZ Simulation
    tf::TransformBroadcaster tfOdomBroadcaster_;
    geometry_msgs::TransformStamped odomTrans_;
    sensor_msgs::JointState jointState_;
    ros::Publisher pubArmState_;
    ros::Publisher pubPointsOnRobot_;
    ros::Rate r_;
    std::shared_ptr<PointsOnRobot> pointsOnRobot_;

    // To initialize
    Settings settings_;
    CurrentState currentState_;
    KinematicInterfaceConfig kinematicInterfaceConfig_;
    std::shared_ptr<voxblox::EsdfCachingServer> esdfCachingServer_;
    ros::ServiceClient serviceLoadMap_;
    std::shared_ptr<KinematicsInterfaceAD> kinematicsInterface_;

    // Solution to planning problem
    bool writeSolutionTrajectoryToFile_ = false;
    bool writeConditioningToFile_ = false;
    bool writeOccupancyGridToFile_ = true;

    // For external programm requesting Trajectory
    ros::Subscriber subDesiredEndEffectorPoseSubscriber_;
    actionlib::SimpleActionServer<mabi_msgs::PlanTrajectoryAction> asPlanTrajectory_;
    mabi_msgs::PlanTrajectoryResult planTrajectoryResult_;
    ros::ServiceServer srvWriteOccupancyGridToFile_;


    // Mutex
    boost::shared_mutex observationMutex_;


   //Initialize
    void initializeState();
    void initializeKinematicInterfaceConfig();
    void testLoop();
    void loadMap();
    void setUpVoxbloxCostConfig();

    bool planTrajectory(const kindr::HomTransformQuatD &goal_pose, std::string name);
    ompl::base::GoalPtr convertPoseToOmplGoal(const kindr::HomTransformQuatD& goal_pose);

    // Functions to save Data
    void writeTrajectoryToFile(const std::vector<CurrentState>& trajectory, const std::string& name);
    void writeConditioningToFile(const MabiStateSpace::StateType* goalState, CurrentState startState ,std::string name);
    void writeOccupancyGridToFile(float resolution, std::string name);

    //Visualization
    void publishSolutionTrajectory(const std::vector<CurrentState>& solutionTrajectory);
    void visualizeCollisionPoints(const geometry_msgs::TransformStamped& base, sensor_msgs::JointState joints);
    void visualizeOccupancyGrid(std::vector<Eigen::Matrix<float, 3,1>> collisionPoints);

    // For external programms requesting Trajectory
    void cbDesiredEndEffectorPose(const geometry_msgs::PoseStampedConstPtr& msgPtr);
    void cbPlanTrajectory(const mabi_msgs::PlanTrajectoryGoalConstPtr &goal);
    //void cbWriteOccupancyGridToFile()
    bool cbWriteOccupancyGridToFile(mabi_msgs::WriteOcGridRequest &req, mabi_msgs::WriteOcGridResponse &res);
    //bool RvizPublisher::servicePublishTrajectory (varileg_msgs::ActionEndRequest &req, varileg_msgs::ActionEndResponse &res)

};

}
#endif //PERCEPTIVE_MPC_OMPLPLANNER_H
