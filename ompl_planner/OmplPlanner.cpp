//
// Created by nick on 29/09/2020.
//

#include "ompl_planner/OmplPlanner.h"

using namespace perceptive_mpc;
namespace ob = ompl::base;
namespace og = ompl::geometric;

OmplPlanner::OmplPlanner(const ros::NodeHandle& nodeHandle) : nh_(nodeHandle) {
    ROS_WARN("I am alive");
    ROS_INFO("Adaptive Trajectories initialized");

    sub_desired_end_effector_pose_subscriber_ =
            nh_.subscribe("/perceptive_mpc/desired_end_effector_pose", 1, &OmplPlanner::cbDesiredEndEffectorPose, this);
    pub_arm_state_ = nh_.advertise<sensor_msgs::JointState>("test",1);
    initializeState();
}

void OmplPlanner::initializeState() {
    current_state_.position2DBase << 0,0;
    current_state_.yaw_base = 0;
    current_state_.jointAngles.setZero();
}

void OmplPlanner::cbDesiredEndEffectorPose(const geometry_msgs::PoseStampedConstPtr &msgPtr) {
    sensor_msgs::JointState msg;
    msg.position.resize(3);
    msg.position[0] = msgPtr->pose.position.x;
    msg.position[1] = msgPtr->pose.position.y;
    msg.position[2] = msgPtr->pose.orientation.x;
    kindr::HomTransformQuatD goal_pose;
    kindr::Position3D test;
    Eigen::Matrix<float, 13, 1> start_position;

    pub_arm_state_.publish(msg);
    planTrajectory(start_position, goal_pose);
}

ompl::base::GoalPtr OmplPlanner::convertPoseToOmplGoal(const kindr::HomTransformQuatD& goal_pose) {
    auto space(std::make_shared<MabiStateSpace>());
    og::SimpleSetup ss(space);

    EndEffectorGoal::Settings eeGoalSettings;
    eeGoalSettings.goal = goal_pose;
    eeGoalSettings.positionTolerance = settings_.positionTolerance;
    eeGoalSettings.angularTolerance = settings_.orientationTolerance;
    eeGoalSettings.transformWrist2_X_Endeffector = settings_.transformWrist2_X_Endeffector;
    eeGoalSettings.transformBase_X_ArmMount = settings_.transformBase_X_ArmMount;
    auto endEffectorGoal = std::make_shared<EndEffectorGoal>(ss.getSpaceInformation(), eeGoalSettings);
    return endEffectorGoal;
}

void OmplPlanner::planTrajectory(const Eigen::Matrix<float, Definitions::STATE_DIM_, 1>& start_position, const kindr::HomTransformQuatD& goal_pose) {
    // define state space
    auto space(std::make_shared<MabiStateSpace>());

    // Bounds for the base
    RealVectorBounds basePositionBounds(2);
    basePositionBounds.low.resize(2);
    Eigen::Vector2d::Map(basePositionBounds.low.data(), basePositionBounds.low.size()) = settings_.minBasePositionLimit;
    basePositionBounds.high.resize(2);
    Eigen::Vector2d::Map(basePositionBounds.high.data(), basePositionBounds.high.size()) = settings_.maxBasePositionLimit;
    space->basePoseStateSpace()->setBounds(basePositionBounds);

    // Bounds for the Arm
    RealVectorBounds armPositionBounds(Definitions::ARM_STATE_DIM_);
    armPositionBounds.low.resize(Definitions::ARM_STATE_DIM_);
    Eigen::Matrix<double, Definitions::ARM_STATE_DIM_, 1>::Map(armPositionBounds.low.data(), armPositionBounds.low.size()) =
            settings_.minArmPositionLimits;
    armPositionBounds.high.resize(Definitions::ARM_STATE_DIM_);
    Eigen::Matrix<double, Definitions::ARM_STATE_DIM_, 1>::Map(armPositionBounds.high.data(), armPositionBounds.high.size()) =
            settings_.maxArmPositionLimits;
    space->armStateSpace()->setBounds(armPositionBounds);

    // define a simple setup class
    og::SimpleSetup ss(space);

    //Setting up the collision detection
    auto si = ss.getSpaceInformation();
    auto voxbloxStateValidityChecker = std::make_shared<VoxbloxStateValidityChecker>(si.get(), settings_.voxbloxCostConfig);
    ss.setStateValidityChecker(voxbloxStateValidityChecker);

    //Defining the start new //@todo See if this works
    ob::ScopedState<MabiStateSpace> start(space);
    start->basePoseState()->setXY(current_state_.position2DBase.x(), current_state_.position2DBase.y()); //todo Change to real observations
    start->basePoseState()->setYaw(current_state_.yaw_base);
    Eigen::VectorXd arm_state(6);
    arm_state << 0.0, 0.0, 0.0, 0, 0, 0; //todo change this to real observations
    for (int i = 0; i < arm_state.size(); i++) start->armState()->values[i] = current_state_.jointAngles[i];
    ss.setStartState(start);

    //Defining the start //@todo otherwhise choose this
//    ob::ScopedState<MabiStateSpace> start2(space);
//    mpcToOmplState(start_position, start2.get());
//    ss.setStartState(start);

    //Setting the goal
    auto goal = convertPoseToOmplGoal(goal_pose);
    ss.setGoal(goal);

    // Defining the planner
    // auto planner = std::make_shared<og::RRTXstatic>(si);
    // auto planner = std::make_shared<og::RRTConnect>(si);
    auto planner = std::make_shared<og::BFMT>(si);
    planner->setExtendedFMT(true);
    planner->setTermination(true);
    planner->setNumSamples(10000);
    ss.setPlanner(planner);
    ss.setup();

    std::cout << "goal threshold (position, orientation: (" << settings_.positionTolerance << ", " << settings_.orientationTolerance << ")"
              << std::endl;

    ob::PlannerStatus solved = ss.solve(settings_.maxPlanningTime);
    ROS_ERROR("Solved");
    // ob::PlannerStatus solved = ss.solve(exactSolnPlannerTerminationCondition(ss.getProblemDefinition()));

//    if (solved) {
//        std::cout << "Solved:" << solved.asString() << std::endl;
//        std::cout << "state count:" << ss.getSolutionPath().getStateCount() << std::endl;
//        std::cout << "computation time:" << ss.getLastPlanComputationTime() << std::endl;
//        std::cout << "length:" << ss.getSolutionPath().length() << std::endl;
//        // ss.print();
//        //    std::cout << "goal distance start: " << endEffectorGoal->distanceGoal(ss.getSolutionPath().getStates().front()) << std::endl;
//        //    std::cout << "goal distance end: " << endEffectorGoal->distanceGoal(ss.getSolutionPath().getStates().back()) << std::endl;
//        std::cout << "clearance: " << ss.getSolutionPath().clearance() << std::endl;
//
//        //    std::cout << "goal reached: " << endEffectorGoal->isSatisfied(ss.getSolutionPath().getStates().back()) << std::endl;
//        //    ss.getSolutionPath().asGeometric().printAsMatrix(std::cout);
//        // ss.getSolutionPath().print(std::cout);
//
//        auto output = std::make_unique<PlannerOutput>();
//
//        output->states.reserve(ss.getSolutionPath().getStateCount());
//        for (const auto& state : ss.getSolutionPath().getStates()) {
//            Eigen::VectorXd stateDyn;
//            omplToMpcState(state->as<MabiState>(), stateDyn);
//            output->states.push_back(stateDyn.head<STATE_DIM_>());
//        }
//        output->times = std::vector<double>(ss.getSolutionPath().getStateCount() - 1, 0.01);
//        return output;
//    } else {
//        std::cout << "No solution found after" << settings_.maxPlanningTime << "s." << std::endl;
//        return nullptr;
//    }
}

void OmplPlanner::publishArmState() {}