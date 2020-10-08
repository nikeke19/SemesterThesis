//
// Created by nick on 29/09/2020.
//

#include "ompl_planner/OmplPlanner.h"

using namespace perceptive_mpc;
namespace ob = ompl::base;
namespace og = ompl::geometric;

OmplPlanner::OmplPlanner(const ros::NodeHandle& nodeHandle) : nh_(nodeHandle), r_(100) {
    ROS_WARN("I am alive");

    subDesiredEndEffectorPoseSubscriber_ =
            nh_.subscribe("/perceptive_mpc/desired_end_effector_pose", 1, &OmplPlanner::cbDesiredEndEffectorPose, this);
    pubArmState_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
    while(ros::ok() && pubArmState_.getNumSubscribers() == 0)
        ros::Rate(100).sleep();
    serviceLoadMap_ = nh_.serviceClient<voxblox_msgs::FilePath>("/voxblox_node/load_map");

    //ros::Duration(15).sleep();
    initializeState();
    initializeKinematicInterfaceConfig();
    loadMap();
    ros::Duration(10).sleep();
    setUpVoxbloxCostConfig();
}
//@todo for debug
bool isStateValid(const ob::State *state) {
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Initialize
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void OmplPlanner::initializeState() {
    //Setting up own State
    currentState_.position2DBase << 0,0;
    currentState_.yaw_base = 0;
    currentState_.jointAngles << 0, -PI/2, -PI/4, 0, -PI/2, PI/4;

    //Setting up Transform for the base
    odomTrans_.header.frame_id = "odom";
    odomTrans_.child_frame_id = "base_link";
    odomTrans_.header.stamp = ros::Time::now();
    odomTrans_.transform.translation.x = currentState_.position2DBase.x();
    odomTrans_.transform.translation.y = currentState_.position2DBase.y();
    odomTrans_.transform.translation.z = 0;
    odomTrans_.transform.rotation = tf::createQuaternionMsgFromYaw(currentState_.yaw_base);

    //Setting up the joint state
    jointState_.header.stamp = ros::Time::now();
    jointState_.name = {"SH_ROT", "SH_FLE", "EL_FLE", "EL_ROT", "WR_FLE", "WR_ROT"};
    jointState_.position.resize(6);

    for(int i=0; i < jointState_.position.size(); i++)
        jointState_.position[i] = currentState_.jointAngles[i];

    //publishing base transform and joint states
    pubArmState_.publish(jointState_);
    tfOdomBroadcaster_.sendTransform(odomTrans_);

    testLoop();
}
void OmplPlanner::testLoop() {
    int i = 0;
    while(ros::ok() == true && i < 100) {
        jointState_.header.stamp = ros::Time::now();
        odomTrans_.header.stamp = ros::Time::now();
        pubArmState_.publish(jointState_);
        tfOdomBroadcaster_.sendTransform(odomTrans_);
        ros::spinOnce();
        r_.sleep();
        i++;
    }
}

void OmplPlanner::initializeKinematicInterfaceConfig() {
    MabiKinematics<double> kinematics(kinematicInterfaceConfig_);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    {
        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = tfBuffer.lookupTransform("base_link", kinematics.armMountLinkName(), ros::Time(0), ros::Duration(1.0));
        } catch (tf2::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
            throw;
        }
        Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                                transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

        kinematicInterfaceConfig_.transformBase_X_ArmMount = Eigen::Matrix4d::Identity();
        kinematicInterfaceConfig_.transformBase_X_ArmMount.block<3, 3>(0, 0) = quat.toRotationMatrix();

        kinematicInterfaceConfig_.transformBase_X_ArmMount(0, 3) = transformStamped.transform.translation.x;
        kinematicInterfaceConfig_.transformBase_X_ArmMount(1, 3) = transformStamped.transform.translation.y;
        kinematicInterfaceConfig_.transformBase_X_ArmMount(2, 3) = transformStamped.transform.translation.z;
        //ROS_INFO_STREAM("baseToArmMount_: " << std::endl << kinematicInterfaceConfig_.transformBase_X_ArmMount);
    }

    {
        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = tfBuffer.lookupTransform(kinematics.toolMountLinkName(), "ENDEFFECTOR", ros::Time(0), ros::Duration(1.0));
        } catch (tf2::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
            throw;
        }
        Eigen::Quaterniond quat(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                                transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

        kinematicInterfaceConfig_.transformToolMount_X_Endeffector = Eigen::Matrix4d::Identity();
        kinematicInterfaceConfig_.transformToolMount_X_Endeffector.block<3, 3>(0, 0) = quat.toRotationMatrix();

        kinematicInterfaceConfig_.transformToolMount_X_Endeffector(0, 3) = transformStamped.transform.translation.x;
        kinematicInterfaceConfig_.transformToolMount_X_Endeffector(1, 3) = transformStamped.transform.translation.y;
        kinematicInterfaceConfig_.transformToolMount_X_Endeffector(2, 3) = transformStamped.transform.translation.z;
        //ROS_INFO_STREAM("wrist2ToEETransform_: " << std::endl << kinematicInterfaceConfig_.transformToolMount_X_Endeffector);
    }

    //Writing everything to settings:
    settings_.transformBase_X_ArmMount = kinematicInterfaceConfig_.transformBase_X_ArmMount;
    settings_.transformWrist2_X_Endeffector = kinematicInterfaceConfig_.transformToolMount_X_Endeffector;
}

void OmplPlanner::loadMap() {
    voxblox_msgs::FilePath srv;
    srv.request.file_path = "/home/nick/mpc_ws/src/perceptive_mpc/example/example_map.esdf";
    serviceLoadMap_.waitForExistence();
    if (serviceLoadMap_.call(srv))
        ROS_INFO("Service load map called succesfully");
    else
        ROS_ERROR("Could not load map, try manually");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Calculating Goal
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void OmplPlanner::cbDesiredEndEffectorPose(const geometry_msgs::PoseStampedConstPtr &msgPtr) {
    kindr::HomTransformQuatD goal_pose;
    kindr_ros::convertFromRosGeometryMsg(msgPtr->pose, goal_pose);

    ROS_ERROR("OMPL: Received Desired End Effector CB");
    planTrajectory(goal_pose);
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Planning Trajectory
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void OmplPlanner::planTrajectory(const kindr::HomTransformQuatD& goal_pose) {
    ROS_ERROR("OMPL: Create State Space ");

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
    //setUpVoxbloxCostConfig();
    esdfCachingServer_->updateInterpolator();
    auto voxbloxStateValidityChecker = std::make_shared<VoxbloxStateValidityChecker>(si.get(), settings_.voxbloxCostConfig);
    ss.setStateValidityChecker(voxbloxStateValidityChecker);

    //todo alternative to only run
//    auto si = ss.getSpaceInformation();
//    ss.setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });


    //Defining the start new //@todo See if this works
    ob::ScopedState<MabiStateSpace> start(space);
    start->basePoseState()->setXY(currentState_.position2DBase.x(), currentState_.position2DBase.y()); //todo Change to real observations
    start->basePoseState()->setYaw(currentState_.yaw_base);
    for (int i = 0; i < currentState_.jointAngles.size(); i++)
        start->armState()->values[i] = currentState_.jointAngles[i];
    ss.setStartState(start);

    //Setting the goal
    auto goal = convertPoseToOmplGoal(goal_pose);
    ss.setGoal(goal);

    // Defining the planner
//    auto planner = std::make_shared<og::RRT>(si);
    auto planner = std::make_shared<og::RRTstar>(si);
    planner->setRange(0.1);
    planner->setGoalBias(0.5);
    ss.setPlanner(planner);
    ss.setup();

    std::cout << "goal threshold (position, orientation: (" << settings_.positionTolerance << ", " << settings_.orientationTolerance << ")"
              << std::endl;

    ob::PlannerStatus solved = ss.solve(settings_.maxPlanningTime);

    if (solved) {
        std::cout << "Solved:" << solved.asString() << std::endl;

        //Adding more points to solution trajectory
        int n_points = int(ss.getSolutionPath().length() *10);
        std::cout << "number of points to add"<< n_points <<std::endl;
        ss.getSolutionPath().interpolate(n_points);

        //Object to store solution in
        std::vector<CurrentState> solutionTrajectory;
        solutionTrajectory.resize(ss.getSolutionPath().getStateCount());

        int i = 0;
        for (const auto& state : ss.getSolutionPath().getStates()) {
            auto mabiSpace = state->as<MabiState>();
            CurrentState trajectoryPoint;

            trajectoryPoint.position2DBase.x() = mabiSpace->basePoseState()->getX();
            trajectoryPoint.position2DBase.y() = mabiSpace->basePoseState()->getY();
            trajectoryPoint.yaw_base = mabiSpace->basePoseState()->getYaw();

            for(int k = 0; k < 6; k++)
                trajectoryPoint.jointAngles[k] = mabiSpace->armState()->values[k];

            solutionTrajectory[i++] = trajectoryPoint;
        }

        publishSolutionTrajectory(solutionTrajectory);
        ROS_WARN("Achieved Output");

    }
    else {
        std::cout << "No solution found after" << settings_.maxPlanningTime << "s." << std::endl;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Publish to RVIZ
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void OmplPlanner::publishSolutionTrajectory(const std::vector<CurrentState>& solutionTrajectory) {
    for(int i = 0; i < solutionTrajectory.size() && ros::ok(); i++) {
        //Setting up Transform for the base
        odomTrans_.header.stamp = ros::Time::now();
        odomTrans_.transform.translation.x = solutionTrajectory[i].position2DBase.x();
        odomTrans_.transform.translation.y = solutionTrajectory[i].position2DBase.y();
        odomTrans_.transform.rotation = tf::createQuaternionMsgFromYaw(solutionTrajectory[i].yaw_base);

        //Setting up the joint state
        jointState_.header.stamp = ros::Time::now();

        for(int k=0; k < jointState_.position.size(); k++)
            jointState_.position[k] = solutionTrajectory[i].jointAngles[k];

        //publishing base transform and joint states
        pubArmState_.publish(jointState_);
        tfOdomBroadcaster_.sendTransform(odomTrans_);
        ros::spinOnce();
        ros::Rate(25).sleep();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Collision check
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void OmplPlanner::setUpVoxbloxCostConfig() {
    KinematicInterfaceConfig kinematicInterfaceConfig;
    kinematicInterfaceConfig.transformToolMount_X_Endeffector = settings_.transformWrist2_X_Endeffector;
    kinematicInterfaceConfig.transformBase_X_ArmMount = settings_.transformBase_X_ArmMount;
    kinematicInterfaceConfig.baseMass = 70;
    kinematicInterfaceConfig.baseCOM = Eigen::Vector3d::Zero();

    auto kinematicInterfaceConfigPtr = std::make_shared<MabiKinematics<ad_scalar_t>>(kinematicInterfaceConfig);
    //auto kinematicInterfaceConfigPtr = std::shared_ptr<KinematicsInterface<ad_scalar_t>>(kinematicInterfaceConfig)


    ros::NodeHandle pNh("~");
    std::shared_ptr<VoxbloxCostConfig> voxbloxCostConfig = nullptr;

    if (pNh.hasParam("collision_points")) {
        perceptive_mpc::PointsOnRobot::points_radii_t pointsAndRadii(8);
        using pair_t = std::pair<double, double>;

        XmlRpc::XmlRpcValue collisionPoints;
        pNh.getParam("collision_points", collisionPoints);
        if (collisionPoints.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_WARN("collision_points parameter is not of type array.");
        }
        for (int i = 0; i < collisionPoints.size(); i++) {
            if (collisionPoints.getType() != XmlRpc::XmlRpcValue::TypeArray) {
                ROS_WARN_STREAM("collision_points[" << i << "] parameter is not of type array.");
                break;
            }
            for (int j = 0; j < collisionPoints[i].size(); j++) {
                if (collisionPoints[j].getType() != XmlRpc::XmlRpcValue::TypeArray) {
                    ROS_WARN_STREAM("collision_points[" << i << "][" << j << "] parameter is not of type array.");
                    break;
                }
                if (collisionPoints[i][j].size() != 2) {
                    ROS_WARN_STREAM("collision_points[" << i << "][" << j << "] does not have 2 elements.");
                    break;
                }
                double segmentId = collisionPoints[i][j][0];
                double radius = collisionPoints[i][j][1];
                pointsAndRadii[i].push_back(pair_t(segmentId, radius));
                ROS_INFO_STREAM("segment=" << i << ". relative pos on segment:" << segmentId << ". radius:" << radius);
            }
        }
        perceptive_mpc::PointsOnRobotConfig config;
        config.pointsAndRadii = pointsAndRadii;
        using ad_type = CppAD::AD<CppAD::cg::CG<double>>;
        config.kinematics = kinematicInterfaceConfigPtr;

        std::shared_ptr<PointsOnRobot> pointsOnRobot_;
        pointsOnRobot_.reset(new perceptive_mpc::PointsOnRobot(config));

        if (pointsOnRobot_->numOfPoints() > 0) {
            voxbloxCostConfig.reset(new VoxbloxCostConfig());
            voxbloxCostConfig->pointsOnRobot = pointsOnRobot_;

            esdfCachingServer_.reset(new voxblox::EsdfCachingServer(ros::NodeHandle(), ros::NodeHandle("~")));
            voxbloxCostConfig->interpolator = esdfCachingServer_->getInterpolator();

            pointsOnRobot_->initialize("points_on_robot");

        } else {
            // if there are no points defined for collision checking, set this pointer to null to disable the visualization
            pointsOnRobot_ = nullptr;
        }
    }

    settings_.voxbloxCostConfig.swap(voxbloxCostConfig);
}

