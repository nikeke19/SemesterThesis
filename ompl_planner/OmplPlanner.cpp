#include "ompl_planner/OmplPlanner.h"

using namespace perceptive_mpc;
namespace ob = ompl::base;
namespace og = ompl::geometric;

OmplPlanner::OmplPlanner(const ros::NodeHandle& nodeHandle) : nh_(nodeHandle), r_(100),
asPlanTrajectory_(nodeHandle, "plan_trajectory", boost::bind(&OmplPlanner::cbPlanTrajectory, this, _1), false) {
    ROS_WARN("I am alive");

    subDesiredEndEffectorPoseSubscriber_ =
            nh_.subscribe("/perceptive_mpc/desired_end_effector_pose", 1, &OmplPlanner::cbDesiredEndEffectorPose, this);
    pubArmState_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
    pubPointsOnRobot_ = nh_.advertise<visualization_msgs::MarkerArray>("/perceptive_mpc/collision_points", 1, false);
    while(ros::ok() && pubArmState_.getNumSubscribers() == 0)
        ros::Rate(100).sleep();
    serviceLoadMap_ = nh_.serviceClient<voxblox_msgs::FilePath>("/voxblox_node/load_map");

    initializeState();
    initializeKinematicInterfaceConfig();
    loadMap();
    setUpVoxbloxCostConfig();
    asPlanTrajectory_.start();
    srvWriteOccupancyGridToFile_ = nh_.advertiseService("write_oc_grid_to_file", &OmplPlanner::cbWriteOccupancyGridToFile, this);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Initialize
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void OmplPlanner::initializeState() {
    //Setting up own State
    currentState_.position2DBase << 0,0;
    currentState_.yaw_base = 0;
    currentState_.jointAngles << 0, -PI/2, 0, 0, -PI/2, PI/4;

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
    for(int i = 0; i < 100; i++) {
        jointState_.header.stamp = ros::Time::now();
        odomTrans_.header.stamp = ros::Time::now();
        pubArmState_.publish(jointState_);
        tfOdomBroadcaster_.sendTransform(odomTrans_);
        ros::spinOnce();
        r_.sleep();
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
    }

    //Writing everything to settings:
    settings_.transformBase_X_ArmMount = kinematicInterfaceConfig_.transformBase_X_ArmMount;
    settings_.transformWrist2_X_Endeffector = kinematicInterfaceConfig_.transformToolMount_X_Endeffector;
    kinematicsInterface_ = std::make_shared<MabiKinematics<ad_scalar_t>>(kinematicInterfaceConfig_);
}

void OmplPlanner::loadMap() {
    voxblox_msgs::FilePath srv;
//    srv.request.file_path = "/home/nick/mpc_ws/src/perceptive_mpc/maps/test.esdf";
    srv.request.file_path = "/home/nick/mpc_ws/src/perceptive_mpc/maps/example_map.esdf";
    serviceLoadMap_.waitForExistence();
    if (serviceLoadMap_.call(srv))
        ROS_INFO("Service load map called succesfully");
    else
        ROS_WARN("Could not load map, retry manually");
}

void OmplPlanner::setUpVoxbloxCostConfig() {
    KinematicInterfaceConfig kinematicInterfaceConfig;
    kinematicInterfaceConfig.transformToolMount_X_Endeffector = settings_.transformWrist2_X_Endeffector;
    kinematicInterfaceConfig.transformBase_X_ArmMount = settings_.transformBase_X_ArmMount;
    kinematicInterfaceConfig.baseMass = 70;
    kinematicInterfaceConfig.baseCOM = Eigen::Vector3d::Zero();

    auto kinematicInterfaceConfigPtr = std::make_shared<MabiKinematics<ad_scalar_t>>(kinematicInterfaceConfig);

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
    settings_.voxbloxCostConfig = voxbloxCostConfig;

    // To visualize
    geometry_msgs::TransformStamped quaternion;
    quaternion.transform.translation.x = currentState_.position2DBase.x();
    quaternion.transform.translation.y = currentState_.position2DBase.y();
    quaternion.transform.translation.z = 0;
    quaternion.transform.rotation = tf::createQuaternionMsgFromYaw(currentState_.yaw_base);
    sensor_msgs::JointState joints;
    joints.position.resize(6);
    for(int i=0; i < joints.position.size(); i++)
        joints.position[i] = currentState_.jointAngles[i];

    visualizeCollisionPoints(quaternion, joints);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// External Call functions to generate trajectory
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void OmplPlanner::cbPlanTrajectory(const mabi_msgs::PlanTrajectoryGoalConstPtr &goal) {
    // Setting new current state
    currentState_.position2DBase.x() = goal->current_state.pose_base.x;
    currentState_.position2DBase.x() = goal->current_state.pose_base.x;
    currentState_.yaw_base = goal->current_state.pose_base.theta;
    for(int i = 0; i < 6; i++)
        currentState_.jointAngles[i] = goal->current_state.joint_state.position[i];

    // Setting goal:
    kindr::HomTransformQuatD goalPose;
    kindr_ros::convertFromRosGeometryMsg(goal->goal_pose.pose, goalPose);

    //Enabling writing to file
    writeOccupancyGridToFile_ = true;
    writeConditioningToFile_ = true;

    planTrajectoryResult_.trajectory_found = planTrajectory(goalPose, goal->file_name);
    asPlanTrajectory_.setSucceeded(planTrajectoryResult_, "Motion plan executed");
}

bool OmplPlanner::cbWriteOccupancyGridToFile(mabi_msgs::WriteOcGridRequest &req, mabi_msgs::WriteOcGridResponse &res) {
    writeOccupancyGridToFile(req.resolution, req.name);
    res.wrote_to_file = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Calculating Goal
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void OmplPlanner::cbDesiredEndEffectorPose(const geometry_msgs::PoseStampedConstPtr &msgPtr) {
    kindr::HomTransformQuatD goalPose;
    kindr_ros::convertFromRosGeometryMsg(msgPtr->pose, goalPose);

    ROS_ERROR("OMPL: Received Desired End Effector CB");
    planTrajectory(goalPose, "test");
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

bool OmplPlanner::planTrajectory(const kindr::HomTransformQuatD &goal_pose, std::string name) {
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
    esdfCachingServer_->updateInterpolator();
    std::shared_ptr<VoxbloxStateValidityChecker> voxbloxStateValidityChecker = std::make_shared<VoxbloxStateValidityChecker>(si.get(), settings_.voxbloxCostConfig);
    ss.setStateValidityChecker(voxbloxStateValidityChecker);

    // Defining the start
    {
        boost::shared_lock<boost::shared_mutex> lock(observationMutex_);
        ob::ScopedState<MabiStateSpace> start(space);
        start->basePoseState()->setXY(currentState_.position2DBase.x(), currentState_.position2DBase.y());
        start->basePoseState()->setYaw(currentState_.yaw_base);
        for (int i = 0; i < currentState_.jointAngles.size(); i++)
            start->armState()->values[i] = currentState_.jointAngles[i];
        ss.setStartState(start);
    }

    //Setting the goal
    auto goal = convertPoseToOmplGoal(goal_pose);
    ss.setGoal(goal);

    // Defining the planner
    auto planner = std::make_shared<og::RRT>(si);
//    auto planner = std::make_shared<og::RRTstar>(si);
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
        std::cout <<"state count is" << ss.getSolutionPath().getStateCount();

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

        if(writeSolutionTrajectoryToFile_)
            writeTrajectoryToFile(solutionTrajectory, name);
        if(writeConditioningToFile_)
            writeConditioningToFile(ss.getSolutionPath().getState(n_points-1)->as<MabiState>(), solutionTrajectory[0], name);
//        if(writeOccupancyGridToFile_)
//            writeOccupancyGridToFile(0.1, "test");

        ROS_WARN("Achieved Output");
        return true;
    }
    else {
        std::cout << "No solution found after" << settings_.maxPlanningTime << "s." << std::endl;
        return false;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Publish Solution Trajectory to RVIZ
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void OmplPlanner::publishSolutionTrajectory(const std::vector<CurrentState>& solutionTrajectory) {
    for(int i = 0; i < solutionTrajectory.size() && ros::ok(); i++) {
        //Setting up Transform for the base
        odomTrans_.header.stamp = ros::Time::now();
        odomTrans_.transform.translation.x = solutionTrajectory[i].position2DBase.x();
        odomTrans_.transform.translation.y = solutionTrajectory[i].position2DBase.y();
        odomTrans_.transform.translation.z = 0.0;
        odomTrans_.transform.rotation = tf::createQuaternionMsgFromYaw(solutionTrajectory[i].yaw_base);

        //Setting up the joint state
        jointState_.header.stamp = ros::Time::now();

        for(int k=0; k < jointState_.position.size(); k++)
            jointState_.position[k] = solutionTrajectory[i].jointAngles[k];

        //publishing base transform and joint states
        pubArmState_.publish(jointState_);
        tfOdomBroadcaster_.sendTransform(odomTrans_);
        visualizeCollisionPoints(odomTrans_, jointState_);
        ros::spinOnce();
        ros::Rate(25).sleep();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Write Data to file
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void OmplPlanner::writeTrajectoryToFile(const std::vector<CurrentState>& trajectory, const std::string& name) {
    ROS_ERROR("Writing trajectory to file");

    std::ofstream trajectoryFile;
    trajectoryFile.open("/home/nick/Data/Table/" + name + "_trajectory.txt");

    //Filling header of trajectory file:
    trajectoryFile << "Position_x" << "," << "Position_y" << "," << "Yaw";
    for (int k = 0; k < 6; k++)
        trajectoryFile << "," << "Joint_" + std::to_string(k);
    trajectoryFile << std::endl;

    int n = trajectory.size();

    //Filling trajectory file
    for (int i = 1; i < n; i++) {
        trajectoryFile << trajectory[i].position2DBase.x() << ","
                        << trajectory[i].position2DBase.y() << ","
                        << trajectory[i].yaw_base;
        for (int k = 0; k < 6; k++)
            trajectoryFile << "," << trajectory[i].jointAngles[k];

        trajectoryFile << std::endl;
    }
    trajectoryFile.close();
    ROS_ERROR("Writing finished");
}

void OmplPlanner::writeConditioningToFile(const MabiStateSpace::StateType* goalState, CurrentState startState ,std::string name) {
    // Obtaining position and orientation of the Endeffector in the Goal State
    Eigen::VectorXd goalMpcState;
    Eigen::Matrix<double, 4, 4> goalEndeffectorTransform;
    MabiKinematics<double> kinematics(kinematicInterfaceConfig_);
    Eigen::Vector3d goalEndeffectorPosition;

    omplToMpcState(goalState, goalMpcState);
    kinematics.computeState2EndeffectorTransform(goalEndeffectorTransform, goalMpcState);
    Eigen::Quaterniond goalEndeffectorRotation(goalEndeffectorTransform.block<3,3>(0, 0));
    goalEndeffectorPosition = goalEndeffectorTransform.block<3,1>(0,3);

    //Opening file
    std::ofstream conditionFile;
    conditionFile.open("/home/nick/Data/Table/" + name + "_condition.txt");

    //Filling header
    conditionFile << "Start_x" << "," << "Start_y" << "," << "Start_yaw" << "," << "Goal_x" << "," << "Goal_y" << ","
                  << "Goal_z" << "," << "Goal_q_w" << "," << "Goal_q_x" << "," << "Goal_q_y" << "," << "Goal_q_z" << std::endl;
    //Putting content in
    conditionFile << startState.position2DBase.x() << "," << startState.position2DBase.y() << "," << startState.yaw_base
                  << "," << goalEndeffectorPosition.x() << "," << goalEndeffectorPosition.y() << ","
                  << goalEndeffectorPosition.z() << "," << goalEndeffectorRotation.w() << ","
                  << goalEndeffectorRotation.x() << "," << goalEndeffectorRotation.y() << ","
                  << goalEndeffectorRotation.z() << std::endl;
    conditionFile.close();
}

void OmplPlanner::writeOccupancyGridToFile(float resolution, std::string name) {
    ROS_WARN("Writing Occupancy Grid");
    Eigen::Matrix<float, 3, 1> checkPoint;
    float distance;
    auto interpolator = esdfCachingServer_->getInterpolator();

    const int n_x_points = int((settings_.maxBasePositionLimit(0) - settings_.minBasePositionLimit(0))/resolution) + 1;
    const int n_y_points = int((settings_.maxBasePositionLimit(1) - settings_.minBasePositionLimit(1))/resolution) + 1;
    const int n_z_points = int((settings_.minMaxHeight(1) - settings_.minMaxHeight(0))/resolution) + 1;


    //Allocate Memory for 3D occupancy Grid
    int*** occupancyGrid = new int**[n_z_points];
    for (int i = 0; i < n_z_points; i++) {
        occupancyGrid[i] = new int*[n_y_points];

        for (int j = 0; j < n_y_points; j++)
            occupancyGrid[i][j] = new int[n_x_points];
    }

    std::vector<Eigen::Matrix<float, 3, 1>> collisionPoints;
    std::ofstream occupancyGridFile;
    occupancyGridFile.open("/home/nick/Data/Table/" + name + "_occupancy_grid.txt");

    //Filling header
    //occupancyGridFile << "Start_x" << "," << "Start_y" << "," << "Start_z" << std::endl;
    occupancyGridFile << "Grid_Points" << std::endl;


    //Start to fill occupancy Grid
    for(int i_z = 0; i_z < n_z_points; i_z++) {
        for(int i_y = 0; i_y < n_y_points; i_y++) {
            for(int i_x = 0; i_x < n_x_points; i_x++) {
                checkPoint = {float(settings_.minBasePositionLimit[0] + i_x * resolution) ,
                              float(settings_.minBasePositionLimit[1] + i_y * resolution),
                              float(settings_.minMaxHeight[0] + i_z * resolution)};
                interpolator->getInterpolatedDistance(checkPoint, &distance);

                if(distance <= resolution / 2) {
                    occupancyGrid[i_z][i_y][i_x] = 1;
                    //z > 0 to ensure that ground is not included
                    if(checkPoint.z() > 0.01)
                        occupancyGridFile << 1 << ",";
                    else
                        occupancyGridFile << 0 << ",";


                    // todo Only for debug
                    if(checkPoint.z() > 0.01
                       && checkPoint.x() < 2
                       && checkPoint.x() > -2
                       && checkPoint.y() < 2
                       && checkPoint.y() > -2) {
                        //occupancyGridFile << checkPoint.x() << "," << checkPoint.y() << "," << checkPoint.z() << std::endl;
                        collisionPoints.insert(collisionPoints.end(), checkPoint);
                        //std::cout << checkPoint << std::endl << std::endl;
                    }
                    
                }
                else {
                    occupancyGrid[i_z][i_y][i_x] = 0;
                    occupancyGridFile << 0 << ",";
                }
            }
            occupancyGridFile << std::endl;
        }
        occupancyGridFile << std::endl;
    }

    ROS_WARN("Finished Occupancy Grid");
    occupancyGridFile.close();
    visualizeOccupancyGrid(collisionPoints);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Visualization in RVIZ
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void OmplPlanner::visualizeCollisionPoints(const geometry_msgs::TransformStamped& base, sensor_msgs::JointState joints) {
    Eigen::Matrix<double, 4, 1> quat(base.transform.rotation.w, base.transform.rotation.x,
                                     base.transform.rotation.y, base.transform.rotation.z);
    Eigen::Matrix<double, 3,1> positionBase(base.transform.translation.x, base.transform.translation.y, base.transform.translation.z);
    Eigen::Matrix<double, 6,1> jointsMatrix;
    for(int i = 0; i < joints.position.size(); i++)
        jointsMatrix[i] = joints.position[i];

    Eigen::Matrix<double, 13,1> state;
    state.block<3,1>(0,0) = positionBase;
    state.block<4,1>(3,0) = quat;
    state.block<6,1>(7,0) = jointsMatrix;

    pubPointsOnRobot_.publish(pointsOnRobot_->getVisualization(state));
}

void OmplPlanner::visualizeOccupancyGrid(const std::vector<Eigen::Matrix<float, 3, 1>> collisionPoints) {
    int n = collisionPoints.size();
    visualization_msgs::MarkerArray markerArray;
    markerArray.markers.resize(n);

    for (int i = 0; i < n; i++) {
        auto& marker = markerArray.markers[i];
        marker.type = visualization_msgs::Marker::Type::SPHERE;
        marker.id = i;
        marker.action = 0;
        marker.scale.x = 0.4;
        marker.scale.y = 0.4;
        marker.scale.z = 0.4;
        marker.pose.position.x = collisionPoints[i].x();
        marker.pose.position.y = collisionPoints[i].y();
        marker.pose.position.z = collisionPoints[i].z();

        marker.pose.orientation.w = 1;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;

        marker.color.a = 0.5;
        marker.color.r = 0.0;
        marker.color.b = 0.0;
        marker.color.g = 0.5;

        marker.frame_locked = true;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time::now();
    }
    pubPointsOnRobot_.publish(markerArray);
}
