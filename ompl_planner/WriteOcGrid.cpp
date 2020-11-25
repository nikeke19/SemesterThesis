//
// Created by nick on 06/11/2020.
//

#include "ompl_planner/WriteOcGrid.h"

using namespace perceptive_mpc;

WriteOcGrid::WriteOcGrid(const ros::NodeHandle &nodeHandle) : nh_(nodeHandle) {
    ROS_INFO("hi, oc grid writer alive");
    serviceLoadMap_ = nh_.serviceClient<voxblox_msgs::FilePath>("/voxblox_node/load_map");
    srvWriteOccupancyGridToFile_ = nh_.advertiseService("write_oc_grid_to_file",
                                                        &WriteOcGrid::cbWriteOccupancyGridToFile, this);
    srvServerLoadMap_ = nh_.advertiseService("load_map", &WriteOcGrid::cbLoadMap, this);
    esdfCachingServer_.reset(new voxblox::EsdfCachingServer(ros::NodeHandle(), ros::NodeHandle("~")));
}

bool WriteOcGrid::cbLoadMap(mabi_msgs::LoadMapRequest &req, mabi_msgs::LoadMapResponse &res) {
    voxblox_msgs::FilePath srv;
    srv.request.file_path = req.name;
    ROS_INFO_STREAM("file path: " << req.name);
    serviceLoadMap_.waitForExistence();
    esdfCachingServer_->enableMeshUpdate();
    if (serviceLoadMap_.call(srv)) {
        ROS_INFO("Service load map called succesfully");
        res.map_loaded = true;
        return true;
    }
    else {
        ROS_WARN("Could not load map, retry manually");
        ROS_WARN_STREAM("file path is:" << req.name);
        res.map_loaded = false;
        return false;
    }
}

bool WriteOcGrid::cbWriteOccupancyGridToFile(mabi_msgs::WriteCenteredOcGridRequest &req,
                                             mabi_msgs::WriteCenteredOcGridResponse &res) {

    Eigen::Matrix<float, 2, 1> center = {req.center[0], req.center[1]};
    ros::Rate r(1);
    while(ros::ok() && esdfCachingServer_->getMeshUpdateState()) { // Mesh update true if no map loaded
        ROS_INFO("Map not loaded yet");
        r.sleep();
    }

    writeOccupancyGridToFile(req.resolution, req.name, center);
    res.wrote_to_file = true;
    return true;
}

void WriteOcGrid::writeOccupancyGridToFile(const float resolution, std::string name, const Eigen::Matrix<float, 2, 1> center) {
    ROS_INFO("Writing Occupancy Grid");
    Eigen::Matrix<float, 3, 1> checkPoint;
    float distance;
    esdfCachingServer_->updateInterpolator();
    auto interpolator = esdfCachingServer_->getInterpolator();
    Settings settings;

    const int n_x_points =
            int((settings.maxBasePositionLimit(0) - settings.minBasePositionLimit(0)) / resolution) + 1;
    const int n_y_points =
            int((settings.maxBasePositionLimit(1) - settings.minBasePositionLimit(1)) / resolution) + 1;
    const int n_z_points = int((settings.minMaxHeight(1) - settings.minMaxHeight(0)) / resolution) + 1;


    //Allocate Memory for 3D occupancy Grid
//    int ***occupancyGrid = new int **[n_z_points];
//    for (int i = 0; i < n_z_points; i++) {
//        occupancyGrid[i] = new int *[n_y_points];
//
//        for (int j = 0; j < n_y_points; j++)
//            occupancyGrid[i][j] = new int[n_x_points];
//    }

    // Writing center to file
    std::ofstream centerFile;
    std::string file_center = std::string("/home/nick/Data/Table/world_") + name[6] + std::string("_center.csv");
    centerFile.open(file_center, std::ios_base::app);
    centerFile << std::endl;
    centerFile << name << ","  << center(0) << "," << center(1);
    centerFile.close();

    std::vector<Eigen::Matrix<float, 3, 1>> collisionPoints;
    std::ofstream occupancyGridFile;
    std::string file_name = "/home/nick/Data/Table/" + name + "_occupancy_grid.txt";
    ROS_INFO_STREAM(file_name);
    occupancyGridFile.open("/home/nick/Data/Table/" + name + "_occupancy_grid.txt");

    //Start to fill occupancy Grid
    for (int i_z = 0; i_z < n_z_points; i_z++) {
        for (int i_y = 0; i_y < n_y_points; i_y++) {
            for (int i_x = 0; i_x < n_x_points; i_x++) {
                checkPoint = {float(settings.minBasePositionLimit[0] + i_x * resolution + center[0]),
                              float(settings.minBasePositionLimit[1] + i_y * resolution + center[1]),
                              float(0.01 + i_z * resolution)};
                interpolator->getInterpolatedDistance(checkPoint, &distance);

                if (distance <= resolution / 2
                    && checkPoint.x() < 5.1 && checkPoint.x() > -4.1        //Voxblox is weird outside map [-5,5]
                    && checkPoint.y() < 5.1 && checkPoint.y() > -5.1) {
//                    occupancyGrid[i_z][i_y][i_x] = 1;
                    occupancyGridFile << 1 << ",";
                } else {
//                    occupancyGrid[i_z][i_y][i_x] = 0;
                    occupancyGridFile << 0 << ",";
                }
            }
            occupancyGridFile << std::endl;
        }
        occupancyGridFile << std::endl;
    }

    ROS_INFO("Finished Occupancy Grid");
    occupancyGridFile.close();
}