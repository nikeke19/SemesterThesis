//
// Created by nick on 06/11/2020.
//

#ifndef PERCEPTIVE_MPC_WRITEOCGRID_H
#define PERCEPTIVE_MPC_WRITEOCGRID_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include "mabi_msgs/WriteCenteredOcGrid.h"

// Custom Classes
#include "VoxbloxStateValidityChecker.h"
#include "perceptive_mpc/EsdfCachingServer.hpp"
#include "voxblox_msgs/FilePath.h"
#include "ompl_planner/OmplPlanner.h"
#include "mabi_msgs/LoadMap.h"

namespace perceptive_mpc {

    class WriteOcGrid {
    public:
        explicit WriteOcGrid(const ros::NodeHandle &nodeHandle);

        ~WriteOcGrid() = default;

    private:
        ros::NodeHandle nh_;
        std::shared_ptr<voxblox::EsdfCachingServer> esdfCachingServer_;
        ros::ServiceClient serviceLoadMap_;
        ros::ServiceServer srvServerLoadMap_;
        ros::ServiceServer srvWriteOccupancyGridToFile_;

        bool cbWriteOccupancyGridToFile(mabi_msgs::WriteCenteredOcGridRequest &req,
                                        mabi_msgs::WriteCenteredOcGridResponse &res);
        bool cbLoadMap(mabi_msgs::LoadMapRequest &req, mabi_msgs::LoadMapResponse &res);

        void writeOccupancyGridToFile(const float resolution, std::string name, const Eigen::Matrix<float, 2, 1> center);

    };
}


#endif //PERCEPTIVE_MPC_WRITEOCGRID_H
