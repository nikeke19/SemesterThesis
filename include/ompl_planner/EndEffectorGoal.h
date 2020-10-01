//
// Created by johannes on 05.05.20.
//
#pragma once
#include <kindr/poses/HomogeneousTransformation.hpp>
#include <ompl/base/goals/GoalRegion.h>

#include "EndEffectorGoal.h"
#include "OmplStateConversions.h"
#include "perceptive_mpc/kinematics/KinematicsInterface.hpp"
#include "perceptive_mpc/kinematics/mabi/MabiKinematics.hpp"

namespace perceptive_mpc {
using namespace ompl::base;
class EndEffectorGoal : public ompl::base::GoalRegion {
 public:
  struct Settings {
    kindr::HomTransformQuatD goal;
    double positionTolerance = 0.1;
    double angularTolerance = 0.1;
    Eigen::Matrix4d transformBase_X_ArmMount = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d transformWrist2_X_Endeffector = Eigen::Matrix4d::Identity();
  };
  EndEffectorGoal(SpaceInformationPtr si, Settings settings);
  double distanceGoal(const State* st) const override;

 private:
  Settings settings_;
};

}  // namespace ocs2_mm