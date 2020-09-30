//
// Created by johannes on 05.05.20.
//
#include "ocs2_mobile_manipulator_interface/sampling_planner/EndEffectorGoal.h"
#include <ocs2_mobile_manipulator_interface/sampling_planner/OmplStateConversions.h>
#include <mabi_mobile_robcogen/MabiMobileTransforms.hpp>

#include "ompl_planner/EndEffectorGoal.h"
#include "ompl_planner/OmplStateConversions.h"

using namespace perceptive_mpc;
EndEffectorGoal::EndEffectorGoal(SpaceInformationPtr si, Settings settings) : GoalRegion(si), settings_(settings) {
  setThreshold(1.0);
}

double EndEffectorGoal::distanceGoal(const State* st) const {
  Eigen::VectorXd mpcState;
  omplToMpcState(st->as<MabiStateSpace::StateType>(), mpcState);

  Eigen::Matrix4d endEffectorPose;
  mabi_mobile_robcogen::MabiMobileTransforms<double>::computeState2EndeffectorTransform(
          endEffectorPose, mpcState, settings_.transformBase_X_ArmMount, settings_.transformWrist2_X_Endeffector);

  double angularError =
      std::abs(settings_.goal.getRotation().toImplementation().angularDistance(Eigen::Quaterniond(endEffectorPose.block<3, 3>(0, 0))));
  double positionError = (endEffectorPose.block<3, 1>(0, 3) - settings_.goal.getPosition().toImplementation()).norm();

  return std::max(positionError / settings_.positionTolerance, angularError / settings_.angularTolerance);
}
