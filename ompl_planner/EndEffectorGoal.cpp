//
// Created by johannes on 05.05.20.
//
//#include "ocs2_mobile_manipulator_interface/sampling_planner/EndEffectorGoal.h"
//#include <ocs2_mobile_manipulator_interface/sampling_planner/OmplStateConversions.h>
//#include <mabi_mobile_robcogen/MabiMobileTransforms.hpp>


#include "ompl_planner/EndEffectorGoal.h"

using namespace perceptive_mpc;
EndEffectorGoal::EndEffectorGoal(SpaceInformationPtr si, Settings settings) : GoalRegion(si), settings_(settings) {
  setThreshold(1); //todo was 1
}

double EndEffectorGoal::distanceGoal(const State* st) const {
  Eigen::VectorXd mpcState;
  omplToMpcState(st->as<MabiStateSpace::StateType>(), mpcState);

  std::cout << "Mpc State" << std::endl;
  std::cout << mpcState << std::endl <<std::endl;

  //New See if it works //todo see if it works
  KinematicInterfaceConfig kinematicInterfaceConfig;
  kinematicInterfaceConfig.transformToolMount_X_Endeffector = settings_.transformWrist2_X_Endeffector;
  kinematicInterfaceConfig.transformBase_X_ArmMount = settings_.transformBase_X_ArmMount;
  kinematicInterfaceConfig.baseMass = 70;
  kinematicInterfaceConfig.baseCOM = Eigen::Vector3d::Zero();

  MabiKinematics<double> kinematics(kinematicInterfaceConfig);
  Eigen::Matrix4d endEffectorPose;
  kinematics.computeState2EndeffectorTransform(endEffectorPose, mpcState);

  std::cout << "endEffector Transformation" <<std::endl;
  std::cout << endEffectorPose<<std::endl << std::endl;

  std::cout << "endEffector Goal" <<std::endl;
  std::cout << settings_.goal.getPosition().vector()<<std::endl << std::endl;
  //Old
  //  Eigen::Matrix4d endEffectorPose;
//  mabi_mobile_robcogen::MabiMobileTransforms<double>::computeState2EndeffectorTransform(
//          endEffectorPose, mpcState, settings_.transformBase_X_ArmMount, settings_.transformWrist2_X_Endeffector);

  double angularError =
      std::abs(settings_.goal.getRotation().toImplementation().angularDistance(Eigen::Quaterniond(endEffectorPose.block<3, 3>(0, 0))));
  double positionError = (endEffectorPose.block<3, 1>(0, 3) - settings_.goal.getPosition().toImplementation()).norm();

  return std::max(positionError / settings_.positionTolerance, angularError / settings_.angularTolerance);
}
