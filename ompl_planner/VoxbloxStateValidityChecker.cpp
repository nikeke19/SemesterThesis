//
// Created by johannes on 05.05.20.
//
#include "ompl_planner/VoxbloxStateValidityChecker.h"


//#include <ocs2_mobile_manipulator_interface/costs/VoxbloxCost.h>
//#include <ocs2_mobile_manipulator_interface/sampling_planner/OmplStateConversions.h>
//#include <ocs2_mobile_manipulator_interface/sampling_planner/StateSpace.h>
//#include <ocs2_mobile_manipulator_interface/sampling_planner/VoxbloxStateValidityChecker.h>
using namespace perceptive_mpc;
using namespace ompl::base;

VoxbloxStateValidityChecker::VoxbloxStateValidityChecker(SpaceInformation* si, std::shared_ptr<VoxbloxCostConfig> voxbloxCostConfig)
    : ompl::base::StateValidityChecker(si), voxbloxCost_(std::make_unique<VoxbloxCost>(*voxbloxCostConfig)) {

}

VoxbloxStateValidityChecker::~VoxbloxStateValidityChecker() = default;

bool VoxbloxStateValidityChecker::isValid(const State* state) const {
  double distance = clearance(state);
  //std::cout << distance << std::endl; //todo debug, delivers always the same clearance
  return distance > 0;
}

double VoxbloxStateValidityChecker::clearance(const State* state) const {
  Eigen::VectorXd mpcState;
  omplToMpcState(state->as<MabiState>(), mpcState);
  //std::cout << mpcState << std::endl << std::endl; //todo, seems okay, always different mpc state
  std::lock_guard<std::mutex> costLock(costMutex_);
  voxbloxCost_->setCurrentStateAndControl(0, mpcState.head<Definitions::STATE_DIM_>(), VoxbloxCost::input_vector_t::Zero());
  return voxbloxCost_->getMinDistance();
}



