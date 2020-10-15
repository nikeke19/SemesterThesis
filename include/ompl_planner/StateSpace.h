//
// Created by johannes on 05.05.20.
//

#pragma once

//#include "ocs2_mobile_manipulator_interface/Definitions.h"
#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/spaces/ReedsSheppStateSpace.h"

namespace perceptive_mpc {
using namespace ompl::base;
const int ARM_STATE_DIM = 6;

class MabiState : public CompoundState {
 public:
  const SE2StateSpace::StateType* basePoseState() const { return static_cast<const SE2StateSpace::StateType*>(components[0]); }

  SE2StateSpace::StateType* basePoseState() { return static_cast<SE2StateSpace::StateType*>(components[0]); }

  const RealVectorStateSpace::StateType* armState() const { return static_cast<const RealVectorStateSpace::StateType*>(components[1]); }

  RealVectorStateSpace::StateType* armState() { return static_cast<RealVectorStateSpace::StateType*>(components[1]); }
};

class MabiStateSpace : public CompoundStateSpace {
 public:
  /** \brief Define the type of state allocated by this state space */
  typedef MabiState StateType;

  MabiStateSpace() {
//    addSubspace(StateSpacePtr(new SE2StateSpace()), 1.0);
    addSubspace(StateSpacePtr(new ReedsSheppStateSpace(0.15)), 1.0);
    addSubspace(StateSpacePtr(new RealVectorStateSpace(ARM_STATE_DIM)), 1.0);
  }
  const std::shared_ptr<SE2StateSpace> basePoseStateSpace() { return std::static_pointer_cast<SE2StateSpace>(getSubspace(0)); }
  const std::shared_ptr<RealVectorStateSpace> armStateSpace() { return std::static_pointer_cast<RealVectorStateSpace>(getSubspace(1)); }
};
}  // namespace ocs2_mm