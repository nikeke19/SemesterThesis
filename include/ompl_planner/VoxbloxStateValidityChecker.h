//
// Created by johannes on 05.05.20.
//

#pragma once

#include "perceptive_mpc/costs/VoxbloxCost.h"
#include "ompl_planner/StateSpace.h"
#include "perceptive_mpc/Definitions.h"
#include "ompl_planner/StateSpace.h"
#include "ompl_planner/OmplStateConversions.h" //@todo See if needed

#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ompl/base/StateValidityChecker.h>
#include <mutex>

#include "kindr/poses/HomogeneousTransformation.hpp"
namespace perceptive_mpc {
class VoxbloxCost;
class VoxbloxCostConfig;

class VoxbloxStateValidityChecker : public ompl::base::StateValidityChecker {
 public:
  using State = ompl::base::State;
  using SpaceInformation = ompl::base::SpaceInformation;
  VoxbloxStateValidityChecker(SpaceInformation *si, std::shared_ptr<VoxbloxCostConfig> voxbloxCostConfig);
  bool isValid(const State* state) const override;
  double clearance(const State* state) const override;
  ~VoxbloxStateValidityChecker() override;

 private:
  std::unique_ptr<VoxbloxCost> voxbloxCost_;
  mutable std::mutex costMutex_;
};
}  // namespace ocs2_mm