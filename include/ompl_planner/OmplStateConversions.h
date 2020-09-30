//
// Created by johannes on 05.05.20.
//
#pragma once
//#include <ocs2_mobile_manipulator_interface/Conversions.h>
//#include <ocs2_mobile_manipulator_interface/Definitions.h>
//#include <ocs2_mobile_manipulator_interface/sampling_planner/StateSpace.h>

#include "ompl_planner/StateSpace.h"
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include "ompl_planner/Conversions.h"

#include <Eigen/Dense>
#include "kindr/poses/HomogeneousTransformation.hpp"
#include "perceptive_mpc/Definitions.h"

namespace perceptive_mpc {

inline Eigen::Quaterniond yawToEigenQuaternion(double yaw) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}
template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}
inline double eigenQuaternionToYaw(const Eigen::Quaterniond& eigenQuaternion) {
  Eigen::AngleAxisd aa;
  aa = eigenQuaternion;
  return aa.angle() * sgn(aa.axis()[2]);
}

inline void mpcToOmplState(const Eigen::VectorXd& mpcStateVector, MabiStateSpace::StateType* omplState) {
  using namespace ompl::base;

  kindr::HomTransformQuatD basePose;
  Eigen::VectorXd armState;
  Conversions::readMpcState(mpcStateVector, basePose, armState);

  omplState->basePoseState()->setYaw(eigenQuaternionToYaw(basePose.getRotation().toImplementation()));
  omplState->basePoseState()->setXY(basePose.getPosition().x(), basePose.getPosition().y());
  for (int i = 0; i < armState.size(); i++) omplState->armState()->values[i] = armState[i];
}

inline void omplToMpcState(const MabiStateSpace::StateType* omplState, Eigen::VectorXd& mpcStateVector) {
    using namespace ompl::base;

    mpcStateVector.resize(Definitions::STATE_DIM_);

    kindr::HomTransformQuatD basePose;
    Eigen::VectorXd armState((uint)Definitions::ARM_STATE_DIM_);

    basePose.getRotation().toImplementation() = yawToEigenQuaternion(omplState->basePoseState()->getYaw());
    basePose.getPosition().x() = omplState->basePoseState()->getX();
    basePose.getPosition().y() = omplState->basePoseState()->getY();
    basePose.getPosition().z() = 0;

    for (int i = 0; i < Definitions::ARM_STATE_DIM_; i++) armState[i] = omplState->armState()->values[i];

    Conversions::state_vector_t mpcStateFixedSize;
    Conversions::writeMpcState(mpcStateFixedSize, basePose, armState);
    mpcStateVector.head<Definitions::STATE_DIM_>() = mpcStateFixedSize;
}

inline void mpcToOmplControl(const Eigen::VectorXd& mpcControlVector, ompl::control::RealVectorControlSpace::ControlType* omplControl) {
  Eigen::VectorXd::Map(omplControl->values, Definitions::INPUT_DIM_) = mpcControlVector;
}

inline void omplToMpcControl(const ompl::control::RealVectorControlSpace::ControlType* omplControl, Eigen::VectorXd& mpcControlVector) {
  mpcControlVector.resize(Definitions::INPUT_DIM_);
  mpcControlVector = Eigen::VectorXd::Map(omplControl->values, Definitions::INPUT_DIM_);
}

}  // namespace ocs2_mm