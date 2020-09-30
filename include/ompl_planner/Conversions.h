//
// Created by johannes on 13.06.19.
//

#pragma once

#include "perceptive_mpc/ExplicitTemplateInstantiations.h"
#include "perceptive_mpc/Definitions.h"
#include <kindr/Core>
#include <Eigen/Core>

namespace perceptive_mpc {

class Conversions {
 public:
  typedef ocs2::SystemObservation<Definitions::STATE_DIM_, Definitions::INPUT_DIM_> Observation;
  typedef Observation::state_vector_t state_vector_t;
  typedef Observation::input_vector_t input_vector_t;

  static void writeMpcState(state_vector_t& stateVector, const kindr::HomTransformQuatD& pose, const Eigen::VectorXd& armPositions);
  static void readMpcState(const state_vector_t& stateVector, kindr::HomTransformQuatD& pose, Eigen::VectorXd& armPositions);

  static void readMpcInput(const input_vector_t& inputVector, kindr::TwistLocalD& twist, Eigen::VectorXd& armVelocities);

};

}  // namespace smb_path_following