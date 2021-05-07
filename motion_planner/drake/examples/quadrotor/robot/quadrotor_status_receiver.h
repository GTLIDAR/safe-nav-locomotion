#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/lcmt_quadrotor_status.hpp"
#include "drake/examples/quadrotor/robot/quadrotor_constants.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace quadrotor {

/// Handles lcmt_quadrotor_status messages from a LcmSubscriberSystem.
///
/// Note that this system does not actually subscribe to an LCM channel. To
/// receive the message, the input of this system should be connected to a
/// systems::lcm::LcmSubscriberSystem::Make<lcmt_quadrotor_status>().
///
/// This system has one abstract-valued input port of type lcmt_quadrotor_status.
///
/// This system has many vector-valued output ports, each of which has exactly
/// num_joints elements.  The ports will output zeros until an input message is
/// received.
//
/// @system{ QuadrotorStatusReceiver,
///   @input_port{lcmt_quadrotor_status},
///   @output_port{position_commanded}
///   @output_port{position_measured}
///   @output_port{velocity_estimated}
///   @output_port{torque_commanded}
///   @output_port{torque_measured}
///   @output_port{torque_external}
/// }
/// @see `lcmt_quadrotor_status.lcm` for additional documentation.
class QuadrotorStatusReceiver : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QuadrotorStatusReceiver)

  explicit QuadrotorStatusReceiver(int num_joints = kQuadrotorNumJoints);

  /// @name Named accessors for this System's input and output ports.
  //@{
  const systems::InputPort<double>& get_input_port() const;
  const systems::OutputPort<double>& get_position_commanded_output_port() const;
  const systems::OutputPort<double>& get_position_measured_output_port() const;
  const systems::OutputPort<double>& get_velocity_estimated_output_port() const;
  const systems::OutputPort<double>& get_torque_commanded_output_port() const;
  const systems::OutputPort<double>& get_torque_measured_output_port() const;
  const systems::OutputPort<double>& get_torque_external_output_port() const;
  //@}
  void SetInitialPosition(Eigen::VectorXd init_state);

 private:
  template <std::vector<double> drake::lcmt_quadrotor_status::*>
  void CalcLcmOutput(const systems::Context<double>&,
                     systems::BasicVector<double>*) const;
                     
  Eigen::VectorXd init_state_;
  const int num_joints_;
};

}  // namespace quadrotor
}  // namespace examples
}  // namespace drake
