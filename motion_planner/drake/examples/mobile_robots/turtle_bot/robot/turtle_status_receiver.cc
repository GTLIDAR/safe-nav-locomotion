#include "drake/examples/mobile_robots/turtle_bot/robot/turtle_status_receiver.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace examples {
namespace mobile_robots {
namespace turtle_bot {

using systems::BasicVector;
using systems::Context;

TurtleStatusReceiver::TurtleStatusReceiver(int num_joints)
    : num_joints_(num_joints) {
  this->DeclareAbstractInputPort(
      "lcmt_turtle_status", Value<lcmt_turtle_status>{});
  this->DeclareVectorOutputPort(
      "position_commanded", BasicVector<double>(num_joints_),
      &TurtleStatusReceiver::CalcLcmOutput<
        &lcmt_turtle_status::joint_position_commanded>);
  this->DeclareVectorOutputPort(
      "position_measured", BasicVector<double>(num_joints_),
      &TurtleStatusReceiver::CalcLcmOutput<
        &lcmt_turtle_status::joint_position_measured>);
  this->DeclareVectorOutputPort(
      "velocity_estimated", BasicVector<double>(num_joints_),
      &TurtleStatusReceiver::CalcLcmOutput<
        &lcmt_turtle_status::joint_velocity_estimated>);
  this->DeclareVectorOutputPort(
      "torque_commanded", BasicVector<double>(num_joints_),
      &TurtleStatusReceiver::CalcLcmOutput<
        &lcmt_turtle_status::joint_torque_commanded>);
  this->DeclareVectorOutputPort(
      "torque_measured", BasicVector<double>(num_joints_),
      &TurtleStatusReceiver::CalcLcmOutput<
        &lcmt_turtle_status::joint_torque_measured>);
  this->DeclareVectorOutputPort(
      "torque_external", BasicVector<double>(num_joints_),
      &TurtleStatusReceiver::CalcLcmOutput<
        &lcmt_turtle_status::joint_torque_external>);
}

const systems::InputPort<double>& TurtleStatusReceiver::get_input_port() const {
  return LeafSystem<double>::get_input_port(0);
}
using OutPort = systems::OutputPort<double>;
const OutPort& TurtleStatusReceiver::get_position_commanded_output_port() const {
  return LeafSystem<double>::get_output_port(0);
}
const OutPort& TurtleStatusReceiver::get_position_measured_output_port() const {
  return LeafSystem<double>::get_output_port(1);
}
const OutPort& TurtleStatusReceiver::get_velocity_estimated_output_port() const {
  return LeafSystem<double>::get_output_port(2);
}
const OutPort& TurtleStatusReceiver::get_torque_commanded_output_port() const {
  return LeafSystem<double>::get_output_port(3);
}
const OutPort& TurtleStatusReceiver::get_torque_measured_output_port() const {
  return LeafSystem<double>::get_output_port(4);
}
const OutPort& TurtleStatusReceiver::get_torque_external_output_port() const {
  return LeafSystem<double>::get_output_port(5);
}

template <std::vector<double> drake::lcmt_turtle_status::* field_ptr>
void TurtleStatusReceiver::CalcLcmOutput(
    const Context<double>& context, BasicVector<double>* output) const {
  const auto& status = get_input_port().Eval<lcmt_turtle_status>(context);

  // If we're using a default constructed message (i.e., we haven't received
  // any status message yet), output zero.
  if (status.num_joints == 0) {
    if(init_state_.size() == 0){
      Eigen::VectorXd init_state(num_joints_);
      init_state.setZero();
      init_state[0] = 1; //Quaternion w = 1
      output->SetFromVector(init_state);
    }
    else{
      output->SetFromVector(init_state_); // Set everything
    }
  } else {
    const auto& status_field = status.*field_ptr;
    DRAKE_THROW_UNLESS(status.num_joints == num_joints_);
    DRAKE_THROW_UNLESS(static_cast<int>(status_field.size()) == num_joints_);
    output->get_mutable_value() = Eigen::Map<const Eigen::VectorXd>(
        status_field.data(), num_joints_);
  }
}


void TurtleStatusReceiver::SetInitialPosition(Eigen::VectorXd init_state){

  drake::log()->info("ini: {} {}", init_state.size(), num_joints_);
  DRAKE_DEMAND(init_state.size() == num_joints_);
  // Try to ensure a valid quaternion value
  DRAKE_DEMAND(!(init_state[0] == 0 && init_state[1] == 0 && 
                init_state[2] == 0 && init_state[3] == 0));
  init_state_ = init_state;
}

}  // namespace turtle_bot
}  // namespace mobile_robots
}  // namespace examples
}  // namespace drake
