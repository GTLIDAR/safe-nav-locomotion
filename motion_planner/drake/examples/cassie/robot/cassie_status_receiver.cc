#include "drake/examples/cassie/robot/cassie_status_receiver.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace examples {
namespace cassie {

using systems::BasicVector;
using systems::Context;

CassieStatusReceiver::CassieStatusReceiver(int num_joints)
    : num_joints_(num_joints) {
  this->DeclareAbstractInputPort(
      "lcmt_cassie_status", Value<lcmt_cassie_status>{});
  this->DeclareVectorOutputPort(
      "position_commanded", BasicVector<double>(num_joints_),
      &CassieStatusReceiver::CalcLcmOutput<
        &lcmt_cassie_status::joint_position_commanded>);
  this->DeclareVectorOutputPort(
      "position_measured", BasicVector<double>(num_joints_),
      &CassieStatusReceiver::CalcLcmOutput<
        &lcmt_cassie_status::joint_position_measured>);
  this->DeclareVectorOutputPort(
      "velocity_estimated", BasicVector<double>(num_joints_),
      &CassieStatusReceiver::CalcLcmOutput<
        &lcmt_cassie_status::joint_velocity_estimated>);
  this->DeclareVectorOutputPort(
      "torque_commanded", BasicVector<double>(num_joints_),
      &CassieStatusReceiver::CalcLcmOutput<
        &lcmt_cassie_status::joint_torque_commanded>);
  this->DeclareVectorOutputPort(
      "torque_measured", BasicVector<double>(num_joints_),
      &CassieStatusReceiver::CalcLcmOutput<
        &lcmt_cassie_status::joint_torque_measured>);
  this->DeclareVectorOutputPort(
      "torque_external", BasicVector<double>(num_joints_),
      &CassieStatusReceiver::CalcLcmOutput<
        &lcmt_cassie_status::joint_torque_external>);
}

const systems::InputPort<double>& CassieStatusReceiver::get_input_port() const {
  return LeafSystem<double>::get_input_port(0);
}
using OutPort = systems::OutputPort<double>;
const OutPort& CassieStatusReceiver::get_position_commanded_output_port() const {
  return LeafSystem<double>::get_output_port(0);
}
const OutPort& CassieStatusReceiver::get_position_measured_output_port() const {
  return LeafSystem<double>::get_output_port(1);
}
const OutPort& CassieStatusReceiver::get_velocity_estimated_output_port() const {
  return LeafSystem<double>::get_output_port(2);
}
const OutPort& CassieStatusReceiver::get_torque_commanded_output_port() const {
  return LeafSystem<double>::get_output_port(3);
}
const OutPort& CassieStatusReceiver::get_torque_measured_output_port() const {
  return LeafSystem<double>::get_output_port(4);
}
const OutPort& CassieStatusReceiver::get_torque_external_output_port() const {
  return LeafSystem<double>::get_output_port(5);
}

template <std::vector<double> drake::lcmt_cassie_status::* field_ptr>
void CassieStatusReceiver::CalcLcmOutput(
    const Context<double>& context, BasicVector<double>* output) const {
  const auto& status = get_input_port().Eval<lcmt_cassie_status>(context);

  drake::log()->info("reading cassie status");
  for(int i = 0; i < status.num_joints; i++){
    drake::log()->info(status.joint_position_measured[i]);
  }
  drake::log()->info("end of status");

  // If we're using a default constructed message (i.e., we haven't received
  // any status message yet), output zero.
  if (status.num_joints == 0) {
    if(init_state_.size() == 0){
      Eigen::VectorXd init_state;
      init_state.resize(27);
      init_state[0] = 1; //Quaternion W = 1
      init_state[6] = 1.2; //Position Z = 1.2 meters
      output->SetFromVector(init_state);
    }
    else if(init_state_.size() == 7){
      Eigen::VectorXd init_state;
      init_state.resize(27);
      for(int i = 0; i < 7; i++){
        init_state[i] = init_state_[i];
      }
      output->SetFromVector(init_state); // Set only the base position (4q,3xyz)
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

void CassieStatusReceiver::SetInitialPosition(Eigen::VectorXd init_state){
  DRAKE_DEMAND(init_state.size() == num_joints_ || init_state.size() == 7);
  // Try to ensure a valid quaternion value
  DRAKE_DEMAND(!(init_state[0] == 0 && init_state[1] == 0 && 
                init_state[2] == 0 && init_state[3] == 0));
  init_state_ = init_state;
}

}  // namespace cassie
}  // namespace examples
}  // namespace drake
