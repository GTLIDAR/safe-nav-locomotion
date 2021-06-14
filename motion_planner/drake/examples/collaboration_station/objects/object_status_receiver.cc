#include "drake/examples/collaboration_station/objects/object_status_receiver.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace examples {
namespace collaboration_station {

using systems::BasicVector;
using systems::Context;

ObjectStatusReceiver::ObjectStatusReceiver(int num_joints)
    : num_joints_(num_joints) {
  this->DeclareAbstractInputPort(
      "lcmt_object_status", Value<lcmt_object_status>{});
  this->DeclareVectorOutputPort(
      "position_commanded", BasicVector<double>(num_joints_),
      &ObjectStatusReceiver::CalcLcmOutput<
        &lcmt_object_status::joint_position_commanded>);
  this->DeclareVectorOutputPort(
      "position_measured", BasicVector<double>(num_joints_),
      &ObjectStatusReceiver::CalcLcmOutput<
        &lcmt_object_status::joint_position_measured>);
  this->DeclareVectorOutputPort(
      "velocity_estimated", BasicVector<double>(num_joints_),
      &ObjectStatusReceiver::CalcLcmOutput<
        &lcmt_object_status::joint_velocity_estimated>);
  this->DeclareVectorOutputPort(
      "torque_commanded", BasicVector<double>(num_joints_),
      &ObjectStatusReceiver::CalcLcmOutput<
        &lcmt_object_status::joint_torque_commanded>);
  this->DeclareVectorOutputPort(
      "torque_measured", BasicVector<double>(num_joints_),
      &ObjectStatusReceiver::CalcLcmOutput<
        &lcmt_object_status::joint_torque_measured>);
  this->DeclareVectorOutputPort(
      "torque_external", BasicVector<double>(num_joints_),
      &ObjectStatusReceiver::CalcLcmOutput<
        &lcmt_object_status::joint_torque_external>);
}

const systems::InputPort<double>& ObjectStatusReceiver::get_input_port() const {
  return LeafSystem<double>::get_input_port(0);
}
using OutPort = systems::OutputPort<double>;
const OutPort& ObjectStatusReceiver::get_position_commanded_output_port() const {
  return LeafSystem<double>::get_output_port(0);
}
const OutPort& ObjectStatusReceiver::get_position_measured_output_port() const {
  return LeafSystem<double>::get_output_port(1);
}
const OutPort& ObjectStatusReceiver::get_velocity_estimated_output_port() const {
  return LeafSystem<double>::get_output_port(2);
}
const OutPort& ObjectStatusReceiver::get_torque_commanded_output_port() const {
  return LeafSystem<double>::get_output_port(3);
}
const OutPort& ObjectStatusReceiver::get_torque_measured_output_port() const {
  return LeafSystem<double>::get_output_port(4);
}
const OutPort& ObjectStatusReceiver::get_torque_external_output_port() const {
  return LeafSystem<double>::get_output_port(5);
}

template <std::vector<double> drake::lcmt_object_status::* field_ptr>
void ObjectStatusReceiver::CalcLcmOutput(
    const Context<double>& context, BasicVector<double>* output) const {
  const auto& status = get_input_port().Eval<lcmt_object_status>(context);

  // If we're using a default constructed message (i.e., we haven't received
  // any status message yet), output zero.
  if (status.num_joints == 0) {
    if(init_state_.size() == 0){
      Eigen::VectorXd init_state(7);
      init_state[0] = 1; //Quaternion W = 1
      output->SetFromVector(init_state);
    }
    else{
      output->SetFromVector(init_state_);
    }
  } else {
    const auto& status_field = status.*field_ptr;
    // drake::log()->info("status joitns: {}\nnum_joints_: {}", status.num_joints, num_joints_);
    DRAKE_THROW_UNLESS(status.num_joints == num_joints_);
    DRAKE_THROW_UNLESS(static_cast<int>(status_field.size()) == num_joints_);
    output->get_mutable_value() = Eigen::Map<const Eigen::VectorXd>(
        status_field.data(), num_joints_);
  }
}

void ObjectStatusReceiver::SetInitialPosition(Eigen::VectorXd init_state){
  // Have you changed the config file parameter: "num_objects"?
  DRAKE_DEMAND(init_state.size() == num_joints_);
  
  // Try to ensure a valid quaternion value
  const int num_objects = num_joints_ / 7;

  // This goes through every single object
  // Checks for bad quaternion values for initialized states
  for(int i = 0; i < num_objects; i++){
    DRAKE_DEMAND(!(init_state[i*7] == 0 && init_state[i*7+1] == 0 && 
                  init_state[i*7+2] == 0 && init_state[i*7+3] == 0));
  }

  init_state_ = init_state;

}

}  // namespace collaboration_station
}  // namespace examples
}  // namespace drake
