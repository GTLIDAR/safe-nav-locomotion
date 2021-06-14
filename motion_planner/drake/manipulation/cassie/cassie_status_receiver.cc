#include "drake/manipulation/cassie/cassie_status_receiver.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace manipulation {
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

  // If we're using a default constructed message (i.e., we haven't received
  // any status message yet), output zero.
  if (status.num_joints == 0) {
    output->get_mutable_value().setZero();
  } else {
    const auto& status_field = status.*field_ptr;
    // drake::log()->info("status joitns: {}\nnum_joints_: {}", status.num_joints, num_joints_);
    DRAKE_THROW_UNLESS(status.num_joints == num_joints_);
    DRAKE_THROW_UNLESS(static_cast<int>(status_field.size()) == num_joints_);
    output->get_mutable_value() = Eigen::Map<const Eigen::VectorXd>(
        status_field.data(), num_joints_);
  }
}

void CassieStatusReceiver::set_initial_position(
    Context<double>* context,
    const Eigen::Ref<const VectorX<double>>& x) const {
        drake::log()->info("comes here: set_initial_position in cassie_status_receiver.cc");
        drake::log()->info("joints: {}\nx size: {}", num_joints_, x.size());
        drake::log()->info("---");
        // drake::log()->info("size0: {}",context->get_mutable_numeric_parameter(0).size());
        // drake::log()->info("size1: {}",context->get_mutable_numeric_parameter(1).size());
    drake::log()->info(" contsize: {}", context->get_mutable_continuous_state().size());
    // DRAKE_THROW_UNLESS(x.size() == num_joints_);
    // context->get_mutable_numeric_parameter(0).SetFromVector(x);
    // context->get_mutable_numeric_parameter(1).size();

    drake::log()->info("passes by: set_initial_position");
    // auto state_value = context->get_mutable_discrete_state(0).get_mutable_value();
    // state_value.setZero();
    // state_value.head(num_joints_) = x;
    // state_value.tail(num_joints_ * 2) = VectorX<double>::Zero(num_joints_ * 2);
}

}  // namespace cassie
}  // namespace manipulation
}  // namespace drake
