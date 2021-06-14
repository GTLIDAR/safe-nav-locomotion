#include "drake/examples/mobile_robots/turtle_bot/robot/turtle_command_sender.h"

namespace drake {
namespace examples {
namespace mobile_robots {
namespace turtle_bot {

TurtleCommandSender::TurtleCommandSender(int num_joints)
    : num_joints_(num_joints) {
  this->DeclareInputPort(
      "position", systems::kVectorValued, num_joints_);
  this->DeclareInputPort(
      "torque", systems::kVectorValued, num_joints_);
  this->DeclareAbstractOutputPort(
      "lcmt_turtle_command", &TurtleCommandSender::CalcOutput);
}

using InPort = systems::InputPort<double>;
const InPort& TurtleCommandSender::get_position_input_port() const {
  return LeafSystem<double>::get_input_port(0);
}
const InPort& TurtleCommandSender::get_torque_input_port() const {
  return LeafSystem<double>::get_input_port(1);
}
const systems::OutputPort<double>& TurtleCommandSender::get_output_port() const {
  return LeafSystem<double>::get_output_port(0);
}

void TurtleCommandSender::CalcOutput(
    const systems::Context<double>& context, lcmt_turtle_command* output) const {
  const auto& position = get_position_input_port().Eval(context);
  const bool has_torque = get_torque_input_port().HasValue(context);
  const int num_torques = has_torque ? num_joints_ : 0;

  lcmt_turtle_command& command = *output;
  command.utime = context.get_time() * 1e6;
  command.num_joints = num_joints_;
  command.joint_position.resize(num_joints_);
  for (int i = 0; i < num_joints_; ++i) {
    command.joint_position[i] = position[i];
  }
  command.num_torques = num_torques;
  command.joint_torque.resize(num_torques);
  if (has_torque) {
    const auto& torque = get_torque_input_port().Eval(context);
    for (int i = 0; i < num_torques; ++i) {
      command.joint_torque[i] = torque[i];
    }
  }
}

}  // namespace turtle_bot
}  // namespace mobile_robots
}  // namespace examples
}  // namespace drake
