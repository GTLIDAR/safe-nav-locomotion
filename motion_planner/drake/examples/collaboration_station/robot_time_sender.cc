#include "drake/examples/collaboration_station/robot_time_sender.h"

namespace drake {
namespace examples {
namespace collaboration_station {

RobotTimeSender::RobotTimeSender()
{
  this->DeclareAbstractOutputPort(
      "lcmt_robot_time", &RobotTimeSender::CalcOutput);
}

const systems::OutputPort<double>& RobotTimeSender::get_output_port() const {
  return LeafSystem<double>::get_output_port(0);
}

void RobotTimeSender::CalcOutput(
    const systems::Context<double>& context, lcmt_robot_time* output) const {

  lcmt_robot_time& robot_time = *output;
  robot_time.utime = context.get_time() * 1e6;
}

}  // namespace collaboration_station
}  // namespace examples
}  // namespace drake
