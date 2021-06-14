#pragma once

#include "drake/examples/mobile_robots/turtle_bot/robot/turtle_command_receiver.h"
#include "drake/examples/mobile_robots/turtle_bot/robot/turtle_command_sender.h"
#include "drake/examples/mobile_robots//turtle_bot/robot/turtle_constants.h"
#include "drake/examples/mobile_robots/turtle_bot/robot/turtle_status_receiver.h"
#include "drake/examples/mobile_robots/turtle_bot/robot/turtle_status_sender.h"

namespace drake {
namespace examples {
namespace mobile_robots {
namespace turtle_bot {

// These details have moved to files under drake/examples/mobile_robots/turtle_bot/robot.
// These forwarding aliases are placed here for compatibility purposes.
using examples::mobile_robots::turtle_bot::TurtleCommandReceiver;
using examples::mobile_robots::turtle_bot::TurtleCommandSender;
using examples::mobile_robots::turtle_bot::TurtleStatusReceiver;
using examples::mobile_robots::turtle_bot::TurtleStatusSender;
using examples::mobile_robots::turtle_bot::kTurtleLcmStatusPeriod;

}  // namespace turtle_bot
}  // namespace mobile_robots
}  // namespace examples
}  // namespace drake
