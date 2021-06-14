#pragma once

#include "drake/examples/quadrotor/robot/quadrotor_command_receiver.h"
#include "drake/examples/quadrotor/robot/quadrotor_command_sender.h"
#include "drake/examples/quadrotor/robot/quadrotor_constants.h"
#include "drake/examples/quadrotor/robot/quadrotor_status_receiver.h"
#include "drake/examples/quadrotor/robot/quadrotor_status_sender.h"

namespace drake {
namespace examples {
namespace quadrotor {

// These details have moved to files under drake/examples/mobile_robots/turtle_bot/robot.
// These forwarding aliases are placed here for compatibility purposes.
using examples::quadrotor::QuadrotorCommandReceiver;
using examples::quadrotor::QuadrotorCommandSender;
using examples::quadrotor::QuadrotorStatusReceiver;
using examples::quadrotor::QuadrotorStatusSender;
using examples::quadrotor::kQuadrotorLcmStatusPeriod;

}  // namespace quadrotor
}  // namespace examples
}  // namespace drake
