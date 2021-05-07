#pragma once

#include "drake/examples/cassie/robot/cassie_command_receiver.h"
#include "drake/examples/cassie/robot/cassie_command_sender.h"
#include "drake/examples/cassie/robot/cassie_constants.h"
#include "drake/examples/cassie/robot/cassie_status_receiver.h"
#include "drake/examples/cassie/robot/cassie_status_sender.h"

namespace drake {
namespace examples {
namespace cassie {

// These details have moved to files under drake/examples/kuka_cassie.
// These forwarding aliases are placed here for compatibility purposes.
using examples::cassie::CassieCommandReceiver;
using examples::cassie::CassieCommandSender;
using examples::cassie::CassieStatusReceiver;
using examples::cassie::CassieStatusSender;
using examples::cassie::kCassieLcmStatusPeriod;

}  // namespace cassie_arm
}  // namespace examples
}  // namespace drake
