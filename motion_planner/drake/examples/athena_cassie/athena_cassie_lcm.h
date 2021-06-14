#pragma once

// #include "drake/examples/athena_cassie/robot/athena_cassie_command_receiver.h"
// #include "drake/examples/athena_cassie/robot/athena_cassie_command_sender.h"
#include "drake/examples/athena_cassie/robot/athena_cassie_constants.h"
#include "drake/examples/athena_cassie/robot/athena_cassie_status_receiver.h"
#include "drake/examples/athena_cassie/robot/athena_cassie_status_sender.h"

namespace drake {
namespace examples {
namespace athena_cassie {

// These details have moved to files under drake/examples/kuka_cassie.
// These forwarding aliases are placed here for compatibility purposes.
// using examples::cassie::CassieCommandReceiver;
// using examples::cassie::CassieCommandSender;
using examples::athena_cassie::AthenaCassieStatusReceiver;
using examples::athena_cassie::AthenaCassieStatusSender;
using examples::athena_cassie::kAthenaCassieLcmStatusPeriod;

}  // namespace athena_cassie
}  // namespace examples
}  // namespace drake
