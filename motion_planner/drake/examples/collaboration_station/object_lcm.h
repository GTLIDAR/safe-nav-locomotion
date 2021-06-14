#pragma once

// #include "drake/examples/collaboration_station/objects/object_command_receiver.h"
// #include "drake/examples/collaboration_station/objects/object_command_sender.h"
#include "drake/examples/collaboration_station/objects/object_constants.h"
#include "drake/examples/collaboration_station/objects/object_status_receiver.h"
#include "drake/examples/collaboration_station/objects/object_status_sender.h"

namespace drake {
namespace examples {
namespace collaboration_station {

// These details have moved to files under drake/examples/kuka_object.
// These forwarding aliases are placed here for compatibility purposes.
// using examples::collaboration_station::ObjectCommandReceiver;
// using examples::collaboration_station::ObjectCommandSender;
using examples::collaboration_station::ObjectStatusReceiver;
using examples::collaboration_station::ObjectStatusSender;
using examples::collaboration_station::kObjectLcmStatusPeriod;

}  // namespace collaboration_station
}  // namespace examples
}  // namespace drake
