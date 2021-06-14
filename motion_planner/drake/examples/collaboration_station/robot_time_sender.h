#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/lcmt_robot_time.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace collaboration_station {

/// Creates and outputs lcmt_robot_time messages.
///
/// Note that this system does not actually send the message an LCM channel. To
/// send the message, the output of this system should be connected to a
/// systems::lcm::LcmPublisherSystem::Make<lcmt_robot_time>().
///
/// This system has one abstract-valued output port of type lcmt_robot_time.
///
/// @system{RobotTimeSender,
///   @output_port{lcmt_robot_time}
/// }
///
/// @see `lcmt_robot_time.lcm` for additional documentation.
class RobotTimeSender : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RobotTimeSender)

  explicit RobotTimeSender();

  /// @name Named accessors for this System's input and output ports.
  //@{
  const systems::OutputPort<double>& get_output_port() const;
  //@}

 private:
  void CalcOutput(const systems::Context<double>&, lcmt_robot_time*) const;

};

}  // namespace collaboration_station
}  // namespace examples
}  // namespace drake
