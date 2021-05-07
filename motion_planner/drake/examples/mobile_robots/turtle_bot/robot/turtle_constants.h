#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace mobile_robots {
namespace turtle_bot {

constexpr int kTurtleNumJoints = 7;

/// Returns the maximum joint velocities provided by Kuka.
/// @return Maximum joint velocities (rad/s).
VectorX<double> get_turtle_max_joint_velocities();

extern const double kTurtleLcmStatusPeriod;

}  // namespace turtle_bot
}  // namespace mobile_robots
}  // namespace examples
}  // namespace drake
