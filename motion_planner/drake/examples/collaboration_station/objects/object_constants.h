#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace collaboration_station {

constexpr int kObjectNumJoints = 7;

/// Returns the maximum joint velocities provided by Kuka.
/// @return Maximum joint velocities (rad/s).
VectorX<double> get_object_max_joint_velocities();

extern const double kObjectLcmStatusPeriod;

}  // namespace collaboration_station
}  // namespace examples
}  // namespace drake
