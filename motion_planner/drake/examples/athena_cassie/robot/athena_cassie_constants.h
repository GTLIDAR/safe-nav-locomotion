#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace athena_cassie {

constexpr int kAthenaCassieNumJoints = 82;

/// Returns the maximum joint velocities provided by Kuka.
/// @return Maximum joint velocities (rad/s).
VectorX<double> get_athena_cassie_max_joint_velocities();

extern const double kAthenaCassieLcmStatusPeriod;

}  // namespace athena_cassie
}  // namespace examples
}  // namespace drake
