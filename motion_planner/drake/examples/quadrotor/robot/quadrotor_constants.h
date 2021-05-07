#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace quadrotor {

constexpr int kQuadrotorNumJoints = 7;

/// Returns the maximum joint velocities provided by Kuka.
/// @return Maximum joint velocities (rad/s).
VectorX<double> get_quadrotor_max_joint_velocities();

extern const double kQuadrotorLcmStatusPeriod;

}  // namespace quadrotor
}  // namespace examples
}  // namespace drake
