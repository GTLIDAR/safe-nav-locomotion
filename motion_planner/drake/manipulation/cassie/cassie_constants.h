#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace manipulation {
namespace cassie {

constexpr int kCassieNumJoints = 10;

/// Returns the maximum joint velocities provided by Kuka.
/// @return Maximum joint velocities (rad/s).
VectorX<double> get_cassie_max_joint_velocities();

extern const double kCassieLcmStatusPeriod;

}  // namespace cassie
}  // namespace manipulation
}  // namespace drake
