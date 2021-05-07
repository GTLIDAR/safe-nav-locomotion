#pragma once

#include <string>
#include <vector>

#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/examples/athena_cassie/robot/athena_cassie_constants.h"

namespace drake {
namespace examples {
namespace athena_cassie {

// These details have moved to files under drake/manipulation/athena_cassie.
// These forwarding aliases are placed here for compatibility purposes.
using examples::athena_cassie::kAthenaCassieNumJoints;
using examples::athena_cassie::get_athena_cassie_max_joint_velocities;

/// Used to set the feedback gains for the simulated position controlled KUKA.
void SetPositionControlledAthenaCassieGains(Eigen::VectorXd* Kp,
                                    Eigen::VectorXd* Ki,
                                    Eigen::VectorXd* Kd);

/// Used to set the feedback gains for the simulated torque controlled KUKA.
void SetTorqueControlledAthenaCassieGains(Eigen::VectorXd* stiffness,
                                  Eigen::VectorXd* damping_ratio);

/// Scales a plan so that no step exceeds the robot's maximum joint velocities.
/// The number of columns in @p keyframes must match the size of @p time.  Times
/// must be in strictly increasing order.
/// @see get_athena_cassie_max_joint_velocities
DRAKE_DEPRECATED("2020-07-01",
                 "This function is being moved to manipulation::util.")
void ApplyJointVelocityLimits(const MatrixX<double>& keyframes,
                              std::vector<double>* time);

/// Makes a robotlocomotion::robot_plan_t message.  The number of rows in @p
/// keyframes must match the size of @p joint_names.  The number of columns in
/// @p keyframes must match the size of @p time.  Times must be in strictly
/// increasing order.
DRAKE_DEPRECATED("2020-07-01",
                 "This function is being moved to manipulation::util.")
robotlocomotion::robot_plan_t EncodeKeyFrames(
    const std::vector<std::string>& joint_names,
    const std::vector<double>& time, const std::vector<int>& info,
    const MatrixX<double>& keyframes);

}  // namespace athena_cassie
}  // namespace examples
}  // namespace drake