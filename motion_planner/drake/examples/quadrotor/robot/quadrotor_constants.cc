#include "drake/examples/quadrotor/robot/quadrotor_constants.h"

namespace drake {
namespace examples {
namespace quadrotor {

VectorX<double> get_quadrotor_max_joint_velocities() {
  // These are the maximum joint velocities given in Section 4.3.2 "Axis data,
  // LBR quadrotor 14 R820" of the "LBR quadrotor 7 R800, LBR quadrotor 14 R820 Specification".
  // That document is available here:
  // https://www.kuka.com/-/media/kuka-downloads/imported/48ec812b1b2947898ac2598aff70abc0/spez_lbr_quadrotor_en.pdf
  return (VectorX<double>(7) << 1.483529,  //  85°/s in rad/s
          1.483529,
          1.483529,
          1.483529,
          1.483529,
          1.483529,
          1.483529)                        // 135°/s in rad/s

      .finished();
}

// This value is chosen to match the value in getSendPeriodMilliSec()
// when initializing the FRI configuration on the quadrotor's control
// cabinet.
const double kQuadrotorLcmStatusPeriod = 0.005;

}  // namespace quadrotor
}  // namespace examples
}  // namespace drake
