#include "drake/examples/mobile_robots/turtle_bot/robot/turtle_constants.h"

namespace drake {
namespace examples {
namespace mobile_robots {
namespace turtle_bot {

VectorX<double> get_turtle_max_joint_velocities() {
  // These are the maximum joint velocities given in Section 4.3.2 "Axis data,
  // LBR turtle 14 R820" of the "LBR turtle 7 R800, LBR turtle 14 R820 Specification".
  // That document is available here:
  // https://www.kuka.com/-/media/kuka-downloads/imported/48ec812b1b2947898ac2598aff70abc0/spez_lbr_turtle_en.pdf
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
// when initializing the FRI configuration on the turtle's control
// cabinet.
const double kTurtleLcmStatusPeriod = 0.005;

}  // namespace turtle_bot
}  // namespace mobile_robots
}  // namespace examples
}  // namespace drake
