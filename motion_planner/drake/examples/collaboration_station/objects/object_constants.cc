#include "drake/examples/collaboration_station/objects/object_constants.h"

namespace drake {
namespace examples {
namespace collaboration_station {

VectorX<double> get_object_max_joint_velocities() {
  // These are the maximum joint velocities given in Section 4.3.2 "Axis data,
  // LBR object 14 R820" of the "LBR object 7 R800, LBR object 14 R820 Specification".
  // That document is available here:
  // https://www.kuka.com/-/media/kuka-downloads/imported/48ec812b1b2947898ac2598aff70abc0/spez_lbr_object_en.pdf
  return (VectorX<double>(7) << 1.483529,  //  85°/s in rad/s
          1.483529,                        //  85°/s in rad/s
          1.745329,                        // 100°/s in rad/s
          1.308996,                        //  75°/s in rad/s
          2.268928,                        // 130°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194)

      .finished();
}

// This value is chosen to match the value in getSendPeriodMilliSec()
// when initializing the FRI configuration on the object's control
// cabinet.
const double kObjectLcmStatusPeriod = 0.005;

}  // namespace collaboration_station
}  // namespace examples
}  // namespace drake
