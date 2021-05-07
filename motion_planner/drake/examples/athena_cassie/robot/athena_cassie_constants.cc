#include "drake/examples/athena_cassie/robot/athena_cassie_constants.h"

namespace drake {
namespace examples {
namespace athena_cassie {

VectorX<double> get_athena_cassie_max_joint_velocities() {
  // These are the maximum joint velocities given in Section 4.3.2 "Axis data,
  // LBR cassie 14 R820" of the "LBR cassie 7 R800, LBR cassie 14 R820 Specification".
  // That document is available here:
  // https://www.kuka.com/-/media/kuka-downloads/imported/48ec812b1b2947898ac2598aff70abc0/spez_lbr_cassie_en.pdf

  return (VectorX<double>(82) << 1.483529,  //  85°/s in rad/s
          1.483529,                        //  85°/s in rad/s
          1.745329,                        // 100°/s in rad/s
          1.308996,                        //  75°/s in rad/s
          2.268928,                        // 130°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
                    1.308996,                        //  75°/s in rad/s
          2.268928,                        // 130°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
                    1.308996,                        //  75°/s in rad/s
          2.268928,                        // 130°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
                    1.308996,                        //  75°/s in rad/s
          2.268928,                        // 130°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,   
                    1.308996,                        //  75°/s in rad/s
          2.268928,                        // 130°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,   
                             1.308996,                        //  75°/s in rad/s
          2.268928,                        // 130°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,   
                             1.308996,                        //  75°/s in rad/s
          2.268928,                        // 130°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,   
                             1.308996,                        //  75°/s in rad/s
          2.268928,                        // 130°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,   
                             1.308996,                        //  75°/s in rad/s
          2.268928,                        // 130°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,   
                             1.308996,                        //  75°/s in rad/s
          2.268928,                        // 130°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,   
                             1.308996,                        //  75°/s in rad/s
          2.268928,                        // 130°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,   
                             1.308996,                        //  75°/s in rad/s
          2.268928,                        // 130°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,   
                             1.308996,                        //  75°/s in rad/s
          2.268928,                        // 130°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194,                        // 135°/s in rad/s
          2.356194)   
          
          
          
          
          
          
          
          .finished();
}

// This value is chosen to match the value in getSendPeriodMilliSec()
// when initializing the FRI configuration on the cassie's control
// cabinet.
const double kAthenaCassieLcmStatusPeriod = 0.005;

}  // namespace athena_cassie
}  // namespace examples
}  // namespace drake
