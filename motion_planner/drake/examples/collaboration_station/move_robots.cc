/// @file
///
/// Demo of moving the iiwa's end effector in cartesian space.  This
/// program creates a plan to move the end effector from the current
/// position to the location specified on the command line.  The
/// current calculated position of the end effector is printed before,
/// during, and after the commanded motion.

#include "lcm/lcm-cpp.hpp"
#include "robotlocomotion/robot_plan_t.hpp"
#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/examples/mobile_robot/turtle_bot/turtle_common.h"
#include "drake/lcmt_turtle_status.hpp"
#include "drake/manipulation/util/move_ik_demo_base.h"
#include "drake/math/rigid_transform.h"

DEFINE_string(urdf, "", "Name of urdf to load");
DEFINE_string(lcm_status_channel, "IIWA_STATUS",
              "Channel on which to listen for lcmt_iiwa_status messages.");
DEFINE_string(lcm_plan_channel, "COMMITTED_ROBOT_PLAN",
              "Channel on which to send robot_plan_t messages.");
DEFINE_double(x, 0., "x coordinate to move to");
DEFINE_double(y, 0., "y coordinate to move to");
DEFINE_double(z, 0., "z coordinate to move to");
DEFINE_double(roll, 0., "target roll about world x axis for end effector");
DEFINE_double(pitch, 0., "target pitch about world y axis for end effector");
DEFINE_double(yaw, 0., "target yaw about world z axis for end effector");
DEFINE_string(ee_name, "iiwa_link_ee", "Name of the end effector link");

namespace drake {
namespace examples {
namespace collaboration_station {
namespace {

using manipulation::util::MoveIkDemoBase;

const char kIiwaUrdf[] =
    "drake/examples/mobile_robots/turtle_bot/models/turtlebot.urdf";

class MoveRobots {
 public:
  /// plant is aliased
  explicit MoveTurtle() {

  }


  void Run() {
    math::RigidTransformd pose(
        math::RollPitchYawd(FLAGS_roll, FLAGS_pitch, FLAGS_yaw),
        Eigen::Vector3d(FLAGS_x, FLAGS_y, FLAGS_z));

    MoveIkDemoBase demo(
        !FLAGS_urdf.empty() ? FLAGS_urdf : FindResourceOrThrow(kIiwaUrdf),
        "base", FLAGS_ee_name, 100);
    demo.set_joint_velocity_limits(get_turtle_max_joint_velocities());

    ::lcm::LCM lc;
    while(lc.handle()>=0){ }
          Eigen::VectorXd turtle_q(status->num_joints);
          for (int i = 0; i < status->num_joints; i++) {
            turtle_q[i] = status->joint_position_measured[i];
          }
          demo.HandleStatus(turtle_q);
          if (demo.status_count() == 1) {
            std::optional<robotlocomotion::robot_plan_t> plan = demo.Plan(pose);
            if (plan.has_value()) {
              lc.publish(FLAGS_lcm_plan_channel, &plan.value());
            }
          }
        

    while (lc.handle() >= 0) { }
    return 0;
  }

private:

  void HandleStatus(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_turtle_status* status) {
    turtle_status_ = *status;
  }

  std::optional<robotlocomotion::robot_plan_t> MoveIkDemoBase::Plan(
      const math::RigidTransformd& goal_pose) {

    DRAKE_THROW_UNLESS(status_count_ > 0);

    // Create a single waypoint for our plan (the destination).
    // This results in a trajectory with two knot points (the
    // current pose (read from the status message currently being
    // processes and passed directly to PlanSequentialTrajectory as
    // iiwa_q) and the calculated final pose).
    ConstraintRelaxingIk::IkCartesianWaypoint wp;
    wp.pose = goal_pose;
    wp.constrain_orientation = true;
    std::vector<ConstraintRelaxingIk::IkCartesianWaypoint> waypoints;
    waypoints.push_back(wp);
    std::vector<Eigen::VectorXd> q_sol;
    const bool result =
        constraint_relaxing_ik_.PlanSequentialTrajectory(
            waypoints, plant_.GetPositions(*context_), &q_sol);
    drake::log()->info("IK result: {}", result);

    if (result) {
      drake::log()->info("IK sol size {}", q_sol.size());

      // Run the resulting plan over 2 seconds (which is a fairly
      // arbitrary choice).  This may be slowed down if executing
      // the plan in that time would exceed any joint velocity
      // limits.
      std::vector<double> times{0, 2};
      DRAKE_DEMAND(q_sol.size() == times.size());

      ApplyJointVelocityLimits(
          q_sol, joint_velocity_limits_, &times);
      std::vector<int> info{1, 1};
      robotlocomotion::robot_plan_t plan = EncodeKeyFrames(
          joint_names_, times, info, q_sol);
      return plan;
    }

    return std::nullopt;
  }





};

int do_main() {
  MoveRobots moveRobots();
  moveRobots.Run();
  return 0;
}



}  // namespace
}  // namespace collaboration_station
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::mobile_robots::turtle_bot::DoMain();
}
