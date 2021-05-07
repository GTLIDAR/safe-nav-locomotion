#include <limits>

#include <gflags/gflags.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/examples/collaboration_station/collaboration_station.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/lcmt_cassie_command.hpp"
#include "drake/lcmt_cassie_status.hpp"
#include "drake/manipulation/kuka_iiwa/iiwa_command_receiver.h"
#include "drake/manipulation/kuka_iiwa/iiwa_status_sender.h"
#include "drake/manipulation/cassie/cassie_command_receiver.h"
#include "drake/manipulation/cassie/cassie_status_sender.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/matrix_gain.h"



namespace drake {
namespace examples {
namespace collaboration_station {
namespace {
// Nameless namespace used for files with no dependence

using Eigen::VectorXd;


DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(duration, std::numeric_limits<double>::infinity(),
              "Simulation duration.");
DEFINE_string(setup, "manipulation_class",
              "Manipulation station type to simulate. "
              "Can be {manipulation_class, clutter_clearing}");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  systems::DiagramBuilder<double> builder;

  // Create the "collaboration station".
  auto station = builder.AddSystem<CollaborationStation>();
  station->SetupHomeEnvironmentV1();
  station->AddManipulandFromFile(
    "drake/examples/collaboration_station/models/061_foam_brick.sdf",
    math::RigidTransform<double>(math::RotationMatrix<double>::Identity(),
                                    Eigen::Vector3d(0.6, 0, 0)));
  station->Finalize();

  geometry::ConnectDrakeVisualizer(&builder, station->get_scene_graph(),
                                   station->GetOutputPort("pose_bundle"));

  auto lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();

  auto iiwa_command_subscriber = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_iiwa_command>(
          "IIWA_COMMAND", lcm));
  auto iiwa_command =
      builder.AddSystem<manipulation::kuka_iiwa::IiwaCommandReceiver>();
  builder.Connect(iiwa_command_subscriber->get_output_port(),
                  iiwa_command->get_message_input_port());
  builder.Connect(station->GetOutputPort("iiwa_position_measured"),
                  iiwa_command->get_position_measured_input_port());

  // Pull the positions out of the state.
  builder.Connect(iiwa_command->get_commanded_position_output_port(),
                  station->GetInputPort("iiwa_position"));
  builder.Connect(iiwa_command->get_commanded_torque_output_port(),
                  station->GetInputPort("iiwa_feedforward_torque"));

  auto iiwa_status =
      builder.AddSystem<manipulation::kuka_iiwa::IiwaStatusSender>();
  builder.Connect(station->GetOutputPort("iiwa_position_commanded"),
                  iiwa_status->get_position_commanded_input_port());
  builder.Connect(station->GetOutputPort("iiwa_position_measured"),
                  iiwa_status->get_position_measured_input_port());
  builder.Connect(station->GetOutputPort("iiwa_velocity_estimated"),
                  iiwa_status->get_velocity_estimated_input_port());
  builder.Connect(station->GetOutputPort("iiwa_torque_commanded"),
                  iiwa_status->get_torque_commanded_input_port());
  builder.Connect(station->GetOutputPort("iiwa_torque_measured"),
                  iiwa_status->get_torque_measured_input_port());
  builder.Connect(station->GetOutputPort("iiwa_torque_external"),
                  iiwa_status->get_torque_external_input_port());
  auto iiwa_status_publisher = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<drake::lcmt_iiwa_status>(
          "IIWA_STATUS", lcm, 0.005 /* publish period */));
  builder.Connect(iiwa_status->get_output_port(),
                  iiwa_status_publisher->get_input_port());

  // Receive the WSG commands.
  auto wsg_command_subscriber = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_schunk_wsg_command>(
          "SCHUNK_WSG_COMMAND", lcm));
  auto wsg_command =
      builder.AddSystem<manipulation::schunk_wsg::SchunkWsgCommandReceiver>();
  builder.Connect(wsg_command_subscriber->get_output_port(),
                  wsg_command->GetInputPort("command_message"));
  builder.Connect(wsg_command->get_position_output_port(),
                  station->GetInputPort("wsg_position"));
  builder.Connect(wsg_command->get_force_limit_output_port(),
                  station->GetInputPort("wsg_force_limit"));

//   Publish the WSG status.
  auto wsg_status =
      builder.AddSystem<manipulation::schunk_wsg::SchunkWsgStatusSender>();
  builder.Connect(station->GetOutputPort("wsg_state_measured"),
                  wsg_status->get_state_input_port());
  builder.Connect(station->GetOutputPort("wsg_force_measured"),
                  wsg_status->get_force_input_port());
  auto wsg_status_publisher = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<drake::lcmt_schunk_wsg_status>(
          "SCHUNK_WSG_STATUS", lcm, 0.05 /* publish period */));
  builder.Connect(wsg_status->get_output_port(0),
                  wsg_status_publisher->get_input_port());


//   auto cassie_command_subscriber = builder.AddSystem(
//       systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_cassie_command>(
//           "CASSIE_COMMAND", lcm));
//   auto cassie_command =
//       builder.AddSystem<manipulation::cassie::CassieCommandReceiver>();
//   builder.Connect(cassie_command_subscriber->get_output_port(),
//                   cassie_command->get_message_input_port());
//   builder.Connect(station->GetOutputPort("cassie_position_measured"),
//                   cassie_command->get_position_measured_input_port());

//   // Pull the positions out of the state.
//   builder.Connect(cassie_command->get_commanded_position_output_port(),
//                   station->GetInputPort("cassie_position"));
//   builder.Connect(cassie_command->get_commanded_torque_output_port(),
//                   station->GetInputPort("cassie_feedforward_torque"));

//   auto cassie_status =
//       builder.AddSystem<manipulation::cassie::CassieStatusSender>();
//   builder.Connect(station->GetOutputPort("cassie_position_commanded"),
//                   cassie_status->get_position_commanded_input_port());
//   builder.Connect(station->GetOutputPort("cassie_position_measured"),
//                   cassie_status->get_position_measured_input_port());
//   builder.Connect(station->GetOutputPort("cassie_velocity_estimated"),
//                   cassie_status->get_velocity_estimated_input_port());




  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.AdvanceTo(FLAGS_duration);

  return 0;
}


}  // namespace
}  // namespace collaboration_station
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::collaboration_station::do_main(argc, argv);
}
