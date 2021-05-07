#include <gflags/gflags.h>
#include <limits>

#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"



namespace drake {
namespace examples {
namespace collaboration_station {
namespace {
// Nameless namespace used for files with no dependence

using drake::math::RigidTransformd;
using drake::multibody::MultibodyPlant;
using Eigen::Translation3d;
using Eigen::VectorXd;

DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(duration, std::numeric_limits<double>::infinity(),
              "Simulation duration.");

DEFINE_double(initial_height, 0.051, "Initial height of Cassie.");


template <typename T>
class Cassie : public systems::Diagram<T> {
 public:
  Cassie(){
    this->set_name("Cassie");

    systems::DiagramBuilder<double> builder;

    // auto lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();

    // auto cassie_command_subscriber = builder.AddSystem(
    //     systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_cassie_command>(
    //         "CASSIE_COMMAND", lcm));

    auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);
    geometry::ConnectDrakeVisualizer(&builder, scene_graph);
      multibody::Parser parser(&plant);
    parser.AddModelFromFile(
        FindResourceOrThrow(model_path_));

    parser.AddModelFromFile(
        FindResourceOrThrow("drake/examples/collaboration_station/models/"
        "home_environment_v1.sdf"));
    plant.Finalize();
    drake::log()->info("num act: {}\nnum_pos: {}\nnum vel: {}", 
      plant.num_actuators(),plant.num_positions(),plant.num_velocities());

    DRAKE_DEMAND(plant.num_actuators() == 10);
    DRAKE_DEMAND(plant.num_positions() == 28);


    // const VectorX<double> constant_vector = 
    //   VectorX<double>::Zero(plant.num_actuators());

    // drake::log()->info("Comes here2");

    // auto constant_zero_source =
    //   builder.AddSystem<systems::ConstantVectorSource<double>>(
    //   constant_vector);
    // drake::log()->info("Comes here3");

    // constant_zero_source->set_name("zero input");
    // drake::log()->info("Comes here4");

    // builder.Connect(constant_zero_source->get_output_port(), 
    //                 plant.get_actuation_input_port());
    // drake::log()->info("Comes here5");

    auto diagram = builder.Build();

    // Create a context for this system:
    std::unique_ptr<systems::Context<double>> diagram_context =
        diagram->CreateDefaultContext();
    systems::Context<double>& plant_context =
        diagram->GetMutableSubsystemContext(plant, diagram_context.get());

    const VectorXd tau = VectorXd::Ones(plant.num_actuated_dofs());
    plant.get_actuation_input_port().FixValue(&plant_context, tau);

    // Set the pelvis frame P initial pose.
    const Translation3d X_WP(0.0, 0.0, 0.95);

    const drake::multibody::Body<double>& pelvis = plant.GetBodyByName("pelvis");
    DRAKE_DEMAND(pelvis.is_floating());
    DRAKE_DEMAND(pelvis.has_quaternion_dofs());
    DRAKE_DEMAND(pelvis.floating_positions_start() == 0);
    DRAKE_DEMAND(pelvis.floating_velocities_start() == plant.num_positions());

    plant.SetFreeBodyPoseInWorldFrame(&plant_context, pelvis, X_WP);

    plant_ = &plant;

    auto simulator =
        MakeSimulatorFromGflags(*diagram, std::move(diagram_context));
    simulator->AdvanceTo(FLAGS_duration);

    drake::log()->info("Comes here6");


  }

  // void SetDefaultState(const systems::Context<T>& context,
  //                      systems::State<T>* state) const override {
  //   DRAKE_DEMAND(state != nullptr);
  //   systems::Diagram<T>::SetDefaultState(context, state);
  //   const systems::Context<T>& plant_context =
  //       this->GetSubsystemContext(*plant_, context);
  //   systems::State<T>& plant_state =
  //       this->GetMutableSubsystemState(*plant_, state);
  //   const math::RigidTransform<T> X_WB(
  //       Vector3<T>{0.0, 0.0, FLAGS_initial_height});
  //   plant_->SetFreeBodyPose(
  //       plant_context, &plant_state, plant_->GetBodyByName(joint_connection_), X_WB);
  // }

 private:
  multibody::MultibodyPlant<T>* plant_{};
  const std::string model_path_ = "drake/manipulation/models/cassie_description/urdf/cassie.urdf";
  const std::string joint_connection_ = "pelvis";

};

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  const Cassie<double> model;
  // auto simulator = MakeSimulatorFromGflags(model);
  // simulator->AdvanceTo(FLAGS_duration);








  return 0;
}


}  // namespace
}  // namespace collaboration_station
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::collaboration_station::do_main(argc, argv);
}
