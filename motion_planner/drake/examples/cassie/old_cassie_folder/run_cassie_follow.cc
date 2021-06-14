#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

#include "drake/examples/cassie/cassie.h"
DEFINE_double(simulation_time, 22.205, "Desired duration of the simulation in seconds");
DEFINE_double(max_time_step, 1.0e-4, "Simulation time step used for integrator.");

namespace drake {
namespace examples {
namespace cassie {

using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Simulator;
using drake::systems::rendering::MultibodyPositionToGeometryPose;
using Eigen::VectorBlock;
using Eigen::VectorXd;
using Eigen::Vector3d;
using drake::math::RigidTransformd;

template <int Derived>
int read_data(std::vector<Eigen::Matrix<double, Derived, 1>>& data, std::string& file_name)
{
  std::ifstream inFile;
  inFile.open(file_name, std::ios::in);
  std::string line;
        while (getline(inFile, line))
  {
    std::istringstream linestream(line);
    std::vector<std::string> vv;
    std::string v;
    while (getline(linestream, v, ' '))
    {
        vv.push_back(v);
    }

    Eigen::Matrix<double, Derived, 1> dd;
    for (int i = 0; i < Derived; i++)
    {
      dd(i, 0) = std::atof(vv[i].c_str());
    }
    data.push_back(dd);
  }
  inFile.close();

  return 1;
}

void DoMain() {
  if (FLAGS_max_time_step < 0) {
    throw std::runtime_error(
        "mbp_discrete_update_period must be a non-negative number.");
  }

  std::string file_name;
  std::string full_path;

  file_name = "/home/yuki/drake/examples/cassie/lists/log_COM.txt";
  // full_path = FindResourceOrThrow(file_name);
  std::vector<Eigen::Matrix<double, 9, 1>> COM_list;
  read_data<9>(COM_list, file_name);

  file_name = "/home/yuki/drake/examples/cassie/lists/log_l_foot.txt";
  // full_path = FindResourceOrThrow(file_name);
  std::vector<Eigen::Matrix<double, 9, 1>> l_foot_list;
  read_data<9>(l_foot_list, file_name);

  file_name = "/home/yuki/drake/examples/cassie/lists/log_r_foot.txt";
  // full_path = FindResourceOrThrow(file_name);
  std::vector<Eigen::Matrix<double, 9, 1>> r_foot_list;
  read_data<9>(r_foot_list, file_name);

  file_name = "/home/yuki/drake/examples/cassie/lists/log_heading.txt";
  // full_path = FindResourceOrThrow(file_name);
  std::vector<Eigen::Matrix<double, 1, 1>> heading_list;
  read_data<1>(heading_list, file_name);

  // Build a multibody plant.
  systems::DiagramBuilder<double> builder;
  MultibodyPlant<double> plant(0.2);
  plant.set_name("plant");

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  plant.RegisterAsSourceForSceneGraph(&scene_graph);

  // Load Cassie    
  std::string full_name;
  full_name = FindResourceOrThrow("drake/examples/cassie/models/cassie.urdf");

  Parser parser(&plant, &scene_graph);
  parser.AddModelFromFile(full_name);

  // Now the model is complete.
  plant.Finalize();

  auto cas = builder.AddSystem<drake::examples::cassie::Cassie>();
  cas->InitSystem(COM_list, l_foot_list, r_foot_list, heading_list);
  
  auto cassie_to_pose =
        builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant);
  
  builder.Connect(*cas, *cassie_to_pose);
  builder.Connect(cassie_to_pose->get_output_port(), scene_graph.get_source_pose_port(
    plant.get_source_id().value()));
  drake::geometry::ConnectDrakeVisualizer(&builder, scene_graph);
  auto diagram = builder.Build();

  auto context = diagram->CreateDefaultContext();
  
  // Set up simulator.
  auto simulator = std::make_unique<Simulator<double>>(*diagram, std::move(context));

  systems::Context<double>& root_context = simulator->get_mutable_context();

  // Context: 3 abs, 0 cont, 0 dis
  drake::log()->info("context absStates: {}",root_context.num_abstract_states());
  drake::log()->info("context contStates: {}",root_context.num_continuous_states());
  drake::log()->info("context disStates: {}",root_context.num_discrete_state_groups());


  simulator->set_target_realtime_rate(1.0);
  simulator->Initialize();
  simulator->AdvanceTo(FLAGS_simulation_time);

}

}  // namespace cassie
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "Cassie robot following CoM and feet trajectories"
      " with joint angles calculated by IK.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::examples::cassie::DoMain();
  return 0;
}

