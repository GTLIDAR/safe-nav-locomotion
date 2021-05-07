#include <memory>

#include <gflags/gflags.h>
#include <jsoncpp/json/json.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

#include "drake/examples/athena_cassie/athena_cassie.h"
DEFINE_double(simulation_time, 23.427	, "Desired duration of the simulation in seconds");
DEFINE_double(max_time_step, 1.0e-4, "Simulation time step used for integrator.");

namespace drake {
namespace examples {
namespace athena_cassie {

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

Json::Value json_parser(std::string file_name){
  Json::Reader reader;
  Json::Value root;
  drake::log()->info("json parser activated.");
  
  // User does not have to specify /home/your_name/ anymore!
  std::ifstream myfile(FindResourceOrThrow(file_name));
  myfile >> root;
  drake::log()->info("reading json config file completed.");
  return root;

  // Example of accessing (DO NOT ERASE BELOW)
  // cout << root["name"].asString() << endl;
}

void DoMain() {
  // Load in config.json file first
  Json::Value root = 
    json_parser("drake/examples/athena_cassie/config.json");

  if (FLAGS_max_time_step < 0) {
    throw std::runtime_error(
        "mbp_discrete_update_period must be a non-negative number.");
  }

  // Load athena_cassie path files
  std::string full_path;

  full_path = root["path_to_drake"].asString() + root["COM_list"].asString();
  std::vector<Eigen::Matrix<double, 9, 1>> COM_list;
  read_data<9>(COM_list, full_path);

  full_path = root["path_to_drake"].asString() + root["l_foot_list"].asString();
  std::vector<Eigen::Matrix<double, 9, 1>> l_foot_list;
  read_data<9>(l_foot_list, full_path);

  full_path = root["path_to_drake"].asString() + root["r_foot_list"].asString();
  std::vector<Eigen::Matrix<double, 9, 1>> r_foot_list;
  read_data<9>(r_foot_list, full_path);

  full_path = root["path_to_drake"].asString() + root["l_wrist_list"].asString();
  std::vector<Eigen::Matrix<double, 6, 1>> l_wrist_list;
  read_data<6>(l_wrist_list, full_path);

  full_path = root["path_to_drake"].asString() + root["r_wrist_list"].asString();
  std::vector<Eigen::Matrix<double, 6, 1>> r_wrist_list;
  read_data<6>(r_wrist_list, full_path);

  full_path = root["path_to_drake"].asString() + root["heading_list"].asString();
  std::vector<Eigen::Matrix<double, 1, 1>> heading_list;
  read_data<1>(heading_list, full_path);

  full_path = root["path_to_drake"].asString() + root["box_list"].asString();
  std::vector<Eigen::Matrix<double, 3, 1>> box_list;
  read_data<3>(box_list, full_path);

  // Build a multibody plant.
  systems::DiagramBuilder<double> builder;
  MultibodyPlant<double> plant(0.2);
  plant.set_name("plant");

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  plant.RegisterAsSourceForSceneGraph(&scene_graph);

  // Load Cassie    
  std::string full_name;
  full_name = FindResourceOrThrow("drake/examples/athena_cassie/models/cassie_with_athena_intermediate.urdf");

  Parser parser(&plant, &scene_graph);
  parser.AddModelFromFile(full_name);

  full_name = FindResourceOrThrow("drake/examples/athena_cassie/models/box.urdf");
  parser.AddModelFromFile(full_name);

  multibody::ModelInstanceIndex  floor1_instance = 
  parser.AddModelFromFile(FindResourceOrThrow("drake/examples/athena_cassie/models/environment_floor_1.sdf"));

  const RigidTransformd X_floor1(Vector3d(0, 0, 0));
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("floor", floor1_instance), X_floor1);
  // Now the model is complete.
  plant.Finalize();

  auto cas = builder.AddSystem<drake::examples::athena_cassie::AthenaCassie>();
  cas->InitSystem(COM_list, l_foot_list, r_foot_list, l_wrist_list, r_wrist_list, heading_list, box_list);
  
  auto cassie_to_pose =
        builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant);
  
  builder.Connect(*cas, *cassie_to_pose);
  builder.Connect(cassie_to_pose->get_output_port(), scene_graph.get_source_pose_port(
    plant.get_source_id().value()));
  drake::geometry::ConnectDrakeVisualizer(&builder, scene_graph);
  auto diagram = builder.Build();
drake::log()->info("num joints: {}",plant.num_joints());  
  auto context = diagram->CreateDefaultContext();
  
  // Set up simulator.
  auto simulator = std::make_unique<Simulator<double>>(*diagram, std::move(context));
  simulator->set_target_realtime_rate(1.0);
  simulator->Initialize();
  if(root["simulation_duration_infinite"].asBool())
  {
    simulator->AdvanceTo(std::numeric_limits<double>::infinity());
  }
  else
  {
    simulator->AdvanceTo(root["simulation_duration"].asDouble());
  }

}

}  // namespace cassie
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "Cassie robot following CoM and feet trajectories"
      " with joint angles calculated by IK.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::examples::athena_cassie::DoMain();
  return 0;
}

