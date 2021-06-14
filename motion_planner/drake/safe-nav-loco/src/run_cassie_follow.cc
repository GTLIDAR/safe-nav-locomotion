#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
//#include "drake/math/RigidTransform"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/parsers/urdf_parser.h"
//#include "drake/systems/framework/diagram_builder.h"
#include "drake/safe-nav-loco/include/slugs_interface.h"
#include "drake/safe-nav-loco/include/phase_space_planner.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"
#include "drake/safe-nav-loco/include/cassie.h"
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
using drake::math::RigidTransform;

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
  
  file_name = "/home/sa-zhao/code/safe-nav-locomotion/motion_planner/drake/safe-nav-loco/vis/log_COM.txt";
  std::vector<Eigen::Matrix<double, 9, 1>> COM_list;
  read_data<9>(COM_list, file_name);

  file_name = "/home/sa-zhao/code/safe-nav-locomotion/motion_planner/drake/safe-nav-loco/vis/log_l_foot.txt";
  std::vector<Eigen::Matrix<double, 9, 1>> l_foot_list;
  read_data<9>(l_foot_list, file_name);

  file_name = "/home/sa-zhao/code/safe-nav-locomotion/motion_planner/drake/safe-nav-loco/vis/log_r_foot.txt";
  std::vector<Eigen::Matrix<double, 9, 1>> r_foot_list;
  read_data<9>(r_foot_list, file_name);

  file_name = "/home/sa-zhao/code/safe-nav-locomotion/motion_planner/drake/safe-nav-loco/vis/log_heading.txt";
  std::vector<Eigen::Matrix<double, 1, 1>> heading_list;
  read_data<1>(heading_list, file_name);

  file_name = "/home/sa-zhao/code/safe-nav-locomotion/motion_planner/drake/safe-nav-loco/vis/log_obstacle.txt";
  std::vector<Eigen::Matrix<double, 6, 1>> location_list;
  read_data<6>(location_list, file_name);





  systems::DiagramBuilder<double> builder;
  MultibodyPlant<double> plant(0.2);
  plant.set_name("plant");

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  plant.RegisterAsSourceForSceneGraph(&scene_graph);


  // Load Cassie   
  
  std::string full_name;
  full_name = FindResourceOrThrow("drake/safe-nav-loco/model/cassie.urdf");


  Parser parser(&plant, &scene_graph);
  parser.AddModelFromFile(full_name);
  
  std::string full_name1;
  full_name1 = FindResourceOrThrow("drake/safe-nav-loco/model/cart.urdf");
  parser.AddModelFromFile(full_name1);

  
  std::string full_name2;
  full_name2 = FindResourceOrThrow("drake/safe-nav-loco/model/cart1.urdf");
  parser.AddModelFromFile(full_name2);

/*
  multibody::ModelInstanceIndex cart2 = parser.AddModelFromFile(
          FindResourceOrThrow("drake/safe-nav-loco/model/cart1.urdf"));

  RigidTransform<double> X_cart(Vector3d(0, 7, 0.65));
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_footprint", cart2), X_cart);
*/

  multibody::ModelInstanceIndex kuka1 = parser.AddModelFromFile(
          FindResourceOrThrow("drake/safe-nav-loco/model/iiwa7_no_world_joint.urdf"));

  RigidTransform<double> kuka1_xyz(Vector3d((2.70351)*3.5, 2.70351*1.65, 0.5));
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("iiwa_link_0", kuka1), kuka1_xyz);

  multibody::ModelInstanceIndex kuka2 = parser.AddModelFromFile(
          FindResourceOrThrow("drake/safe-nav-loco/model/kuka2.urdf"));

  RigidTransform<double> kuka2_xyz(Vector3d((2.70351)*4.35, 2.70351*10.5, 0.5+1.2));
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("iiwa_link_0", kuka2), kuka2_xyz);

  multibody::ModelInstanceIndex kuka3 = parser.AddModelFromFile(
          FindResourceOrThrow("drake/safe-nav-loco/model/kuka3.urdf"));

  RigidTransform<double> kuka3_xyz(Vector3d((2.70351)*2.5, 2.70351*1.65, 0.5));
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("iiwa_link_0", kuka3), kuka3_xyz);

  multibody::ModelInstanceIndex navigation_instance = parser.AddModelFromFile(
          FindResourceOrThrow("drake/safe-nav-loco/model/jrnl_environment.sdf"));
  // Welding home_environment to make sure it doesn't move
  RigidTransform<double> X_WT(Vector3d(0, 0, 0));
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("floor", navigation_instance), X_WT);
  

  
// Now the model is complete.
  plant.Finalize();

  auto cas = builder.AddSystem<drake::examples::cassie::Cassie>();
  cas->InitSystem(COM_list, l_foot_list, r_foot_list, heading_list, location_list);
  
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
  simulator->set_target_realtime_rate(0.25); //runtime rate 
  simulator->Initialize();
  simulator->AdvanceTo(500);

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

