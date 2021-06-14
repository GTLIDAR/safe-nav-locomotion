#include <memory>

#include <gflags/gflags.h>
#include <fstream>
#include <jsoncpp/json/json.h>
#include <vector>

#include "drake/common/find_resource.h"
#include "drake/examples/athena_cassie/athena_cassie.h"
#include "drake/examples/athena_cassie/athena_cassie_lcm.h"
#include "drake/examples/cassie/cassie.h"
#include "drake/examples/cassie/cassie_lcm.h"
#include "drake/examples/collaboration_station/object.h"
#include "drake/examples/collaboration_station/object_lcm.h"
#include "drake/examples/kuka_iiwa_arm/iiwa.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/examples/quadrotor/quadrotor.h"
#include "drake/examples/quadrotor/quadrotor_lcm.h"
#include "drake/examples/collaboration_station/robot_time_sender.h"
#include "drake/examples/mobile_robots/turtle_bot/turtle_bot.h"
#include "drake/examples/mobile_robots/turtle_bot/turtle_lcm.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_cassie_command.hpp"
#include "drake/lcmt_cassie_status.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_robot_time.hpp"
#include "drake/lcmt_object_status.hpp"
#include "drake/lcmt_quadrotor_command.hpp"
#include "drake/lcmt_quadrotor_status.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/lcmt_turtle_command.hpp"
#include "drake/lcmt_turtle_status.hpp"
#include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

namespace drake {
namespace examples {
namespace collaboration_station {

using drake::examples::athena_cassie::AthenaCassieStatusReceiver;
using drake::examples::athena_cassie::AthenaCassieStatusSender;
using drake::examples::athena_cassie::kAthenaCassieLcmStatusPeriod;

using drake::examples::cassie::CassieStatusReceiver;
using drake::examples::cassie::CassieStatusSender;
using drake::examples::cassie::kCassieLcmStatusPeriod;

using drake::examples::collaboration_station::ObjectStatusReceiver;
using drake::examples::collaboration_station::ObjectStatusSender;
using drake::examples::collaboration_station::kObjectLcmStatusPeriod;

using drake::examples::quadrotor::QuadrotorStatusReceiver;
using drake::examples::quadrotor::QuadrotorStatusSender;
using drake::examples::quadrotor::kQuadrotorLcmStatusPeriod;

using drake::examples::mobile_robots::turtle_bot::TurtleStatusReceiver;
using drake::examples::mobile_robots::turtle_bot::TurtleStatusSender;
using drake::examples::mobile_robots::turtle_bot::kTurtleLcmStatusPeriod;

using drake::examples::kuka_iiwa_arm::IiwaStatusReceiver;
using drake::examples::kuka_iiwa_arm::IiwaStatusSender;
using drake::examples::kuka_iiwa_arm::kIiwaLcmStatusPeriod;
using drake::manipulation::kuka_iiwa::kIiwaArmNumJoints;

using drake::manipulation::schunk_wsg::SchunkWsgStatusSender;
using drake::manipulation::schunk_wsg::SchunkWsgStatusReceiver;
using drake::manipulation::schunk_wsg::kSchunkWsgLcmStatusPeriod;
using drake::manipulation::schunk_wsg::kSchunkWsgNumPositions;

using drake::examples::collaboration_station::RobotTimeSender;

using drake::geometry::SceneGraph;
using drake::math::RigidTransformd;
using drake::math::RigidTransform;
using math::RollPitchYaw;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Simulator;
using drake::systems::rendering::MultibodyPositionToGeometryPose;
using Eigen::VectorBlock;
using Eigen::VectorXd;
using Eigen::Vector3d;

#define USE_LCM true
#define USE_ENVIRONMENT true
#define USE_QUADROTOR false
#define USE_QUADROTOR_TEAM true
#define USE_TURTLE false
#define USE_TURTLE_2 false
#define USE_STATIC_TURTLE false
#define USE_IIWA true
#define USE_WSG true
#define USE_IIWA_2 false
#define USE_WSG_2 false
#define USE_IIWA_3 false
#define USE_WSG_3 false
#define USE_MULTI_OBJECT true
#define USE_MULTI_OBJECT_2 true
#define USE_CASSIE false
#define USE_ATHENA_CASSIE false
#define USE_STATIC_OPEN_BOX false

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
    json_parser("drake/examples/collaboration_station/config.json");

  if (root["max_time_step"].asDouble() < 0) 
  {
    throw std::runtime_error(
        "mbp_discrete_update_period must be a non-negative number.");
  }

  // Build a multibody plant.
  systems::DiagramBuilder<double> builder;
  MultibodyPlant<double> plant(1e-5);
  plant.set_name("plant");

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  Parser parser(&plant, &scene_graph);

  #if USE_CASSIE
  MultibodyPlant<double> plant2(1e-5);
  plant2.set_name("plant2");
  Parser parser2(&plant2, &scene_graph);
  #endif

  #if USE_ATHENA_CASSIE
  MultibodyPlant<double> plant3(1e-5);
  plant3.set_name("plant3");
  Parser parser3(&plant3, &scene_graph);
  #endif

  #if USE_IIWA_2
  MultibodyPlant<double> plant4(1e-5);
  plant4.set_name("plant4");
  Parser parser4(&plant4, &scene_graph);
  #endif

  #if USE_IIWA_3
  MultibodyPlant<double> plant5(1e-5);
  plant5.set_name("plant5");
  Parser parser5(&plant5, &scene_graph);
  #endif

  #if USE_QUADROTOR_TEAM
  MultibodyPlant<double> plant6(1e-5);
  plant6.set_name("plant6");
  Parser parser6(&plant6, &scene_graph);
  #endif
  
  unsigned int num_robots = 0;
  #if USE_MULTI_OBJECT 
    num_robots++; 
  #endif
  #if USE_QUADROTOR
    num_robots++;
  #endif
  #if USE_TURTLE 
    num_robots++;  
  #endif
  #if USE_TURTLE_2
    num_robots++;  
  #endif
  #if USE_IIWA 
    num_robots++;  
  #endif
  #if USE_WSG 
    num_robots++;  
  #endif

  #if USE_ENVIRONMENT
  // Load Environment
  multibody::ModelInstanceIndex floor_1_instance = parser.AddModelFromFile(
    FindResourceOrThrow(root["environment_floor_1_path"].asString()));
  // Load Environment floor 2
  multibody::ModelInstanceIndex floor_2_instance = parser.AddModelFromFile(
    FindResourceOrThrow(root["environment_floor_2_path"].asString()));
  // Load Stairs
  multibody::ModelInstanceIndex stairs_instance = parser.AddModelFromFile(
    FindResourceOrThrow(root["environment_stairs_path"].asString()));
  // Load Shelf
  multibody::ModelInstanceIndex shelf_instance = parser.AddModelFromFile(
    FindResourceOrThrow(root["environment_shelf_path"].asString()));
  // Load Shelf 2
  multibody::ModelInstanceIndex shelf_instance_2 = parser.AddModelFromFile(
    FindResourceOrThrow(root["environment_shelf_2_path"].asString()));
  // Load Charging Station
  multibody::ModelInstanceIndex charging_station_instance = parser.AddModelFromFile(
    FindResourceOrThrow(root["charging_station_path"].asString()));
  // Load Charging Station 2
  multibody::ModelInstanceIndex charging_station_instance_2 = parser.AddModelFromFile(
    FindResourceOrThrow(root["charging_station_path_2"].asString()));
  // Load Multi Boxes
  multibody::ModelInstanceIndex multi_static_boxes_instance = parser.AddModelFromFile(
    FindResourceOrThrow(root["multi_static_boxes_path"].asString()));

  #endif

  #if USE_STATIC_OPEN_BOX
  // Load Static Open Box
  multibody::ModelInstanceIndex static_open_box_instance = parser.AddModelFromFile(
    FindResourceOrThrow(root["static_box_path"].asString()));
  // Load Static Open Box2
  multibody::ModelInstanceIndex static_open_box_2_instance = parser.AddModelFromFile(
    FindResourceOrThrow(root["static_box_2_path"].asString()));
  #endif

  #if USE_STATIC_TURTLE
  // Load Static Turtle
  multibody::ModelInstanceIndex static_turtle_instance = parser.AddModelFromFile(
    FindResourceOrThrow(root["static_turtle_path"].asString()));
  #endif
  
  #if USE_MULTI_OBJECT
  multibody::ModelInstanceIndex multi_object_instance = parser.AddModelFromFile(
    FindResourceOrThrow(root["multi_object_path"].asString()));
  #endif
  #if USE_MULTI_OBJECT_2
  multibody::ModelInstanceIndex multi_object_instance_2 = parser5.AddModelFromFile(
    FindResourceOrThrow(root["multi_object_path_2"].asString()));
  #endif
  #if USE_QUADROTOR
  multibody::ModelInstanceIndex quadrotor_instance = parser.AddModelFromFile(
    FindResourceOrThrow(root["robot_quadrotor_path"].asString()));
  #endif 
  #if USE_QUADROTOR_TEAM
  multibody::ModelInstanceIndex quadrotor_team_instance = parser6.AddModelFromFile(
    FindResourceOrThrow(root["robot_quadrotor_team_path"].asString()));
  multibody::ModelInstanceIndex quadrotor_box_1_instance = parser6.AddModelFromFile(
    FindResourceOrThrow(root["quadrotor_box_1_path"].asString()));
  multibody::ModelInstanceIndex quadrotor_box_2_instance = parser6.AddModelFromFile(
    FindResourceOrThrow(root["quadrotor_box_2_path"].asString()));
  multibody::ModelInstanceIndex quadrotor_box_3_instance = parser6.AddModelFromFile(
    FindResourceOrThrow(root["quadrotor_box_3_path"].asString()));
  multibody::ModelInstanceIndex quadrotor_box_4_instance = parser6.AddModelFromFile(
    FindResourceOrThrow(root["quadrotor_box_4_path"].asString()));
  #endif
  #if USE_TURTLE
  multibody::ModelInstanceIndex turtle_instance = parser.AddModelFromFile(
    FindResourceOrThrow(root["robot_turtle_path"].asString()));
  #endif
  #if USE_TURTLE_2
  multibody::ModelInstanceIndex turtle_2_instance = parser.AddModelFromFile(
    FindResourceOrThrow(root["robot_turtle_2_path"].asString()));
  #endif
  #if USE_IIWA
  multibody::ModelInstanceIndex iiwa_instance = parser.AddModelFromFile(
    FindResourceOrThrow(root["robot_iiwa_path"].asString()));
  #endif
  #if USE_WSG
  multibody::ModelInstanceIndex connector_instance = parser.AddModelFromFile(
    FindResourceOrThrow(root["robot_connector_path"].asString()));
  multibody::ModelInstanceIndex wsg_instance = parser.AddModelFromFile(
    FindResourceOrThrow(root["robot_wsg_path"].asString()));
  #endif
  #if USE_IIWA_2
  multibody::ModelInstanceIndex iiwa_instance_2 = parser4.AddModelFromFile(
    FindResourceOrThrow(root["robot_iiwa_2_path"].asString()));
  #endif
  #if USE_WSG_2
  multibody::ModelInstanceIndex connector_instance_2 = parser4.AddModelFromFile(
    FindResourceOrThrow(root["robot_connector_2_path"].asString()));
  multibody::ModelInstanceIndex wsg_instance_2 = parser4.AddModelFromFile(
    FindResourceOrThrow(root["robot_wsg_2_path"].asString()));
  #endif
  #if USE_IIWA_3
  multibody::ModelInstanceIndex iiwa_instance_3 = parser5.AddModelFromFile(
    FindResourceOrThrow(root["robot_iiwa_3_path"].asString()));
  #endif
  #if USE_WSG_3
  multibody::ModelInstanceIndex connector_instance_3 = parser5.AddModelFromFile(
    FindResourceOrThrow(root["robot_connector_3_path"].asString()));
  multibody::ModelInstanceIndex wsg_instance_3 = parser5.AddModelFromFile(
    FindResourceOrThrow(root["robot_wsg_3_path"].asString()));
  #endif
  #if USE_CASSIE
  multibody::ModelInstanceIndex cassie_instance = parser2.AddModelFromFile(
    FindResourceOrThrow(root["robot_cassie_path"].asString()));
  #endif
  #if USE_ATHENA_CASSIE
  multibody::ModelInstanceIndex athena_cassie_instance = parser3.AddModelFromFile(
    FindResourceOrThrow(root["robot_athena_cassie_path"].asString()));
  multibody::ModelInstanceIndex mobile_box_instance = parser3.AddModelFromFile(
    FindResourceOrThrow(root["mobile_box_path"].asString()));
  #endif

  #if USE_ENVIRONMENT
  // Weld environment floor 1
  const RigidTransform<double> X_floor(
      Vector3d(0, 0, 0));
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(
      "floor", floor_1_instance), X_floor);

 // Weld environment floor 2
  const RigidTransform<double> X_floor_2(
      Vector3d(-0.05, 0, 0));
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(
      "center", floor_2_instance), X_floor_2);

  // Weld environment stairs
  const RigidTransform<double> X_stairs(
      RollPitchYaw<double>(0, 0, M_PI_2), Vector3d(-1.2, -3, -0.0375));
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(
      "step_0", stairs_instance), X_stairs);

  // Weld environment shelf
  const RigidTransform<double> X_shelf(
      RollPitchYaw<double>(
        root["shelf_pose"][0].asDouble(), 
        root["shelf_pose"][1].asDouble(), 
        root["shelf_pose"][2].asDouble()),
      Vector3d( 
        root["shelf_pose"][3].asDouble(), 
        root["shelf_pose"][4].asDouble(), 
        root["shelf_pose"][5].asDouble())
  );
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(
      "shelf_body", shelf_instance), X_shelf);

  // Weld environment shelf
  const RigidTransform<double> X_shelf_2(
      RollPitchYaw<double>(
        root["shelf_pose_2"][0].asDouble(), 
        root["shelf_pose_2"][1].asDouble(), 
        root["shelf_pose_2"][2].asDouble()),
      Vector3d( 
        root["shelf_pose_2"][3].asDouble(), 
        root["shelf_pose_2"][4].asDouble(), 
        root["shelf_pose_2"][5].asDouble())
  );
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(
      "shelf_body_2", shelf_instance_2), X_shelf_2);

  // Weld charging station
  const RigidTransform<double> X_charging_station(
      RollPitchYaw<double>(
        root["charging_station_pose"][0].asDouble(), 
        root["charging_station_pose"][1].asDouble(), 
        root["charging_station_pose"][2].asDouble()),
      Vector3d( 
        root["charging_station_pose"][3].asDouble(), 
        root["charging_station_pose"][4].asDouble(), 
        root["charging_station_pose"][5].asDouble())
  );
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(
      "base", charging_station_instance), X_charging_station);

  // Weld charging station 2
  const RigidTransform<double> X_charging_station_2(
      RollPitchYaw<double>(
        root["charging_station_pose_2"][0].asDouble(), 
        root["charging_station_pose_2"][1].asDouble(), 
        root["charging_station_pose_2"][2].asDouble()),
      Vector3d( 
        root["charging_station_pose_2"][3].asDouble(), 
        root["charging_station_pose_2"][4].asDouble(), 
        root["charging_station_pose_2"][5].asDouble())
  );
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(
      "base", charging_station_instance_2), X_charging_station_2);

  // Weld multi static boxes
  const RigidTransform<double> X_multi_static_boxes(    
    RollPitchYaw<double>(0,0,0),
    Vector3d(0,0,0));
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(
      "base", multi_static_boxes_instance), X_multi_static_boxes);

  #endif
  #if USE_STATIC_OPEN_BOX
  // Weld Static Open Box
  const RigidTransform<double> X_static_open_box(
      RollPitchYaw<double>(
        root["box_downstairs_placement"][0].asDouble(), 
        root["box_downstairs_placement"][1].asDouble(), 
        root["box_downstairs_placement"][2].asDouble()),
      Vector3d( 
        root["box_downstairs_placement"][3].asDouble(), 
        root["box_downstairs_placement"][4].asDouble(), 
        root["box_downstairs_placement"][5].asDouble())
  );
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(
      "base1", static_open_box_instance), X_static_open_box);

  // Weld Static Open Box2
  const RigidTransform<double> X_static_open_box_2(
      RollPitchYaw<double>(
        root["box_upstairs_placement"][0].asDouble(), 
        root["box_upstairs_placement"][1].asDouble(), 
        root["box_upstairs_placement"][2].asDouble()),
      Vector3d( 
        root["box_upstairs_placement"][3].asDouble(), 
        root["box_upstairs_placement"][4].asDouble(), 
        root["box_upstairs_placement"][5].asDouble())
  );
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(
      "base1", static_open_box_2_instance), X_static_open_box_2);
  #endif

  #if USE_STATIC_TURTLE
  // Weld static turtle
  const RigidTransform<double> X_static_turtle(
      RollPitchYaw<double>(
        root["static_turtle_pose"][0].asDouble(), 
        root["static_turtle_pose"][1].asDouble(), 
        root["static_turtle_pose"][2].asDouble()),
      Vector3d( 
        root["static_turtle_pose"][3].asDouble(), 
        root["static_turtle_pose"][4].asDouble(), 
        root["static_turtle_pose"][5].asDouble())
  );
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(
      "base_footprint", static_turtle_instance), X_static_turtle);
  #endif
  #if USE_MULTI_OBJECT
  const RigidTransform<double> X_WB(
    RollPitchYaw<double>(0,0,0),
    Vector3d(0,0,0));
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(
      "base", multi_object_instance), X_WB);
  #endif
  #if USE_MULTI_OBJECT_2
    #if USE_IIWA_3
    const RigidTransform<double> X_WB_2(
      RollPitchYaw<double>(0,0,0),
      Vector3d(0,0,0));
    plant5.WeldFrames(plant5.world_frame(), plant5.GetFrameByName(
      "base", multi_object_instance_2), X_WB_2);
    #else
      DRAKE_DEMAND(false); // Iiwa 3 needs to be turned on
    #endif
  #endif
  #if USE_IIWA
  const RigidTransform<double> X_WI(
      RollPitchYaw<double>(
        root["iiwa_pose"][0].asDouble(),
        root["iiwa_pose"][1].asDouble(),
        root["iiwa_pose"][2].asDouble()),
      Vector3d(
        root["iiwa_pose"][3].asDouble(),
        root["iiwa_pose"][4].asDouble(),
        root["iiwa_pose"][5].asDouble())
  );
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(
      "iiwa_link_0", iiwa_instance), X_WI);
  #endif
  #if USE_WSG
    #if USE_IIWA    
    const multibody::Frame<double>& link7 =
        plant.GetFrameByName("iiwa_link_7", iiwa_instance);
    const RigidTransform<double> X_7C(
        RollPitchYaw<double>(0, 0, M_PI_2), Vector3d(0, 0, 0.045));
    plant.WeldFrames(link7, plant.GetFrameByName(
        "connector_link", connector_instance), X_7C);

    const RigidTransform<double> X_7G(
        RollPitchYaw<double>(M_PI_2, 0, M_PI_2), Vector3d(0, 0, 0.1));
    plant.WeldFrames(link7, plant.GetFrameByName(
        "body", wsg_instance), X_7G);
    #else
    drake::log()->info("If you want a WSG, you need an Iiwa");
    DRAKE_DEMAND(false);
    #endif
  #endif
  #if USE_IIWA_2
  const RigidTransform<double> X_WI_2(
      RollPitchYaw<double>(
        root["iiwa_2_pose"][0].asDouble(),
        root["iiwa_2_pose"][1].asDouble(),
        root["iiwa_2_pose"][2].asDouble()),
      Vector3d(
        root["iiwa_2_pose"][3].asDouble(),
        root["iiwa_2_pose"][4].asDouble(),
        root["iiwa_2_pose"][5].asDouble())
  );
  plant4.WeldFrames(plant4.world_frame(), plant4.GetFrameByName(
      "iiwa_link_0", iiwa_instance_2), X_WI_2);
  #endif
  #if USE_WSG_2
    #if USE_IIWA_2    
    const multibody::Frame<double>& link7_2 =
        plant4.GetFrameByName("iiwa_link_7", iiwa_instance_2);
    const RigidTransform<double> X_7C_2(
        RollPitchYaw<double>(0, 0, M_PI_2), Vector3d(0, 0, 0.045));
    plant4.WeldFrames(link7_2, plant4.GetFrameByName(
        "connector_link", connector_instance_2), X_7C_2);

    const RigidTransform<double> X_7G_2(
        RollPitchYaw<double>(M_PI_2, 0, M_PI_2), Vector3d(0, 0, 0.1));
    plant4.WeldFrames(link7_2, plant4.GetFrameByName(
        "body", wsg_instance_2), X_7G_2);
    #else
    drake::log()->info("If you want a WSG_2, you need an Iiwa_2");
    DRAKE_DEMAND(false);
    #endif
  #endif
  #if USE_IIWA_3
  const RigidTransform<double> X_WI_3(
      RollPitchYaw<double>(
        root["iiwa_3_pose"][0].asDouble(),
        root["iiwa_3_pose"][1].asDouble(),
        root["iiwa_3_pose"][2].asDouble()),
      Vector3d(
        root["iiwa_3_pose"][3].asDouble(),
        root["iiwa_3_pose"][4].asDouble(),
        root["iiwa_3_pose"][5].asDouble())
  );
  plant5.WeldFrames(plant5.world_frame(), plant5.GetFrameByName(
      "iiwa_link_0", iiwa_instance_3), X_WI_3);
  #endif
  #if USE_WSG_3
    #if USE_IIWA_3    
    const multibody::Frame<double>& link7_3 =
        plant5.GetFrameByName("iiwa_link_7", iiwa_instance_3);
    const RigidTransform<double> X_7C_3(
        RollPitchYaw<double>(0, 0, M_PI_2), Vector3d(0, 0, 0.045));
    plant5.WeldFrames(link7_3, plant5.GetFrameByName(
        "connector_link", connector_instance_3), X_7C_3);

    const RigidTransform<double> X_7G_3(
        RollPitchYaw<double>(M_PI_2, 0, M_PI_2), Vector3d(0, 0, 0.1));
    plant5.WeldFrames(link7_3, plant5.GetFrameByName(
        "body", wsg_instance_3), X_7G_3);
    #else
    drake::log()->info("If you want a WSG_2, you need an Iiwa_2");
    DRAKE_DEMAND(false);
    #endif
  #endif
  // Now the model is complete.
  plant.Finalize();

  #if USE_CASSIE
  plant2.Finalize();
  #endif
  #if USE_ATHENA_CASSIE
  plant3.Finalize();
  #endif
  #if USE_IIWA_2
  plant4.Finalize();
  #endif
  #if USE_IIWA_3
  plant5.Finalize();
  #endif
  #if USE_QUADROTOR_TEAM
  plant6.Finalize();
  #endif
// ------------------------ USEFUL VARIABLES ---------------------------------
  std::vector<int> all_joints;
  all_joints.resize(num_robots,0);

  #if USE_MULTI_OBJECT
  const int multi_object_num_dof = plant.num_positions(multi_object_instance);
  const int multi_object_id = root["multi_object_id"].asInt();
  all_joints[multi_object_id] = multi_object_num_dof;
  #endif
  #if USE_MULTI_OBJECT_2
  const int multi_object_num_dof_2 = plant5.num_positions(multi_object_instance_2);
  #endif
  #if USE_QUADROTOR
  const int quadrotor_num_joints = plant.num_positions(quadrotor_instance);
  const int quadrotor_id = root["quadrotor_id"].asInt();
  all_joints[quadrotor_id] = quadrotor_num_joints;
  #endif
  #if USE_QUADROTOR_TEAM
  const int quadrotor_team_num_joints = 
    plant6.num_positions(quadrotor_team_instance) + 
    plant6.num_positions(quadrotor_box_1_instance) +
    plant6.num_positions(quadrotor_box_2_instance) + 
    plant6.num_positions(quadrotor_box_3_instance) +
    plant6.num_positions(quadrotor_box_4_instance);
  #endif
  #if USE_TURTLE
  const int turtle_num_joints = plant.num_positions(turtle_instance);
  const int turtle_id = root["turtle_id"].asInt();
  all_joints[turtle_id] = turtle_num_joints;
  #endif
  #if USE_TURTLE_2
  const int turtle_2_num_joints = plant.num_positions(turtle_2_instance);
  const int turtle2_id = root["turtle2_id"].asInt();
  all_joints[turtle2_id] = turtle_2_num_joints;
  #endif 
  #if USE_IIWA
  const int iiwa_id = root["iiwa_id"].asInt();
  all_joints[iiwa_id] = kIiwaArmNumJoints;
  #endif
  #if USE_WSG
  const int wsg_id = root["wsg_id"].asInt();
  all_joints[wsg_id] = kSchunkWsgNumPositions;
  #endif
  #if USE_CASSIE
  const int cassie_num_joints = plant2.num_positions(cassie_instance);
  #endif
  #if USE_ATHENA_CASSIE
  const int athena_cassie_num_joints = plant3.num_positions(athena_cassie_instance)+plant3.num_positions(mobile_box_instance);
  #endif
// ------------------------- SYSTEM CREATION ---------------------------------
  // Create LCM
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

  #if USE_MULTI_OBJECT
  auto multi_object_state_subscriber = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_object_status>(
          "MULTI_OBJECT_STATUS", lcm));
  multi_object_state_subscriber->set_name("multi_object_state_subscriber");

  auto multi_object_state_receiver =
      builder.AddSystem<ObjectStatusReceiver>(multi_object_num_dof);
  multi_object_state_receiver->set_name("multi_object_state_receiver");

  // auto multi_object_time_pub = builder.AddSystem(
  //     systems::lcm::LcmPublisherSystem::Make<drake::lcmt_robot_time>(
  //         "MULTI_OBJECT_TIME", lcm, kObjectLcmStatusPeriod /* publish period */));
  // multi_object_time_pub->set_name("multi_object_time_publisher");
  // auto multi_object_time_sender = builder.AddSystem<RobotTimeSender>();
  // multi_object_time_sender->set_name("multi_object_time_sender");
  #endif
  #if USE_MULTI_OBJECT_2
  auto multi_object_state_subscriber_2 = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_object_status>(
          "MULTI_OBJECT_STATUS_2", lcm));
  multi_object_state_subscriber_2->set_name("multi_object_state_subscriber_2");

  auto multi_object_state_receiver_2 =
      builder.AddSystem<ObjectStatusReceiver>(multi_object_num_dof_2);
  multi_object_state_receiver_2->set_name("multi_object_state_receiver_2");

  // auto multi_object_time_pub_2 = builder.AddSystem(
  //     systems::lcm::LcmPublisherSystem::Make<drake::lcmt_robot_time>(
  //         "MULTI_OBJECT_TIME_2", lcm, kObjectLcmStatusPeriod /* publish period */));
  // multi_object_time_pub_2->set_name("multi_object_time_publisher_2");
  // auto multi_object_time_sender_2 = builder.AddSystem<RobotTimeSender>();
  // multi_object_time_sender_2->set_name("multi_object_time_sender_2");
  #endif
  #if USE_QUADROTOR
  auto quadrotor_state_subscriber = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_quadrotor_status>(
          "QUADROTOR_STATUS", lcm));
  quadrotor_state_subscriber->set_name("quadrotor_state_subscriber");

  auto quadrotor_state_receiver =
      builder.AddSystem<QuadrotorStatusReceiver>(quadrotor_num_joints);
  quadrotor_state_receiver->set_name("quadrotor_state_receiver");

  auto quadrotor_time_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<drake::lcmt_robot_time>(
          "QUADROTOR_TIME", lcm, kQuadrotorLcmStatusPeriod /* publish period */));
  quadrotor_time_pub->set_name("quadrotor_time_publisher");
  auto quadrotor_time_sender = builder.AddSystem<RobotTimeSender>();
  quadrotor_time_sender->set_name("quadrotor_time_sender");
  #endif
  #if USE_QUADROTOR_TEAM
  auto quadrotor_team_state_subscriber = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_quadrotor_status>(
          "QUADROTOR_TEAM_STATUS", lcm));
  quadrotor_team_state_subscriber->set_name("quadrotor_team_state_subscriber");

  auto quadrotor_team_state_receiver =
      builder.AddSystem<QuadrotorStatusReceiver>(quadrotor_team_num_joints);
  quadrotor_team_state_receiver->set_name("quadrotor_team_state_receiver");

  auto quadrotor_team_time_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<drake::lcmt_robot_time>(
          "QUADROTOR_TEAM_TIME", lcm, kQuadrotorLcmStatusPeriod /* publish period */));
  quadrotor_team_time_pub->set_name("quadrotor_team_time_publisher");
  auto quadrotor_team_time_sender = builder.AddSystem<RobotTimeSender>();
  quadrotor_team_time_sender->set_name("quadrotor_team_time_sender");
  #endif
  #if USE_TURTLE
  auto turtle_state_subscriber = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_turtle_status>(
          "TURTLE_STATUS", lcm));
  turtle_state_subscriber->set_name("turtle_state_subscriber");

  auto turtle_state_receiver =
      builder.AddSystem<TurtleStatusReceiver>(turtle_num_joints);
  turtle_state_receiver->set_name("turtle_state_receiver");

  auto turtle_time_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<drake::lcmt_robot_time>(
          "TURTLE_TIME", lcm, kTurtleLcmStatusPeriod /* publish period */));
  turtle_time_pub->set_name("turtle_time_publisher");
  auto turtle_time_sender = builder.AddSystem<RobotTimeSender>();
  turtle_time_sender->set_name("turtle_time_sender");
  #endif
  #if USE_TURTLE_2
  auto turtle2_state_subscriber = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_turtle_status>(
          "TURTLE_2_STATUS", lcm));
  turtle2_state_subscriber->set_name("turtle_2_state_subscriber");

  auto turtle2_state_receiver =
      builder.AddSystem<TurtleStatusReceiver>(turtle_2_num_joints);
  turtle2_state_receiver->set_name("turtle_2_state_receiver");

  auto turtle2_time_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<drake::lcmt_robot_time>(
          "TURTLE_2_TIME", lcm, kTurtleLcmStatusPeriod /* publish period */));
  turtle2_time_pub->set_name("turtle_2_time_publisher");
  auto turtle2_time_sender = builder.AddSystem<RobotTimeSender>();
  turtle2_time_sender->set_name("turtle_2_time_sender");
  #endif
  #if USE_IIWA
  auto iiwa_status_sub = builder.AddSystem(
    systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_iiwa_status>(
      "IIWA_STATUS_1", lcm));
  iiwa_status_sub->set_name("iiwa_state_subscriber");
  auto iiwa_status_rec = builder.AddSystem<IiwaStatusReceiver>(kIiwaArmNumJoints);
  iiwa_status_rec->set_name("iiwa_status_receiver");

  auto iiwa_time_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<drake::lcmt_robot_time>(
        "IIWA_TIME_1", lcm, kIiwaLcmStatusPeriod /* publish period */));
  iiwa_time_pub->set_name("iiwa_time_publisher");
  auto iiwa_time_sender = builder.AddSystem<RobotTimeSender>();
  iiwa_time_sender->set_name("iiwa_time_sender");
  #endif
  #if USE_WSG
  auto wsg_status_sub = builder.AddSystem(
    systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_schunk_wsg_status>(
      "WSG_STATUS_1", lcm));
  wsg_status_sub->set_name("wsg_state_subscriber");
  auto wsg_status_rec = builder.AddSystem<SchunkWsgStatusReceiver>();
  wsg_status_rec->set_name("wsg_status_receiver");

  // auto wsg_time_pub = builder.AddSystem(
  //     systems::lcm::LcmPublisherSystem::Make<drake::lcmt_robot_time>(
  //       "WSG_TIME_1", lcm, kSchunkWsgLcmStatusPeriod /* publish period */));
  // wsg_time_pub->set_name("wsg_time_publisher");
  // auto wsg_time_sender = builder.AddSystem<RobotTimeSender>();
  // wsg_time_sender->set_name("wsg_time_sender");
  #endif
  #if USE_IIWA_2
  auto iiwa_status_sub_2 = builder.AddSystem(
    systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_iiwa_status>(
      "IIWA_STATUS_2", lcm));
  iiwa_status_sub_2->set_name("iiwa_state_subscriber_2");
  auto iiwa_status_rec_2 = builder.AddSystem<IiwaStatusReceiver>(kIiwaArmNumJoints);
  iiwa_status_rec_2->set_name("iiwa_status_receiver_2");

  auto iiwa_time_pub_2 = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<drake::lcmt_robot_time>(
        "IIWA_TIME_2", lcm, kIiwaLcmStatusPeriod /* publish period */));
  iiwa_time_pub_2->set_name("iiwa_time_publisher_2");
  auto iiwa_time_sender_2 = builder.AddSystem<RobotTimeSender>();
  iiwa_time_sender_2->set_name("iiwa_time_sender_2");
  #endif
  #if USE_WSG_2
  auto wsg_status_sub_2 = builder.AddSystem(
    systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_schunk_wsg_status>(
      "WSG_STATUS_2", lcm));
  wsg_status_sub_2->set_name("wsg_state_subscriber_2");
  auto wsg_status_rec_2 = builder.AddSystem<SchunkWsgStatusReceiver>();
  wsg_status_rec_2->set_name("wsg_status_receiver_2");

  // auto wsg_time_pub_2 = builder.AddSystem(
  //     systems::lcm::LcmPublisherSystem::Make<drake::lcmt_robot_time>(
  //       "WSG_TIME_2", lcm, kSchunkWsgLcmStatusPeriod /* publish period */));
  // wsg_time_pub_2->set_name("wsg_time_publisher_2");
  // auto wsg_time_sender_2 = builder.AddSystem<RobotTimeSender>();
  // wsg_time_sender_2->set_name("wsg_time_sender_2");
  #endif
  #if USE_IIWA_3
  auto iiwa_status_sub_3 = builder.AddSystem(
    systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_iiwa_status>(
      "IIWA_STATUS_3", lcm));
  iiwa_status_sub_3->set_name("iiwa_state_subscriber_3");
  auto iiwa_status_rec_3 = builder.AddSystem<IiwaStatusReceiver>(kIiwaArmNumJoints);
  iiwa_status_rec_3->set_name("iiwa_status_receiver_3");

  auto iiwa_time_pub_3 = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<drake::lcmt_robot_time>(
        "IIWA_TIME_3", lcm, kIiwaLcmStatusPeriod /* publish period */));
  iiwa_time_pub_3->set_name("iiwa_time_publisher_3");
  auto iiwa_time_sender_3 = builder.AddSystem<RobotTimeSender>();
  iiwa_time_sender_3->set_name("iiwa_time_sender_3");
  #endif
  #if USE_WSG_3
  auto wsg_status_sub_3 = builder.AddSystem(
    systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_schunk_wsg_status>(
      "WSG_STATUS_3", lcm));
  wsg_status_sub_3->set_name("wsg_state_subscriber_3");
  auto wsg_status_rec_3 = builder.AddSystem<SchunkWsgStatusReceiver>();
  wsg_status_rec_3->set_name("wsg_status_receiver_3");

  // auto wsg_time_pub_3 = builder.AddSystem(
  //     systems::lcm::LcmPublisherSystem::Make<drake::lcmt_robot_time>(
  //       "WSG_TIME_3", lcm, kSchunkWsgLcmStatusPeriod /* publish period */));
  // wsg_time_pub_3->set_name("wsg_time_publisher_3");
  // auto wsg_time_sender_3 = builder.AddSystem<RobotTimeSender>();
  // wsg_time_sender_3->set_name("wsg_time_sender_3");
  #endif
  #if USE_CASSIE
  auto cassie_state_subscriber = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_cassie_status>(
          "CASSIE_STATUS", lcm));
  cassie_state_subscriber->set_name("cassie_state_subscriber");

  auto cassie_state_receiver =
      builder.AddSystem<CassieStatusReceiver>(cassie_num_joints);
  cassie_state_receiver->set_name("cassie_state_receiver");

  auto cassie_time_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<drake::lcmt_robot_time>(
          "CASSIE_TIME", lcm, kCassieLcmStatusPeriod /* publish period */));
  cassie_time_pub->set_name("cassie_time_publisher");
  auto cassie_time_sender = builder.AddSystem<RobotTimeSender>();
  cassie_time_sender->set_name("cassie_time_sender");
  #endif
  #if USE_ATHENA_CASSIE
  auto athena_cassie_state_subscriber = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_cassie_status>(
          "ATHENA_CASSIE_STATUS", lcm));
  athena_cassie_state_subscriber->set_name("athena_cassie_state_subscriber");
// drake::log()->info("num joints: {}",athena_cassie_num_joints);
  auto athena_cassie_state_receiver =
      builder.AddSystem<AthenaCassieStatusReceiver>(athena_cassie_num_joints);
  athena_cassie_state_receiver->set_name("athena_cassie_state_receiver");

  auto athena_cassie_time_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<drake::lcmt_robot_time>(
          "ATHENA_CASSIE_TIME", lcm, kAthenaCassieLcmStatusPeriod /* publish period */));
  athena_cassie_time_pub->set_name("athena_cassie_time_publisher");
  auto athena_cassie_time_sender = builder.AddSystem<RobotTimeSender>();
  athena_cassie_time_sender->set_name("athena_cassie_time_sender");
  #endif

  // Plug in all plant state quantities
  std::vector<int> mux_sizes;
  for(unsigned int i = 0; i < num_robots; i++)
  {
    mux_sizes.push_back(all_joints[i]);
  }

  DRAKE_DEMAND(mux_sizes.size() == num_robots); // Subtract out cassie and athena_cassie
  auto plant_states_mux = builder.AddSystem<systems::Multiplexer>(mux_sizes);    

  // Robot states to geometry visualization system
  auto robot_to_pose =
        builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant);

  #if USE_CASSIE
  auto cassie_to_pose =
        builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant2);
  #endif
  #if USE_ATHENA_CASSIE
  auto athena_cassie_to_pose =
        builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant3);
  #endif
  #if USE_IIWA_2
  auto iiwa_2_to_pose =
        builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant4);
  std::vector<int> iiwa_2_mux_sizes;
  iiwa_2_mux_sizes.resize(2);
  iiwa_2_mux_sizes[0] = 7;
  iiwa_2_mux_sizes[1] = 2;
  auto iiwa_2_mux = 
        builder.AddSystem<systems::Multiplexer>(iiwa_2_mux_sizes);   
  #endif
  #if USE_IIWA_3
    #if USE_MULTI_OBJECT_2
    auto iiwa_3_to_pose =
          builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant5);
    std::vector<int> iiwa_3_mux_sizes;
    iiwa_3_mux_sizes.resize(3);
    iiwa_3_mux_sizes[0] = multi_object_num_dof_2;
    iiwa_3_mux_sizes[1] = 7;
    iiwa_3_mux_sizes[2] = 2;
    auto iiwa_3_mux = 
          builder.AddSystem<systems::Multiplexer>(iiwa_3_mux_sizes);
    const int iiwa_3_id = 1;
    const int wsg_3_id = 2;
    #else
    auto iiwa_3_to_pose =
          builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant5);
    std::vector<int> iiwa_3_mux_sizes;
    iiwa_3_mux_sizes.resize(2);
    iiwa_3_mux_sizes[0] = 7;
    iiwa_3_mux_sizes[1] = 2;
    auto iiwa_3_mux = 
          builder.AddSystem<systems::Multiplexer>(iiwa_3_mux_sizes);  
    const int iiwa_3_id = 0;
    const int wsg_3_id = 1;
    #endif 
  #endif
  #if USE_QUADROTOR_TEAM
  auto quadrotor_team_to_pose =
          builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant6);
  #endif

  #if USE_MULTI_OBJECT
  builder.Connect(multi_object_state_subscriber->get_output_port(),
                  multi_object_state_receiver->get_input_port());
  builder.Connect(multi_object_state_receiver->get_position_measured_output_port(),
                  plant_states_mux->get_input_port(multi_object_id));
  // builder.Connect(multi_object_time_sender->get_output_port(),
  //                 multi_object_time_pub->get_input_port());
  #endif
  #if USE_MULTI_OBJECT_2
  builder.Connect(multi_object_state_subscriber_2->get_output_port(),
                  multi_object_state_receiver_2->get_input_port());
  builder.Connect(multi_object_state_receiver_2->get_position_measured_output_port(),
                  iiwa_3_mux->get_input_port(0));
  // builder.Connect(multi_object_time_sender_2->get_output_port(),
  //                 multi_object_time_pub_2->get_input_port());
  #endif
  #if USE_QUADROTOR
  builder.Connect(quadrotor_state_subscriber->get_output_port(),
                  quadrotor_state_receiver->get_input_port());
  builder.Connect(quadrotor_state_receiver->get_position_measured_output_port(),
                  plant_states_mux->get_input_port(quadrotor_id));
  builder.Connect(quadrotor_time_sender->get_output_port(),
                  quadrotor_time_pub->get_input_port());
  #endif
  #if USE_TURTLE
  builder.Connect(turtle_state_subscriber->get_output_port(),
                  turtle_state_receiver->get_input_port());
  builder.Connect(turtle_state_receiver->get_position_measured_output_port(),
                  plant_states_mux->get_input_port(turtle_id));
  builder.Connect(turtle_time_sender->get_output_port(),
                  turtle_time_pub->get_input_port());
  #endif
  #if USE_TURTLE_2
  builder.Connect(turtle2_state_subscriber->get_output_port(),
                  turtle2_state_receiver->get_input_port());
  builder.Connect(turtle2_state_receiver->get_position_measured_output_port(),
                  plant_states_mux->get_input_port(turtle2_id));
  builder.Connect(turtle2_time_sender->get_output_port(),
                  turtle2_time_pub->get_input_port());
  #endif
  #if USE_IIWA
  builder.Connect(iiwa_status_sub->get_output_port(),
                  iiwa_status_rec->get_input_port());
  builder.Connect(iiwa_status_rec->get_position_measured_output_port(),
                  plant_states_mux->get_input_port(iiwa_id));
  builder.Connect(iiwa_time_sender->get_output_port(),
                  iiwa_time_pub->get_input_port());
  #endif
  #if USE_WSG
  builder.Connect(wsg_status_sub->get_output_port(),
                  wsg_status_rec->get_status_input_port());
  builder.Connect(wsg_status_rec->get_gripper_position_output_port(),
                  plant_states_mux->get_input_port(wsg_id));
  // builder.Connect(wsg_time_sender->get_output_port(),
  //                 wsg_time_pub->get_input_port());
  #endif
  #if USE_IIWA_2
  builder.Connect(iiwa_status_sub_2->get_output_port(),
                  iiwa_status_rec_2->get_input_port());
  builder.Connect(iiwa_status_rec_2->get_position_measured_output_port(),
                  iiwa_2_mux->get_input_port(0));
  builder.Connect(iiwa_time_sender_2->get_output_port(),
                  iiwa_time_pub_2->get_input_port());
  #endif
  #if USE_WSG_2
  builder.Connect(wsg_status_sub_2->get_output_port(),
                  wsg_status_rec_2->get_status_input_port());
  builder.Connect(wsg_status_rec_2->get_gripper_position_output_port(),
                  iiwa_2_mux->get_input_port(1));
  // builder.Connect(wsg_time_sender_2->get_output_port(),
  //                 wsg_time_pub_2->get_input_port());
  #endif
  #if USE_IIWA_3
  builder.Connect(iiwa_status_sub_3->get_output_port(),
                  iiwa_status_rec_3->get_input_port());
  builder.Connect(iiwa_status_rec_3->get_position_measured_output_port(),
                  iiwa_3_mux->get_input_port(iiwa_3_id));
  builder.Connect(iiwa_time_sender_3->get_output_port(),
                  iiwa_time_pub_3->get_input_port());
  #endif
  #if USE_WSG_3
  builder.Connect(wsg_status_sub_3->get_output_port(),
                  wsg_status_rec_3->get_status_input_port());
  builder.Connect(wsg_status_rec_3->get_gripper_position_output_port(),
                  iiwa_3_mux->get_input_port(wsg_3_id));
  // builder.Connect(wsg_time_sender_3->get_output_port(),
  //                 wsg_time_pub_3->get_input_port());
  #endif
  #if USE_CASSIE
  builder.Connect(cassie_state_subscriber->get_output_port(),
                  cassie_state_receiver->get_input_port());
  builder.Connect(cassie_state_receiver->get_position_measured_output_port(),
                  cassie_to_pose->get_input_port());
  builder.Connect(cassie_time_sender->get_output_port(),
                  cassie_time_pub->get_input_port());
  #endif
  #if USE_ATHENA_CASSIE
  builder.Connect(athena_cassie_state_subscriber->get_output_port(),
                  athena_cassie_state_receiver->get_input_port());
  builder.Connect(athena_cassie_state_receiver->get_position_measured_output_port(),
                  athena_cassie_to_pose->get_input_port());
  builder.Connect(athena_cassie_time_sender->get_output_port(),
                  athena_cassie_time_pub->get_input_port());
  #endif
  #if USE_QUADROTOR_TEAM
  builder.Connect(quadrotor_team_state_subscriber->get_output_port(),
                  quadrotor_team_state_receiver->get_input_port());
  builder.Connect(quadrotor_team_state_receiver->get_position_measured_output_port(),
                  quadrotor_team_to_pose->get_input_port());
  builder.Connect(quadrotor_team_time_sender->get_output_port(),
                  quadrotor_team_time_pub->get_input_port());
  #endif

  // Visualization connections
  builder.Connect(plant_states_mux->get_output_port(0),
                  robot_to_pose->get_input_port());
  builder.Connect(robot_to_pose->get_output_port(), 
                  scene_graph.get_source_pose_port(plant.get_source_id().value()));

  #if USE_CASSIE                
  builder.Connect(cassie_to_pose->get_output_port(), 
                  scene_graph.get_source_pose_port(plant2.get_source_id().value()));
  #endif
  #if USE_ATHENA_CASSIE                
  builder.Connect(athena_cassie_to_pose->get_output_port(), 
                  scene_graph.get_source_pose_port(plant3.get_source_id().value()));
  #endif
  #if USE_QUADROTOR_TEAM                
  builder.Connect(quadrotor_team_to_pose->get_output_port(), 
                  scene_graph.get_source_pose_port(plant6.get_source_id().value()));
  #endif
  #if USE_IIWA_2              
  builder.Connect(iiwa_2_mux->get_output_port(0),
                  iiwa_2_to_pose->get_input_port());
  builder.Connect(iiwa_2_to_pose->get_output_port(), 
                  scene_graph.get_source_pose_port(plant4.get_source_id().value()));
  #endif
  #if USE_IIWA_3              
  builder.Connect(iiwa_3_mux->get_output_port(0),
                  iiwa_3_to_pose->get_input_port());
  builder.Connect(iiwa_3_to_pose->get_output_port(), 
                  scene_graph.get_source_pose_port(plant5.get_source_id().value()));
  #endif
  drake::geometry::ConnectDrakeVisualizer(&builder, scene_graph);

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  // Setting initial positions
  #if USE_MULTI_OBJECT
  // Get object manipulation order
  const int num_of_objects = root["num_of_objects_1"].asInt();
  Eigen::VectorXd init_multi_object_state(7*num_of_objects);
  std::string temp = "";
  for(int j = 0; j < num_of_objects; j++)
  {
    temp = root["multi_object_order_1"][j].asString();
    for(int i = 0; i < 7; i++)
    {
      init_multi_object_state[7*j+i] = 
        root["multi_object_position_1"][temp][i].asDouble();
    }
  }
  multi_object_state_receiver->SetInitialPosition(init_multi_object_state);
  #endif
  #if USE_MULTI_OBJECT_2
  // Get object manipulation order
  const int num_of_objects_2 = root["num_of_objects_2"].asInt();
  Eigen::VectorXd init_multi_object_state_2(7*num_of_objects_2);
  std::string temp2 = "";
  for(int j = 0; j < num_of_objects_2; j++)
  {
    temp2 = root["multi_object_order_2"][j].asString();
    for(int i = 0; i < 7; i++)
    {
      init_multi_object_state_2[7*j+i] = 
        root["multi_object_position_2"][temp2][i].asDouble();
    }
  }
  multi_object_state_receiver_2->SetInitialPosition(init_multi_object_state_2);
  #endif


  #if USE_TURTLE
  Eigen::VectorXd init_turtle_state;
  init_turtle_state.resize(7);
  for(int i = 0; i < 7; i++)
  {
    init_turtle_state[i] = root["turtle_position"][i].asDouble();
  }
  turtle_state_receiver->SetInitialPosition(init_turtle_state);
  #endif

  #if USE_TURTLE_2
  Eigen::VectorXd init_turtle_2_state;
  init_turtle_2_state.resize(7);
  for(int i = 0; i < 7; i++)
  {
    init_turtle_2_state[i] = root["turtle_2_position"][i].asDouble();
  }
  turtle2_state_receiver->SetInitialPosition(init_turtle_2_state);
  #endif
  #if USE_CASSIE
  Eigen::VectorXd init_cassie_state;
  init_cassie_state.resize(27);
  for(int i = 0; i < 7; i++)
  {
    init_cassie_state[i] = root["cassie_position"][i].asDouble();
  }
  for(int i = 7; i< 27; i++)
  {
    init_cassie_state[i] = 0;
  }
  cassie_state_receiver->SetInitialPosition(init_cassie_state);
  #endif
  #if USE_ATHENA_CASSIE
  Eigen::VectorXd init_athena_cassie_state;
  init_athena_cassie_state.resize(82);
  for(int i = 0; i < 7; i++)
  {
    init_athena_cassie_state[i] = root["athena_cassie_position"][i].asDouble();
  }
  for(int i = 7; i< 82; i++)
  {
    init_athena_cassie_state[i] = 0;
  }
  athena_cassie_state_receiver->SetInitialPosition(init_athena_cassie_state);
  #endif
  #if USE_QUADROTOR_TEAM
  Eigen::VectorXd init_quadrotor_team_state;
  init_quadrotor_team_state.resize(35);
  for(int i = 0; i < 35; i++)
  {
    init_quadrotor_team_state[i] = root["quadrotor_team_position"][i].asDouble();
  }
  quadrotor_team_state_receiver->SetInitialPosition(init_quadrotor_team_state);

  #endif



  // Set up simulator.
  auto simulator = std::make_unique<Simulator<double>>(*diagram, std::move(context));
  systems::Context<double>& root_context = simulator->get_mutable_context();

  // Context: 3 abs, 0 cont, 0 dis
  drake::log()->info("context absStates: {}",root_context.num_abstract_states());
  drake::log()->info("context contStates: {}",root_context.num_continuous_states());
  drake::log()->info("context disStates: {}",root_context.num_discrete_state_groups());

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

}  // namespace collaboration_station
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "Visualizer for robot collaboration simulation");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::examples::collaboration_station::DoMain();
  return 0;
}

