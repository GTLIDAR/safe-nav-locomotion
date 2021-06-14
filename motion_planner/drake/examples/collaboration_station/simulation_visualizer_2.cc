#include <memory>

#include <gflags/gflags.h>
#include <fstream>
#include <jsoncpp/json/json.h>
#include <vector>

#include "drake/common/find_resource.h"
#include "drake/examples/athena_cassie/athena_cassie.h"
#include "drake/examples/athena_cassie/athena_cassie_lcm.h"
#include "drake/examples/A1/a1_lcm.h"

#include "drake/examples/collaboration_station/robot_time_sender.h"
#include "drake/examples/kuka_iiwa_arm/iiwa.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_cassie_command.hpp"
#include "drake/lcmt_cassie_status.hpp"

#include "drake/lcmt_robot_time.hpp"
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

using drake::examples::kuka_iiwa_arm::IiwaStatusReceiver;
using drake::examples::kuka_iiwa_arm::IiwaStatusSender;
using drake::examples::kuka_iiwa_arm::kIiwaLcmStatusPeriod;
using drake::manipulation::kuka_iiwa::kIiwaArmNumJoints;

using drake::manipulation::schunk_wsg::SchunkWsgStatusSender;
using drake::manipulation::schunk_wsg::SchunkWsgStatusReceiver;
using drake::manipulation::schunk_wsg::kSchunkWsgLcmStatusPeriod;
using drake::manipulation::schunk_wsg::kSchunkWsgNumPositions;

// using drake::examples::A1::A1StatusReceiver;
// using drake::examples::A1::kA1LcmStatusPeriod;

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
#define USE_ATHENA_CASSIE true
#define USE_KUKA true
#define USE_WSG true
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

  // For kuka
  MultibodyPlant<double> plant2(1e-5);
  plant2.set_name("plant2");
  Parser parser2(&plant2, &scene_graph);

  // // For a1
  // MultibodyPlant<double> plant3(1e-5);
  // plant3.set_name("plant3");
  // Parser parser3(&plant3, &scene_graph);
  // // For a1
  // MultibodyPlant<double> plant4(1e-5);
  // plant4.set_name("plant4");
  // Parser parser4(&plant4, &scene_graph);

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
  multibody::ModelInstanceIndex multi_static_boxes_instance2 = parser.AddModelFromFile(
    FindResourceOrThrow(root["multi_static_boxes2_path"].asString()));
    
  // multibody::ModelInstanceIndex a1_instance = parser3.AddModelFromFile(
  //   FindResourceOrThrow(root["robot_a1_path"].asString()));
  // multibody::ModelInstanceIndex a1_2_instance = parser4.AddModelFromFile(
  //   FindResourceOrThrow(root["robot_a1_2_path"].asString()));

  // Load Forklift
  // multibody::ModelInstanceIndex forklift_instance = parser.AddModelFromFile(
  //   FindResourceOrThrow(root["forklift_path"].asString()));
  // Load single box
  multibody::ModelInstanceIndex single_box_instance = parser.AddModelFromFile(
    FindResourceOrThrow(root["single_box_path"].asString()));


  #endif

  #if USE_STATIC_OPEN_BOX
  // Load Static Open Box
  multibody::ModelInstanceIndex static_open_box_instance = parser.AddModelFromFile(
    FindResourceOrThrow(root["static_box_path"].asString()));
  // Load Static Open Box2
  multibody::ModelInstanceIndex static_open_box_2_instance = parser.AddModelFromFile(
    FindResourceOrThrow(root["static_box_2_path"].asString()));
  #endif

  
  #if USE_ATHENA_CASSIE
  multibody::ModelInstanceIndex athena_cassie_instance = parser.AddModelFromFile(
    FindResourceOrThrow(root["robot_athena_cassie_path"].asString()));
  multibody::ModelInstanceIndex mobile_box_instance = parser.AddModelFromFile(
    FindResourceOrThrow(root["mobile_box_path"].asString()));
  #endif


  #if USE_KUKA
  multibody::ModelInstanceIndex iiwa_instance = parser2.AddModelFromFile(
    FindResourceOrThrow(root["robot_iiwa_path"].asString()));
  #endif
  #if USE_WSG
  multibody::ModelInstanceIndex connector_instance = parser2.AddModelFromFile(
    FindResourceOrThrow(root["robot_connector_path"].asString()));
  multibody::ModelInstanceIndex wsg_instance = parser2.AddModelFromFile(
    FindResourceOrThrow(root["robot_wsg_path"].asString()));
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

  const RigidTransform<double> X_multi_static_boxes2(    
    RollPitchYaw<double>(0,0,0),
    Vector3d(0,0,0.4));
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(
      "base", multi_static_boxes_instance2), X_multi_static_boxes2);

  // Weld forklift
  // const RigidTransform<double> X_forklift(    
  //   RollPitchYaw<double>(0,0,0),
  //   Vector3d(0,0,0));
  // plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(
  //     "base", forklift_instance), X_forklift);

  // Weld single box
  const RigidTransform<double> X_single_box(    
    RollPitchYaw<double>(0,0,0),
    Vector3d(0,0,0));
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(
      "base", single_box_instance), X_single_box);




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

#if USE_KUKA
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
  plant2.WeldFrames(plant2.world_frame(), plant2.GetFrameByName(
      "iiwa_link_0", iiwa_instance), X_WI);
  #endif
  #if USE_WSG
    #if USE_KUKA   
    const multibody::Frame<double>& link7 =
        plant2.GetFrameByName("iiwa_link_7", iiwa_instance);
    const RigidTransform<double> X_7C(
        RollPitchYaw<double>(0, 0, M_PI_2), Vector3d(0, 0, 0.045));
    plant2.WeldFrames(link7, plant2.GetFrameByName(
        "connector_link", connector_instance), X_7C);

    const RigidTransform<double> X_7G(
        RollPitchYaw<double>(M_PI_2, 0, M_PI_2), Vector3d(0, 0, 0.1));
    plant2.WeldFrames(link7, plant2.GetFrameByName(
        "body", wsg_instance), X_7G);
    #else
    drake::log()->info("If you want a WSG, you need an Iiwa");
    DRAKE_DEMAND(false);
    #endif
  #endif

  // const RigidTransform<double> X_WA1(
  //     RollPitchYaw<double>(
  //       root["a1_pose"][0].asDouble(),
  //       root["a1_pose"][1].asDouble(),
  //       root["a1_pose"][2].asDouble()),
  //     Vector3d(
  //       root["a1_pose"][3].asDouble(),
  //       root["a1_pose"][4].asDouble(),
  //       root["a1_pose"][5].asDouble())
  // );
  // plant3.WeldFrames(plant3.world_frame(), plant3.GetFrameByName(
  //     "base", a1_instance), X_WA1);

  // const RigidTransform<double> X_WA12(
  //     RollPitchYaw<double>(
  //       root["a1_pose2"][0].asDouble(),
  //       root["a1_pose2"][1].asDouble(),
  //       root["a1_pose2"][2].asDouble()),
  //     Vector3d(
  //       root["a1_pose2"][3].asDouble(),
  //       root["a1_pose2"][4].asDouble(),
  //       root["a1_pose2"][5].asDouble())
  // );
  // plant4.WeldFrames(plant4.world_frame(), plant4.GetFrameByName(
  //     "base", a1_2_instance), X_WA12);

  // Now the model is complete.
  plant.Finalize();
  plant2.Finalize();
  // plant3.Finalize();
  // plant4.Finalize();

 
  
// ------------------------ USEFUL VARIABLES ---------------------------------
  // std::vector<int> all_joints;
  // all_joints.resize(num_robots,0);

 
  #if USE_ATHENA_CASSIE
  const int athena_cassie_num_joints = plant.num_positions(athena_cassie_instance)+plant.num_positions(mobile_box_instance);
  #endif
  #if USE_KUKA
  //const int iiwa_num_joints = plant2.num_positions(iiwa_instance)+plant2.num_positions(wsg_instance);
  #endif
    // const int a1_num_joints = plant3.num_positions(a1_instance);

// ------------------------- SYSTEM CREATION ---------------------------------
  // Create LCM
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

 
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


#if USE_KUKA
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

  // auto a1_state_subscriber = builder.AddSystem(
  //     systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_cassie_status>(
  //         "A1_STATUS", lcm));
  // a1_state_subscriber->set_name("a1_state_subscriber");
  // auto a1_state_receiver =
  //     builder.AddSystem<A1StatusReceiver>(a1_num_joints);
  // a1_state_receiver->set_name("a1_state_receiver");

  // auto a1_time_pub = builder.AddSystem(
  //     systems::lcm::LcmPublisherSystem::Make<drake::lcmt_robot_time>(
  //         "A1_TIME", lcm, kA1LcmStatusPeriod /* publish period */));
  // a1_time_pub->set_name("a1_time_publisher");
  // auto a1_time_sender = builder.AddSystem<RobotTimeSender>();
  // a1_time_sender->set_name("a1_time_sender");

  // auto a1_2_state_subscriber = builder.AddSystem(
  //     systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_cassie_status>(
  //         "A1_2_STATUS", lcm));
  // a1_2_state_subscriber->set_name("a1_2_state_subscriber");
  // auto a1_2_state_receiver =
  //     builder.AddSystem<A1StatusReceiver>(a1_num_joints);
  // a1_2_state_receiver->set_name("a1_2_state_receiver");

  // auto a1_2_time_pub = builder.AddSystem(
  //     systems::lcm::LcmPublisherSystem::Make<drake::lcmt_robot_time>(
  //         "A1_2_TIME", lcm, kA1LcmStatusPeriod /* publish period */));
  // a1_2_time_pub->set_name("a1_2_time_publisher");
  // auto a1_2_time_sender = builder.AddSystem<RobotTimeSender>();
  // a1_2_time_sender->set_name("a1_2_time_sender");

  // // Plug in all plant state quantities
  // std::vector<int> mux_sizes;
  // for(unsigned int i = 0; i < num_robots; i++)
  // {
  //   mux_sizes.push_back(all_joints[i]);
  // }

  // DRAKE_DEMAND(mux_sizes.size() == num_robots); // Subtract out cassie and athena_cassie
  // auto plant_states_mux = builder.AddSystem<systems::Multiplexer>(mux_sizes);    

  // // Robot states to geometry visualization system
  // auto robot_to_pose =
  //       builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant);

  
  #if USE_ATHENA_CASSIE
  auto athena_cassie_to_pose =
        builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant);
  #endif
  #if USE_KUKA
    auto iiwa_to_pose =
        builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant2);

  std::vector<int> iiwa_mux_sizes;
  iiwa_mux_sizes.resize(2);
  iiwa_mux_sizes[0] = 7;
  iiwa_mux_sizes[1] = 2;
  auto iiwa_mux = 
        builder.AddSystem<systems::Multiplexer>(iiwa_mux_sizes);  
  #endif

  // auto a1_to_pose =
  //     builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant3);
  // auto a1_2_to_pose =
  //     builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant4);

  #if USE_ATHENA_CASSIE
  builder.Connect(athena_cassie_state_subscriber->get_output_port(),
                  athena_cassie_state_receiver->get_input_port());
  builder.Connect(athena_cassie_state_receiver->get_position_measured_output_port(),
                  athena_cassie_to_pose->get_input_port());
  builder.Connect(athena_cassie_time_sender->get_output_port(),
                  athena_cassie_time_pub->get_input_port());
  #endif
  #if USE_KUKA
  builder.Connect(iiwa_status_sub->get_output_port(),
                  iiwa_status_rec->get_input_port());
  builder.Connect(iiwa_status_rec->get_position_measured_output_port(),
                  iiwa_mux->get_input_port(0));
  builder.Connect(iiwa_time_sender->get_output_port(),
                  iiwa_time_pub->get_input_port());
  #endif
  #if USE_WSG
  builder.Connect(wsg_status_sub->get_output_port(),
                  wsg_status_rec->get_status_input_port());
  builder.Connect(wsg_status_rec->get_gripper_position_output_port(),
                  iiwa_mux->get_input_port(1));
  #endif

  // builder.Connect(a1_state_subscriber->get_output_port(),
  //                 a1_state_receiver->get_input_port());
  // builder.Connect(a1_state_receiver->get_position_measured_output_port(),
  //                 a1_to_pose->get_input_port());
  // builder.Connect(a1_time_sender->get_output_port(),
  //                 a1_time_pub->get_input_port());

  // builder.Connect(a1_2_state_subscriber->get_output_port(),
  //                 a1_2_state_receiver->get_input_port());
  // builder.Connect(a1_2_state_receiver->get_position_measured_output_port(),
  //                 a1_2_to_pose->get_input_port());
  // builder.Connect(a1_2_time_sender->get_output_port(),
  //                 a1_2_time_pub->get_input_port());

  #if USE_ATHENA_CASSIE                
  builder.Connect(athena_cassie_to_pose->get_output_port(), 
                  scene_graph.get_source_pose_port(plant.get_source_id().value()));
  #endif
  #if USE_KUKA
  builder.Connect(iiwa_mux->get_output_port(0),
                  iiwa_to_pose->get_input_port());
  builder.Connect(iiwa_to_pose->get_output_port(), 
                  scene_graph.get_source_pose_port(plant2.get_source_id().value()));
  #endif

  // builder.Connect(a1_to_pose->get_output_port(), 
  //                 scene_graph.get_source_pose_port(plant3.get_source_id().value()));
  // builder.Connect(a1_2_to_pose->get_output_port(), 
  //                 scene_graph.get_source_pose_port(plant4.get_source_id().value()));

  drake::geometry::ConnectDrakeVisualizer(&builder, scene_graph);

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  // Setting initial positions
  
  #if USE_ATHENA_CASSIE
  Eigen::VectorXd init_athena_cassie_state;
  init_athena_cassie_state.resize(82);
  for(int i = 0; i < 82; i++){
    init_athena_cassie_state[i] = root["athena_cassie_position6"][i].asDouble();
  }
  athena_cassie_state_receiver->SetInitialPosition(init_athena_cassie_state);
  #endif
   
  #if USE_KUKA
  Eigen::VectorXd init_iiwa_state;
  init_iiwa_state.resize(7);
  for(int i = 0; i < 7; i++){
    init_iiwa_state[i] = root["iiwa_position"][i].asDouble();
  }
  iiwa_status_rec->SetInitialPosition(init_iiwa_state);
  #endif

 
  // Eigen::VectorXd init_a1_state;
  // init_a1_state.resize(12);
  // for(int i = 0; i < 12; i++){
  //   init_a1_state[i] = root["a1_position"][i].asDouble();
  // }
  // a1_state_receiver->SetInitialPosition(init_a1_state);
 
  // Eigen::VectorXd init_a1_2_state;
  // init_a1_2_state.resize(12);
  // for(int i = 0; i < 12; i++){
  //   init_a1_2_state[i] = root["a1_2_position"][i].asDouble();
  // }
  // a1_2_state_receiver->SetInitialPosition(init_a1_2_state);
 




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

