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
#include <fstream>
#include <jsoncpp/json/json.h>

#include "drake/common/find_resource.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/collaboration_station/kuka/high_level_planner.h"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_object_status.hpp"
#include "drake/lcmt_object_target.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/manipulation/util/move_ik_demo_base.h"
#include "drake/math/rigid_transform.h"

DEFINE_string(urdf, "", "Name of urdf to load");

namespace drake {
namespace examples {
namespace collaboration_station {
namespace {

using manipulation::util::MoveIkDemoBase;
using drake::examples::collaboration_station::HighLevelPlanner;
const char* const kLcmIiwaStatusChannel = "IIWA_STATUS";
const char* const kLcmWsgStatusChannel = "WSG_STATUS";
const char* const kLcmIiwa2StatusChannel = "IIWA_STATUS_2";
const char* const kLcmWsg2StatusChannel = "WSG_STATUS_2";
const char* const kLcmMultiObjectStatusChannel = "MULTI_OBJECT_STATUS";
const char* const kLcmObjectTargetChannel = "OBJECT_TARGET";
const char* const kLcmPlanChannel = "COMMITTED_ROBOT_PLAN";
const char* const kLcmWsgPlanChannel = "WSG_PLAN";

#define DEBUG_MODE true

class MoveDemoRunner {
 public:
  MoveDemoRunner() {
    lcm_.subscribe(kLcmIiwaStatusChannel,
                    &MoveDemoRunner::HandleIiwaStatus, this);
    lcm_.subscribe(kLcmWsgStatusChannel,
                    &MoveDemoRunner::HandleWsgStatus, this);
    lcm_.subscribe(kLcmIiwa2StatusChannel,
                    &MoveDemoRunner::HandleIiwa2Status, this);
    lcm_.subscribe(kLcmWsg2StatusChannel,
                    &MoveDemoRunner::HandleWsg2Status, this);
    lcm_.subscribe(kLcmMultiObjectStatusChannel,
                    &MoveDemoRunner::HandleMultiObjectStatus, this);
  }
  
  void Run() {
    Json::Value root = JsonParser();

    //Initialize HighLevelPlanner
    Vector6<double> iiwa_pose;
    iiwa_pose << root["iiwa_pose"][0].asDouble(),root["iiwa_pose"][1].asDouble(),
                 root["iiwa_pose"][2].asDouble(),root["iiwa_pose"][3].asDouble(),
                 root["iiwa_pose"][4].asDouble(),root["iiwa_pose"][5].asDouble();
    HighLevelPlanner planner1(iiwa_pose, 
      "drake/examples/collaboration_station/pose_list.json",0);

    //Initialize HighLevelPlanner2
    Vector6<double> iiwa_2_pose;
    iiwa_2_pose << root["iiwa_pose_2"][0].asDouble(),root["iiwa_pose_2"][1].asDouble(),
                   root["iiwa_pose_2"][2].asDouble(),root["iiwa_pose_2"][3].asDouble(),
                   root["iiwa_pose_2"][4].asDouble(),root["iiwa_pose_2"][5].asDouble();
    HighLevelPlanner planner2(iiwa_2_pose,
      "drake/examples/collaboration_station/pose_list_2.json",1);

    MoveIkDemoBase demo(
        !FLAGS_urdf.empty() ? FLAGS_urdf : FindResourceOrThrow(root["robot_iiwa_path"].asString()),
        !FLAGS_urdf.empty() ? FLAGS_urdf : FindResourceOrThrow(root["robot_wsg_path"].asString()),
        "iiwa_link_0", "wsg_grab_point", 100
    );

    VectorX<double> vel_limits(9);  // wsg vel limit is 0.42m/s
    vel_limits << examples::kuka_iiwa_arm::get_iiwa_max_joint_velocities(), 0.21, 0.21;
    demo.set_joint_velocity_limits(vel_limits);

    int object_num = -1;
    bool new_plan = true;
    bool use_planner_1 = true;
    std::string task = "";
    std::string chosen_plan = "";
    lcmt_object_target object_target;
    object_target.target = -1;
    object_target.roll_angle = 0;
    object_target.pitch_angle = 0;
    object_target.yaw_angle = 0;
    num_of_objects_ = root["num_of_objects"].asDouble();
    multi_object_states_ = VectorX<double>::Zero(num_of_objects_*7);
    Vector6<double> newObj;
    Eigen::VectorXd robot_q(9);
    target_object_states_ = Vector6<double>::Zero(6);
    multi_object_ready_to_spawn_ = false;
    std::vector<int> object_correct_order;
    std::vector<std::string> object_list = planner1.GetObjectList();
    handle_iiwa_state_called_ = false;
    handle_iiwa_2_state_called_ = false;

    // Get object manipulation order
    for(int i = 0; i < num_of_objects_; i++){
      std::string temp = object_list[i];
      for (int j = 0; j < num_of_objects_; j++){  
        std::string tmp2 = root["multi_object_order"][j].asString();
        if (tmp2.compare(temp) == 0){   
          object_correct_order.push_back(j);
          break;                   
        }
      }
    }
    drake::log()->info("task name: {}", planner1.GetCurrentTask());

    while(true){

      while (0 == lcm_.handleTimeout(10)) { }

      if(handle_iiwa_state_called_ && !handle_iiwa_2_state_called_){
        use_planner_1 = true;
        drake::log()->info("planner 1");
                        // DRAKE_DEMAND(false);

      }
      else if(!handle_iiwa_state_called_ && handle_iiwa_2_state_called_){
        use_planner_1 = false;
        drake::log()->info("planner 2");
      }
      else{ continue; }

      if(use_planner_1){
        task = planner1.GetCurrentTask();
        object_num = planner1.GetObjectNum();
      }
      else{
        task = planner2.GetCurrentTask();
        object_num = planner2.GetObjectNum();
      }

      drake::log()->info("task name is: {}", task);
      drake::log()->info("object num is: {}", object_num);

      // Calls kuka_position_publisher to send object status
      object_target.target = object_num;
      lcm_.publish(kLcmObjectTargetChannel, &object_target);

      if(multi_object_ready_to_spawn_){
        const int correct_object_num = object_correct_order[object_num];
        // Change format from object status to object states
        const Eigen::Quaternion<double> obj_quat(
                    multi_object_states_[correct_object_num*7+0],
                    multi_object_states_[correct_object_num*7+1],
                    multi_object_states_[correct_object_num*7+2],
                    multi_object_states_[correct_object_num*7+3]);
        const math::RollPitchYaw<double> rpy(obj_quat);

        #if DEBUG_MODE
          // for(int i = 0; i < 3; i++){
          // drake::log()->info("multi states: {}\n {}\n{}\n{}\n{}\n{}\n{}\n{} ",
          //                     i,
          //                     multi_object_states_[i*7+0],
          //                     multi_object_states_[i*7+1],
          //                     multi_object_states_[i*7+2],
          //                     multi_object_states_[i*7+3],
          //                     multi_object_states_[i*7+4],
          //                     multi_object_states_[i*7+5],
          //                     multi_object_states_[i*7+6]);
          // }
        #endif
        target_object_states_[0] = rpy.roll_angle();
        target_object_states_[1] = rpy.pitch_angle();
        target_object_states_[2] = rpy.yaw_angle();
        target_object_states_[3] = multi_object_states_[correct_object_num*7+4];
        target_object_states_[4] = multi_object_states_[correct_object_num*7+5];
        target_object_states_[5] = multi_object_states_[correct_object_num*7+6];

        #if DEBUG_MODE
          drake::log()->info("target obj state: {}\n{}\n{}\n{}\n{}\n{} ",
          target_object_states_[0],
          target_object_states_[1],
          target_object_states_[2],
          target_object_states_[3],
          target_object_states_[4],
          target_object_states_[5]);
        #endif
        // Sets states to corresponding object
        if(use_planner_1) { planner1.Update(target_object_states_); }
        else{ planner2.Update(target_object_states_); }
        multi_object_ready_to_spawn_ = false;
      }

      // If task is new_object, plan does not get published
      if(task.compare("new_object") == 0){
        object_target.target = object_num;
        std::string object_name = "";
        if(use_planner_1){
          newObj = planner1.NewObject();
          object_target.grasp_height = planner1.GetObjectGraspHeight();  
          chosen_plan = planner1.GetChosenPlan();
          object_name = planner1.GetObjectName();
        }
        else{
          newObj = planner2.NewObject();
          object_target.grasp_height = planner2.GetObjectGraspHeight();  
          chosen_plan = planner2.GetChosenPlan();
          object_name = planner2.GetObjectName();
        }
        const Eigen::Quaternion<double> obj_quat(
          root["multi_object_position"][object_list[object_num]][0].asDouble(),
          root["multi_object_position"][object_list[object_num]][1].asDouble(),
          root["multi_object_position"][object_list[object_num]][2].asDouble(),
          root["multi_object_position"][object_list[object_num]][3].asDouble()
        );
        const math::RollPitchYaw<double> rpy(obj_quat);

        object_target.roll_angle = rpy.roll_angle();
        object_target.pitch_angle = rpy.pitch_angle();
        object_target.yaw_angle = rpy.yaw_angle();

        drake::log()->info(
        "cahnge of roll: {}\ncahnge of pitch: {}\ncahnge of yaw: {}",
        rpy.roll_angle(), rpy.pitch_angle(), rpy.yaw_angle());

        #if DEBUG_MODE
        drake::log()->info("new objects: ");
        for(int i = 0; i < 6; i++){
          drake::log()->info("{}:{}",i,newObj[i]);
        }
        if(use_planner_1){
          drake::log()->info("printing: {}", planner1.Print());
          drake::log()->info("Object name and num: {} {}", planner1.GetObjectName(), object_num);
        } else{
          drake::log()->info("printing: {}", planner2.Print());
          drake::log()->info("Object name and num: {} {}", planner2.GetObjectName(), object_num);
        }
        #endif
      }
      // Send plan if task is not "new_object"
      else{
        // Cannot pick up object without target    
        DRAKE_DEMAND(!(task.compare("pick") == 0 && object_num == -1));
        if(handle_iiwa_state_called_){
          for(int i = 0; i < 7; i++){
              robot_q[i] = iiwa_state_[i]; 
          }
          robot_q[7] = wsg_state_/2;
          robot_q[8] = -wsg_state_/2;
        }
        else if(handle_iiwa_2_state_called_){
          for(int i = 0; i < 7; i++){
              robot_q[i] = iiwa_2_state_[i]; 
          }
          robot_q[7] = wsg_2_state_/2;
          robot_q[8] = -wsg_2_state_/2;
        }

        demo.HandleStatus(robot_q);

        if (new_plan) {  
          drake::log()->info("new plan ");

          #if DEBUG_MODE
            drake::log()->info("iiwa next move:");
            math::RigidTransformd po = 
              use_planner_1 ? planner1.NextIiwaMovement() : planner2.NextIiwaMovement();
            Vector3<double> cc = po.translation();
            for(int i = 0 ; i < 3; i++){
              drake::log()->info(cc[i]);
            }
            drake::log()->info("end");
          #endif

          std::optional<robotlocomotion::robot_plan_t> plan;
          if(use_planner_1){
            plan = demo.Plan(planner1.NextIiwaMovement(),planner1.NextWsgMovement());
          } else{
            plan = demo.Plan(planner2.NextIiwaMovement(),planner2.NextWsgMovement());
          }
          if (plan.has_value()) {
            lcm_.publish(kLcmPlanChannel, &plan.value());
          }
          new_plan = false;
        }
      }

      if(demo.IsFinished()){
        if(use_planner_1){
          if(planner1.HasMorePlan()) {
            new_plan = true;
            drake::log()->info("moving to new plannn1: {}",
              planner1.MoveToNewPlan());
          }
          else if(planner1.NextObject()){
            drake::log()->info("moving to NEW OBJECT!!: {}",planner1.GetObjectNum());
            //Reset everything
            new_plan = true;
          }
          else break; //End simulation
        }
        else{
          if(planner2.HasMorePlan()) {
            new_plan = true;            
            drake::log()->info("moving to new plannn2: {}",
              planner2.MoveToNewPlan());
          }
          else if(planner2.NextObject()){
            drake::log()->info("moving to NEW OBJECT!!: {}",planner2.GetObjectNum());
            //Reset everything
            new_plan = true;
          }
          else break; //End simulation
        }
      }
    handle_iiwa_state_called_ = false;
    handle_iiwa_2_state_called_ = false;
    }
  }

 private:
  void HandleIiwaStatus(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_iiwa_status* status) {
    handle_iiwa_state_called_ = true;
    iiwa_state_.resize(7);
    for(int i = 0; i < 7; i++){
      iiwa_state_[i] = status->joint_position_measured[i];
    }
  }

  void HandleWsgStatus(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_schunk_wsg_status* status){
    wsg_state_ = status->actual_position_mm;                  
  }
  
  void HandleIiwa2Status(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_iiwa_status* status) {
    handle_iiwa_2_state_called_ = true;
    iiwa_2_state_.resize(7);
    for(int i = 0; i < 7; i++){
      iiwa_2_state_[i] = status->joint_position_measured[i];
    }
  }

  void HandleWsg2Status(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_schunk_wsg_status* status){
    wsg_2_state_ = status->actual_position_mm;                  
  }

  void HandleMultiObjectStatus(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_object_status* status){
    for(int i = 0; i < num_of_objects_ * 7; i++){
      multi_object_states_[i] = status->joint_position_measured[i];
    }
drake::log()->info("Handler: multi states:\n {}\n {}\n {}\n {} \n{}\n {}\n {} ",
                    multi_object_states_[0],
                    multi_object_states_[1],
                    multi_object_states_[2],
                    multi_object_states_[3],
                    multi_object_states_[4],
                    multi_object_states_[5],
                    multi_object_states_[6]);

    multi_object_ready_to_spawn_ = true;
  }

  Json::Value JsonParser(){
    Json::Reader reader;
    Json::Value root;
    drake::log()->info("json parser activated.");
    
    // User does not have to specify /home/your_name/ anymore!
    std::ifstream myfile(FindResourceOrThrow("drake/examples/"
                          "collaboration_station/config.json"));
    myfile >> root;
    drake::log()->info("reading json config file completed.");
    return root;

    // Example of accessing (DO NOT ERASE BELOW)
    // cout << root["name"].asString() << endl;
  }

  ::lcm::LCM lcm_;
  int num_of_objects_;
  VectorX<double> multi_object_states_;
  Vector6<double> target_object_states_;
  VectorX<double> iiwa_state_;
  double wsg_state_;
  VectorX<double> iiwa_2_state_;
  double wsg_2_state_;
  bool multi_object_ready_to_spawn_;
  bool handle_iiwa_state_called_;
  bool handle_iiwa_2_state_called_;
};

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::examples::collaboration_station::MoveDemoRunner runner;
  runner.Run();
  drake::log()->info("move_iiwa_ee.cc has finished");
  return 0;
}
