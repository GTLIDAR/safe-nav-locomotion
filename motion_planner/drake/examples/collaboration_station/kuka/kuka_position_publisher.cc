/// @file
///
/// kuka_plan_runner is designed to wait for LCM messages constraining
/// a robot_plan_t message, and then execute the plan on an iiwa arm
/// (also communicating via LCM using the
/// lcmt_iiwa_command/lcmt_iiwa_status messages).
///
/// When a plan is received, it will immediately begin executing that
/// plan on the arm (replacing any plan in progress).
///
/// If a stop message is received, it will immediately discard the
/// current plan and wait until a new plan is received.

#include <iostream>
#include <memory>
#include <fstream>
#include <jsoncpp/json/json.h>

#include "lcm/lcm-cpp.hpp"
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/lcmt_cassie_status.hpp"
#include "drake/lcmt_robot_time.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/lcmt_object_status.hpp"
#include "drake/lcmt_object_target.hpp"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;
using drake::Vector1d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace drake {
namespace examples {
namespace collaboration_station {
namespace {

const char* const kLcmMultiObjectChannel = "MULTI_OBJECT_STATUS";
const char* const kLcmTargetChannel = "OBJECT_TARGET";

const char* const kLcmSchunkChannel = "WSG_STATUS";
const char* const kLcmIiwaChannel = "IIWA_STATUS";
const char* const kLcmTimeChannel = "IIWA_TIME";
const char* const kLcmPlanChannel = "COMMITTED_ROBOT_PLAN";

const char* const kLcmSchunk2Channel = "WSG_STATUS_2";
const char* const kLcmIiwa2Channel = "IIWA_STATUS_2";
const char* const kLcmTime2Channel = "IIWA_TIME_2";

const char* const kLcmBoxChannel = "ATHENA_CASSIE_STATUS";
const char* const kLcmStopChannel = "STOP";
const int kNumJoints = 7;

using trajectories::PiecewisePolynomial;
typedef PiecewisePolynomial<double> PPType;
typedef Polynomial<double> PPPoly;
typedef PPType::PolynomialMatrix PPMatrix;

#define DEBUG_MODE true

class RobotPlanRunner {
 public:
  /// plant is aliased
  explicit RobotPlanRunner(const multibody::MultibodyPlant<double>& plant,
                          const multibody::MultibodyPlant<double>& plant2)
      : plant_(plant), plant_2_(plant2), plan_number_(0),plan_number_2_(0) {
    lcm_.subscribe(kLcmTimeChannel,
                    &RobotPlanRunner::HandleTime, this);
    lcm_.subscribe(kLcmPlanChannel,
                    &RobotPlanRunner::HandlePlan, this);
    lcm_.subscribe(kLcmTime2Channel,
                    &RobotPlanRunner::HandleTime2, this);
    lcm_.subscribe(kLcmStopChannel,
                    &RobotPlanRunner::HandleStop, this);
    lcm_.subscribe(kLcmTargetChannel,
                    &RobotPlanRunner::HandleTarget, this);
    lcm_.subscribe(kLcmBoxChannel,
                    &RobotPlanRunner::HandleBox, this);
  }

  void Run(Json::Value root, Json::Value object_list) {
    int cur_plan_number = plan_number_;
    int cur_plan_number_2 = plan_number_2_;
    int64_t cur_time_us = -1;
    int64_t start_time_us = -1;
    int64_t cur_time_us_2 = -1;
    int64_t start_time_us_2 = -1;

    context_ = plant_.CreateDefaultContext();
    context_2_ = plant_2_.CreateDefaultContext();

    // Initialize the timestamp to an invalid number so we can detect
    // the first message.
    iiwa_time_.utime = cur_time_us;

    lcmt_iiwa_status iiwa_status;
    iiwa_status.utime = iiwa_time_.utime;
    iiwa_status.num_joints = kNumJoints;
    iiwa_status.joint_position_measured.resize(kNumJoints, 0.);
    iiwa_status.joint_position_commanded.resize(kNumJoints, 0.);
    iiwa_status.joint_velocity_estimated.resize(kNumJoints, 0.);

    iiwa_status.joint_torque_measured.resize(kNumJoints, 0.);
    iiwa_status.joint_torque_commanded.resize(kNumJoints, 0.);
    iiwa_status.joint_torque_external.resize(kNumJoints, 0.);
    iiwa_status.joint_position_ipo.resize(kNumJoints, 0.);

    lcmt_schunk_wsg_status wsg_status;
    wsg_status.actual_position_mm = 0;
    wsg_status.actual_speed_mm_per_s = 0;
    wsg_status.utime = iiwa_time_.utime;

    const int num_of_objects = root["num_of_objects"].asInt();
    const int num_obj_joints = num_of_objects*7;
    // lcmt_object_status object_state;
    object_state_.num_joints = num_obj_joints;
    object_state_.joint_position_measured.resize(num_obj_joints, 0.);   
    object_state_.joint_velocity_estimated.resize(num_obj_joints, 0.);
    object_state_.joint_position_commanded.resize(num_obj_joints, 0.);
    object_state_.joint_position_ipo.resize(num_obj_joints, 0.);
    object_state_.joint_torque_measured.resize(num_obj_joints, 0.);
    object_state_.joint_torque_commanded.resize(num_obj_joints, 0.);
    object_state_.joint_torque_external.resize(num_obj_joints, 0.);

    athena_cassie_upstairs_box_x_ = root["box_upstairs_placement"][3].asDouble();
    athena_cassie_upstairs_box_y_ = root["box_upstairs_placement"][4].asDouble();
    athena_cassie_upstairs_box_z_ = root["box_upstairs_placement"][5].asDouble();
    athena_cassie_downstairs_box_x_ = root["box_downstairs_placement"][3].asDouble();
    athena_cassie_downstairs_box_y_ = root["box_downstairs_placement"][4].asDouble();
    athena_cassie_downstairs_box_z_ = root["box_downstairs_placement"][5].asDouble();

    bool object_grabbed = false;
    bool init_rot_mat_set = false;
    bool has_been_grabbed = false;
    send_to_kuka_1_ = false;
    send_to_kuka_2_ = false;
    int correct_order = -1;
    Eigen::Vector3d current_pos;
    Eigen::VectorXd q(kNumJoints+2);
    Eigen::Quaternion<double> quat;
    math::RigidTransform<double> current_link_pose;
    math::RollPitchYaw<double> rpy(Vector3<double>::Zero(3));
    math::RollPitchYaw<double> temp_rpy(Vector3<double>::Zero(3));

    std::vector<int> object_correct_order;
    math::RotationMatrixd rot_mat_init;
    math::RotationMatrixd rot_mat_obj_init;
    object_target_ = -1;
    int prev_target = -2;

    // Why do we have to do this? 
    current_pos[0] = 0;
    current_pos[1] = 0;
    current_pos[2] = 0;


    // Get object manipulation order
    for(int i = 0; i < num_of_objects; i++){
      std::string temp = object_list["list"][i].asString();
      object_order_.push_back(temp);
      for (int j = 0; j < num_of_objects; j++){  
        std::string tmp2 = root["multi_object_order"][j].asString();
        if (tmp2.compare(temp) == 0){   
          object_correct_order.push_back(j);
          break;                   
        }
      }
    }

    // Set object state in the manipulation order
    for(int j = 0; j < num_of_objects; j++){
      for(int i = 0; i < 7; i++)  {
        std::string tmp2 = root["multi_object_order"][j].asString();
        object_state_.joint_position_measured[j*7+i] =
         root["multi_object_position"][tmp2][i].asDouble();   
      }
    }

    //Printing
    for(int i = 0; i < num_of_objects; i++){
        drake::log()->info("obj state for {}\n{}\n{}\n{}\n{}\n{}\n{}\n{}\n", 
          i,
          object_state_.joint_position_measured[i*7+0],
          object_state_.joint_position_measured[i*7+1],
          object_state_.joint_position_measured[i*7+2],
          object_state_.joint_position_measured[i*7+3],
          object_state_.joint_position_measured[i*7+4],
          object_state_.joint_position_measured[i*7+5],
          object_state_.joint_position_measured[i*7+6]);
    }

    while (true) {
      // Call lcm handle until at least one status message is processed.
      while (0 == lcm_.handleTimeout(10) || iiwa_time_.utime == -1) { }

      cur_time_us = iiwa_time_.utime;
      cur_time_us_2 = iiwa_2_time_.utime;

      if (plan_) {
        if (plan_number_ != cur_plan_number) {
          std::cout << "Starting new plan.1" << std::endl;
          start_time_us = cur_time_us;
          cur_plan_number = plan_number_;
        }
        const auto desired_next = plan_->value(
          static_cast<double>(cur_time_us - start_time_us) / 1e6);

        iiwa_status.utime = iiwa_time_.utime;
        for (int joint = 0; joint < kNumJoints; joint++) 
        {
          iiwa_status.joint_position_measured[joint] = desired_next(joint);
          iiwa_status.joint_position_commanded[joint] = desired_next(joint);
          q[joint] = desired_next(joint);
        }

        // Set wsg position in millimeters
        wsg_status.actual_position_mm = desired_next(7)-desired_next(8);
        DRAKE_DEMAND(wsg_status.actual_position_mm > 0);
        q[7] = desired_next(7);
        q[8] = desired_next(8);
        object_grabbed = (wsg_status.actual_position_mm < 90) ? true : false;
      }

      else if (plan_2_) {
        if (plan_number_2_ != cur_plan_number_2) {
          std::cout << "Starting new plan. 2" << std::endl;
          start_time_us_2 = cur_time_us_2;
          cur_plan_number_2 = plan_number_2_;
        }

        const auto desired_next_2 = plan_2_->value(
          static_cast<double>(cur_time_us_2 - start_time_us_2) / 1e6);

        iiwa_status.utime = iiwa_2_time_.utime;
        for (int joint = 0; joint < kNumJoints; joint++) 
        {
          iiwa_status.joint_position_measured[joint] = desired_next_2(joint);
          iiwa_status.joint_position_commanded[joint] = desired_next_2(joint);
          q[joint] = desired_next_2(joint);
        }

        // Set wsg position in millimeters
        wsg_status.actual_position_mm = desired_next_2(7)-desired_next_2(8);
        DRAKE_DEMAND(wsg_status.actual_position_mm > 0);
        q[7] = desired_next_2(7);
        q[8] = desired_next_2(8);
        object_grabbed = (wsg_status.actual_position_mm < 90) ? true : false;
      }

      // This is XOR
      if(!plan_ != !plan_2_){
        // When moving with object
          if(send_to_kuka_1_ && !send_to_kuka_2_){
            plant_.SetPositions(context_.get(), q);
            if(object_grasp_height_ < 0.05){
              current_link_pose = plant_.EvalBodyPoseInWorld(
                          *context_, plant_.GetBodyByName("wsg_grab_point"));
            }
            else{
              current_link_pose = plant_.EvalBodyPoseInWorld(
                          *context_, plant_.GetBodyByName("wsg_grab_point2"));
            }
            current_pos = current_link_pose.translation();
          }
          else if(!send_to_kuka_1_ && send_to_kuka_2_){
            DRAKE_DEMAND(false); 
            drake::log()->info("q: {}\n{}\n{}\n{}\n{}\n{}\n{}\n",
            q[0],q[1],q[2],q[3],q[4],q[5],q[6]);

            plant_2_.SetPositions(context_2_.get(), q);
            if(object_grasp_height_ < 0.05){
              current_link_pose = plant_2_.EvalBodyPoseInWorld(
                          *context_, plant_2_.GetBodyByName("wsg_grab_point"));
            }
            else{
              current_link_pose = plant_2_.EvalBodyPoseInWorld(
                          *context_, plant_2_.GetBodyByName("wsg_grab_point2"));
            }
            current_pos = current_link_pose.translation();
          }
          else{ DRAKE_DEMAND(false); }
        

        if(!object_grabbed && has_been_grabbed){
          // When object is NOT in the gripper but has been grabbed,
          // AKA, object has been placed down.
          init_rot_mat_set = false; //Reset variable
          drake::log()->info("object init set false");
        }
        else if(object_grabbed){
          has_been_grabbed = true;
        }

        #if DEBUG_MODE
                  temp_rpy.SetFromRotationMatrix(current_link_pose.rotation());
        drake::log()->info("curr pos: \n{} \n{} \n{}\n{} \n{} \n{}",
           current_pos[0],current_pos[1],current_pos[2],temp_rpy.roll_angle(),temp_rpy.pitch_angle(),temp_rpy.yaw_angle());
        #endif

        // Only does this once
        if(object_target_ != prev_target){
          DRAKE_DEMAND(object_target_ >= 0);
          // This changes object order depending on manipulation order 
          correct_order = object_correct_order[object_target_];

          //Takes current ee position
          rot_mat_init = current_link_pose.rotation();
          prev_target = object_target_;
          //Takes current object position
          const Eigen::Quaternion<double> quat_temp(
            object_state_.joint_position_measured[correct_order*7+0],
            object_state_.joint_position_measured[correct_order*7+1],
            object_state_.joint_position_measured[correct_order*7+2],
            object_state_.joint_position_measured[correct_order*7+3]
          );
          rot_mat_obj_init.set(quat_temp.toRotationMatrix());
          init_rot_mat_set = true;

temp_rpy.SetFromRotationMatrix(rot_mat_init);
          drake::log()->info("rotmat init: {} {} {}",  temp_rpy.roll_angle(), temp_rpy.pitch_angle(),temp_rpy.yaw_angle());
          drake::log()->info("obj_tar: {}\nprev_tar: {}", object_target_, prev_target);
          drake::log()->info("quat init: {} {} {}",  
           quat_temp.w(),quat_temp.x(),quat_temp.y(),quat_temp.z() );

          has_been_grabbed = false;

          


          DRAKE_DEMAND(false); 
        }

        if(init_rot_mat_set){
          // This rotates object to correct orientation. 
          // Rotating objects using rotation matrices are better than rpy.
          rpy.SetFromRotationMatrix(
            current_link_pose.rotation()*rot_mat_init.inverse()*rot_mat_obj_init);
            drake::log()->info("object is moving");
        }
        if(object_grabbed){
          quat = rpy.ToQuaternion();
          object_state_.joint_position_measured[correct_order*7+0] = quat.w();
          object_state_.joint_position_measured[correct_order*7+1] = quat.x();
          object_state_.joint_position_measured[correct_order*7+2] = quat.y();
          object_state_.joint_position_measured[correct_order*7+3] = quat.z();
          object_state_.joint_position_measured[correct_order*7+4] = current_pos[0];
          object_state_.joint_position_measured[correct_order*7+5] = current_pos[1];
          object_state_.joint_position_measured[correct_order*7+6] = current_pos[2];
        }
        
        #if DEBUG_MODE
          drake::log()->info("current grasp angle: \n{} \n{} \n{}",
            rpy.roll_angle(),rpy.pitch_angle(),rpy.yaw_angle());
          drake::log()->info("changed grasp angle: \n{} \n{} \n{}", 
            object_grasp_roll_angle_,object_grasp_pitch_angle_,object_grasp_yaw_angle_ );
          drake::log()->info("correct, target: {} {}", correct_order, object_target_);
          for(int i = 0; i < num_of_objects; i++){
            drake::log()->info("obj state for {}\n{}\n{}\n{}\n{}\n{}\n{}\n{}\n", 
              i,
              object_state_.joint_position_measured[i*7+0],
              object_state_.joint_position_measured[i*7+1],
              object_state_.joint_position_measured[i*7+2],
              object_state_.joint_position_measured[i*7+3],
              object_state_.joint_position_measured[i*7+4],
              object_state_.joint_position_measured[i*7+5],
              object_state_.joint_position_measured[i*7+6]);
          }
        #endif
        // Only publish object state when moved (maybe?)
        lcm_.publish(kLcmMultiObjectChannel, &object_state_);
      }

      if(send_to_kuka_2_){
        lcm_.publish(kLcmIiwa2Channel, &iiwa_status);
        lcm_.publish(kLcmSchunk2Channel, &wsg_status);
      }
      else if(send_to_kuka_1_){
        lcm_.publish(kLcmIiwaChannel, &iiwa_status);
        lcm_.publish(kLcmSchunkChannel, &wsg_status);
      }
    }
  }


 private:

  bool IsNear(double value1, double value2){
    if(value1 + 0.015 > value2 && value1 - 0.015 < value2 )
    {
      return true;
    }
    else return false;
  }

  void HandleTime(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_robot_time* status) {
    iiwa_time_ = *status;
  }
  void HandleTime2(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_robot_time* status) {
    iiwa_2_time_ = *status;
  }

  void HandleBox(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_cassie_status* status){

                        // drake::log()->info("box: {}\npos: {}",
                        // athena_cassie_downstairs_box_x_, status->joint_position_measured[11]
                        // );

    if(IsNear(athena_cassie_upstairs_box_x_, status->joint_position_measured[11]) &&
       IsNear(athena_cassie_upstairs_box_y_, status->joint_position_measured[12]) &&
       IsNear(athena_cassie_upstairs_box_z_, status->joint_position_measured[13])){
        send_to_kuka_2_ = true;
        send_to_kuka_1_ = false;
    }
    else if(
      IsNear(athena_cassie_downstairs_box_x_, status->joint_position_measured[11]) &&
      IsNear(athena_cassie_downstairs_box_y_, status->joint_position_measured[12]) &&
      IsNear(athena_cassie_downstairs_box_z_, status->joint_position_measured[13])){
        send_to_kuka_2_ = false;
        send_to_kuka_1_ = true;
    }
    else{
      send_to_kuka_2_ = false;
      send_to_kuka_1_ = false;
    }

    send_to_kuka_2_ = false; //PLEASE ERASE THIS!!! THIS IS A HACK
    send_to_kuka_1_ = true;

  }

  void HandleTarget(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_object_target* status) {
    object_target_ = status->target;
    object_grasp_height_ = status->grasp_height;
    object_grasp_roll_angle_ = status->roll_angle; 
    object_grasp_pitch_angle_ = status->pitch_angle;
    object_grasp_yaw_angle_ = status->yaw_angle;
    #if DEBUG_MODE
      if(counter_ % 10 == 0){
        drake::log()->info("obj_target num in handler:{}",status->target);
        drake::log()->info("\nobject roll: {}\nobject pitch: {}\nobject yaw: {}",
           object_grasp_roll_angle_,object_grasp_pitch_angle_,object_grasp_yaw_angle_);
      }
    #endif
    lcm_.publish(kLcmMultiObjectChannel, &object_state_);
    counter_++;
  }

  void HandlePlan(const ::lcm::ReceiveBuffer*, const std::string&,
                  const robotlocomotion::robot_plan_t* plan) {
    std::cout << "New plan received." << std::endl;
    if (send_to_kuka_1_ && iiwa_time_.utime == -1) {
      std::cout << "Discarding plan1, no status message received yet"
                << std::endl;
      return;
    } else if (send_to_kuka_2_ && iiwa_2_time_.utime == -1) {
      std::cout << "Discarding plan2, no status message received yet"
                << std::endl;
      return;
    } else if (plan->num_states < 2) {
      std::cout << "Discarding plan, Not enough knot points." << std::endl;
      return;
    }
    std::vector<Eigen::MatrixXd> knots(plan->num_states,
                                       Eigen::MatrixXd::Zero(kNumJoints+2, 1));
    for (int i = 0; i < plan->num_states; ++i) {
      const auto& state = plan->plan[i];
      for (int j = 0; j < state.num_joints; ++j) {
        int idx;
        if(send_to_kuka_1_ && !send_to_kuka_2_){
          if (!plant_.HasJointNamed(state.joint_name[j])) {
            continue;
          }
          const multibody::Joint<double>& joint =
              plant_.GetJointByName(state.joint_name[j]);
          DRAKE_DEMAND(joint.num_positions() == 1);
          idx = joint.position_start();
        }
        else if(!send_to_kuka_1_ && send_to_kuka_2_){
          if (!plant_2_.HasJointNamed(state.joint_name[j])) {
            continue;
          }
          const multibody::Joint<double>& joint =
              plant_2_.GetJointByName(state.joint_name[j]);
          DRAKE_DEMAND(joint.num_positions() == 1);
          idx = joint.position_start();
        }
        DRAKE_DEMAND(idx < kNumJoints+2);   

        // Treat the matrix at knots[i] as a column vector.
        if (i == 0) {
          // Always start moving from the position which we're
          // currently commanding.
          DRAKE_DEMAND(iiwa_time_.utime != -1);
          knots[i](idx, 0) = state.joint_position[j];
          // knots[0](idx, 0) = iiwa_time_.joint_position_commanded[j];
          // Have to delete this line without real iiwa_status available

        } else {
          knots[i](idx, 0) = state.joint_position[j];
        }
      }
    }

    for (int i = 0; i < plan->num_states; ++i) {
      std::cout << knots[i] << std::endl;
    }

    std::vector<double> input_time;
    for (int k = 0; k < static_cast<int>(plan->plan.size()); ++k) {
      input_time.push_back(plan->plan[k].utime / 1e6);
    }

    const Eigen::MatrixXd knot_dot = Eigen::MatrixXd::Zero(kNumJoints+2, 1);
    plan_.reset(new PiecewisePolynomial<double>(
        PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
            input_time, knots, knot_dot, knot_dot)));
    ++plan_number_;
  }

  

  void HandleStop(const ::lcm::ReceiveBuffer*, const std::string&,
                  const robotlocomotion::robot_plan_t*) {
    std::cout << "Received stop command. Discarding plan." << std::endl;
    plan_.reset();
  }

  ::lcm::LCM lcm_;
  const multibody::MultibodyPlant<double>& plant_;
  const multibody::MultibodyPlant<double>& plant_2_;
  int plan_number_{};
  int plan_number_2_{};
  std::unique_ptr<PiecewisePolynomial<double>> plan_;
  std::unique_ptr<PiecewisePolynomial<double>> plan_2_;
  lcmt_robot_time iiwa_time_;
  lcmt_robot_time iiwa_2_time_;
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::Context<double>> context_2_;
  int object_target_;
  std::vector<std::string> object_order_;
  double object_grasp_height_;
  double object_grasp_roll_angle_;
  double object_grasp_pitch_angle_;
  double object_grasp_yaw_angle_;
  int counter_;
  lcmt_object_status object_state_;
  double athena_cassie_upstairs_box_x_;
  double athena_cassie_upstairs_box_y_;
  double athena_cassie_upstairs_box_z_;
  double athena_cassie_downstairs_box_x_;
  double athena_cassie_downstairs_box_y_;
  double athena_cassie_downstairs_box_z_;
  bool send_to_kuka_2_;
  bool send_to_kuka_1_;
};

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

int do_main() {

  Json::Value root = 
    json_parser("drake/examples/collaboration_station/config.json");

  Json::Value object_list_ = 
    json_parser("drake/examples/collaboration_station/object_list.json");

  multibody::MultibodyPlant<double> plant(1e-5);

  // Load Kuka
  multibody::ModelInstanceIndex iiwa_instance = 
    multibody::Parser(&plant).AddModelFromFile(
      FindResourceOrThrow(root["robot_iiwa_path"].asString()));

  // Load WSG                                              
  multibody::ModelInstanceIndex wsg_instance = 
    multibody::Parser(&plant).AddModelFromFile(
      FindResourceOrThrow(root["robot_wsg_path"].asString()));
  
  // Weld iiwa
  const math::RigidTransformd X_WI(
      math::RollPitchYaw<double>(
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

  //For wsg
  const multibody::Frame<double>& link7 =
      plant.GetFrameByName("iiwa_link_7", iiwa_instance);

  // Weld WSG
  const math::RigidTransformd X_7G(
      math::RollPitchYaw<double>(M_PI_2, 0, M_PI_2), Eigen::Vector3d(0, 0, 0.1));
  plant.WeldFrames(link7, plant.GetFrameByName(
      "body", wsg_instance), X_7G);

  plant.Finalize();

  multibody::MultibodyPlant<double> plant2(1e-5);

  // Load Kuka
  multibody::ModelInstanceIndex iiwa_instance2 = 
    multibody::Parser(&plant2).AddModelFromFile(
      FindResourceOrThrow(root["robot_iiwa_path"].asString()));

  // Load WSG                                              
  multibody::ModelInstanceIndex wsg_instance2 = 
    multibody::Parser(&plant2).AddModelFromFile(
      FindResourceOrThrow(root["robot_wsg_path"].asString()));
  
  // Weld iiwa
  const math::RigidTransformd X_WI2(
      math::RollPitchYaw<double>(
        root["iiwa_pose_2"][0].asDouble(),
        root["iiwa_pose_2"][1].asDouble(),
        root["iiwa_pose_2"][2].asDouble()),
      Vector3d(
        root["iiwa_pose_2"][3].asDouble(),
        root["iiwa_pose_2"][4].asDouble(),
        root["iiwa_pose_2"][5].asDouble())
  );
  plant2.WeldFrames(plant2.world_frame(), plant2.GetFrameByName(
      "iiwa_link_0", iiwa_instance2), X_WI2);

  //For wsg
  const multibody::Frame<double>& link7_2 =
      plant2.GetFrameByName("iiwa_link_7", iiwa_instance2);

  // Weld WSG
  const math::RigidTransformd X_7G2(
      math::RollPitchYaw<double>(M_PI_2, 0, M_PI_2), Eigen::Vector3d(0, 0, 0.1));
  plant2.WeldFrames(link7_2, plant2.GetFrameByName(
      "body", wsg_instance2), X_7G2);

  plant2.Finalize();

  RobotPlanRunner runner(plant, plant2);
  runner.Run(root, object_list_);
  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake


int main() {
  return drake::examples::collaboration_station::do_main();
}
