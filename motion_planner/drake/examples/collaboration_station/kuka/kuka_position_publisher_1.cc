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
#include "drake/lcmt_object_status.hpp"
#include "drake/lcmt_object_target.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/lcmt_start_time.hpp"
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
const char* const kLcmTargetChannel = "OBJECT_TARGET_1";
const char* const kLcmSchunkChannel = "WSG_STATUS_1";
const char* const kLcmIiwaChannel = "IIWA_STATUS_1";
const char* const kLcmTimeChannel = "IIWA_TIME_1";
const char* const kLcmBoxChannel = "ATHENA_CASSIE_STATUS";
const char* const kLcmPlanChannel = "COMMITTED_ROBOT_PLAN_1";
const char* const kLcmStopChannel = "STOP_1";
const char* const kLcmStartTimeChannel = "START_TIME_IIWA_1";

const int kNumJoints = 7;

using trajectories::PiecewisePolynomial;
typedef PiecewisePolynomial<double> PPType;
typedef Polynomial<double> PPPoly;
typedef PPType::PolynomialMatrix PPMatrix;

#define DEBUG_MODE false
#define READ_MODE true
#define WRITE_MODE false

class RobotPlanRunner {
 public:
  /// plant is aliased
  explicit RobotPlanRunner(const multibody::MultibodyPlant<double>& plant)
      : plant_(plant), plan_number_(0) {
    lcm_.subscribe(kLcmTimeChannel,
                    &RobotPlanRunner::HandleTime, this);
    lcm_.subscribe(kLcmStartTimeChannel,
                    &RobotPlanRunner::HandleStart, this);
    #if WRITE_MODE
    lcm_.subscribe(kLcmPlanChannel,
                    &RobotPlanRunner::HandlePlan, this);
    lcm_.subscribe(kLcmTargetChannel,
                    &RobotPlanRunner::HandleTarget, this);
    lcm_.subscribe(kLcmMultiObjectChannel,
                    &RobotPlanRunner::HandleMultiObjectStatus, this);
    #endif
  }

  void Run(Json::Value root, Json::Value object_list) {
    #if READ_MODE && WRITE_MODE
        DRAKE_DEMAND(false); //SHouldn't be reading and writing at the same time
    #endif

    #if READ_MODE
    // Load path files
    std::string full_path;
    full_path = root["path_to_drake"].asString() + root["log_iiwa_1_read"].asString();
    std::vector<Eigen::Matrix<double, 7, 1>> iiwa_1_list;
    read_data<7>(iiwa_1_list, full_path);

    full_path = root["path_to_drake"].asString() + root["log_wsg_1_read"].asString();
    std::vector<Eigen::Matrix<double, 2, 1>> wsg_1_list;
    read_data<2>(wsg_1_list, full_path);

    full_path = root["path_to_drake"].asString() + root["log_multi_object_1_read"].asString();
    std::vector<Eigen::Matrix<double, 21, 1>> multi_object_1_list;
    read_data<21>(multi_object_1_list, full_path);

    full_path = root["path_to_drake"].asString() + root["log_step_1_read"].asString();
    std::vector<Eigen::Matrix<double, 1, 1>> step_1;
    read_data<1>(step_1, full_path);
    const int max_steps = step_1[0](0,0);

    #endif  

    int cur_plan_number = plan_number_;
    int64_t cur_time_us = -1;
    int64_t start_time_us = -1;

    context_ = plant_.CreateDefaultContext();

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

    const int num_of_objects = root["num_of_objects_1"].asInt();
    DRAKE_DEMAND(num_of_objects == 3);
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

    athena_cassie_downstairs_box_x_ = root["box_downstairs_placement"][3].asDouble();
    athena_cassie_downstairs_box_y_ = root["box_downstairs_placement"][4].asDouble();
    athena_cassie_downstairs_box_z_ = root["box_downstairs_placement"][5].asDouble();

    bool first_time = true;
    bool object_grabbed = false;
    bool init_rot_mat_set = false;
    bool has_been_grabbed = false;
    send_to_kuka_1_ = false;
    first_handle_called_ = true;
    int correct_order = -1;
    int prev_target = -1;
    int step = 0;
    object_target_ = -1;
    double start_time = 0;

    Eigen::Vector3d current_pos;
    Eigen::VectorXd q(kNumJoints+2);
    Eigen::Quaternion<double> quat;
    math::RigidTransform<double> current_link_pose;
    math::RollPitchYaw<double> rpy(Vector3<double>::Zero(3));
    std::vector<int> object_correct_order;
    math::RotationMatrixd rot_mat_init;
    math::RotationMatrixd rot_mat_obj_init;

    #if READ_MODE
      drake::log()->info("reading because we have to: {} {} {} {} {} {}", 
        cur_plan_number, start_time_us,object_grabbed, init_rot_mat_set,
       has_been_grabbed, correct_order, prev_target);
    #endif

    #if WRITE_MODE
      Eigen::MatrixXd output_matrix_iiwa;
      Eigen::MatrixXd output_matrix_wsg;
      Eigen::MatrixXd output_matrix_multi_object;

      output_matrix_iiwa.resize(100000,7);
      output_matrix_wsg.resize(100000,2);
      output_matrix_multi_object.resize(100000,num_obj_joints);
    #endif

    // Get object manipulation order
    for(int i = 0; i < num_of_objects; i++){
      std::string temp = object_list["list"][i].asString();
      object_order_.push_back(temp);
      for (int j = 0; j < num_of_objects; j++){  
        std::string tmp2 = root["multi_object_order_1"][j].asString();
        if (tmp2.compare(temp) == 0){   
          object_correct_order.push_back(j);
          break;                   
        }
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
      while (0 == lcm_.handleTimeout(10) || iiwa_time_.utime == -1 || !send_to_kuka_1_) { }
      if(first_time){
        // drake::log()->info("inside second while loop");

        // for(int i = 0; i < num_of_objects*7; i++){
        //     object_state_.joint_position_measured[i] = init_multi_object_states_[i];
        // }
        // drake::log()->info("inside second while loop2");

        #if WRITE_MODE
        for(int i = 0; i < num_of_objects; i++){
          for(int j = 0; j < 7; j++){
            // Set all the list values to the init values.
            for(int k = 0; k < 100000; k++){
              output_matrix_multi_object(k, i*7+j) = init_multi_object_states_[i*7+j];
            }
          }
        }
        #endif
        start_time = iiwa_time_.utime;
        first_time = false;
      }

      if(object_target_ == -99){
        break; // end simulation
      }

      cur_time_us = iiwa_time_.utime;

      #if WRITE_MODE
        if(step < int((iiwa_time_.utime - start_time)/1000)){
          step++;
          if(step >= 100000){  drake::log()->info("step too big"); DRAKE_DEMAND(false); }
        }
      #endif

       #if READ_MODE
        step = static_cast<int>(iiwa_time_.utime - start_time)/1000;
        if(step >= max_steps){
          drake::log()->info("max steps reached! Ending simulation");
          break; //End simulation
        }
        else{
          for (int joint = 0; joint < 7; joint++) {
            iiwa_status.joint_position_measured[joint] = iiwa_1_list[step](joint,0);
          }
          wsg_status.actual_position_mm = wsg_1_list[step](0,0) - wsg_1_list[step](1,0);
          for(int joint = 0; joint < num_obj_joints; joint++){
            object_state_.joint_position_measured[joint] = multi_object_1_list[step](joint,0);
          }
          lcm_.publish(kLcmMultiObjectChannel, &object_state_);
        }
      #else
      if (plan_) {
        if (plan_number_ != cur_plan_number) {
          // std::cout << "Starting new plan." << std::endl;
          start_time_us = cur_time_us;
          cur_plan_number = plan_number_;
        }

        const double cur_traj_time_s =
            static_cast<double>(cur_time_us - start_time_us) / 1e6;
        const auto desired_next = plan_->value(cur_traj_time_s);

        iiwa_status.utime = iiwa_time_.utime;
        for (int joint = 0; joint < kNumJoints; joint++) 
        {
          iiwa_status.joint_position_measured[joint] = desired_next(joint);
          iiwa_status.joint_position_commanded[joint] = desired_next(joint);
          q[joint] = desired_next(joint);
          #if WRITE_MODE
            output_matrix_iiwa(step, joint) = desired_next(joint);
          #endif
        }

        // Set wsg position in millimeters
        wsg_status.actual_position_mm = desired_next(7)-desired_next(8);
        DRAKE_DEMAND(wsg_status.actual_position_mm > 0);
        q[7] = desired_next(7);
        q[8] = desired_next(8);
        object_grabbed = (wsg_status.actual_position_mm < 90) ? true : false;
        #if WRITE_MODE
          output_matrix_wsg(step, 0) = desired_next(7);
          output_matrix_wsg(step, 1) = desired_next(8);
        #endif

        // When object is NOT in the gripper but has been grabbed,
        // AKA, object has been placed down.
        if(!object_grabbed && has_been_grabbed){
          init_rot_mat_set = false; //Reset variable
        }

        // This changes object order depending on manipulation order 
        correct_order = object_correct_order[object_target_];

        // Only does this once per grasp
        if(object_target_ != prev_target){
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
          has_been_grabbed = false; // The object has changed. Therefore, it has not been grabbed yet...
        }

        // When moving with object
        if(object_grabbed){
          has_been_grabbed = true;

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

          if(init_rot_mat_set){
            // This rotates object to correct orientation. 
            // Rotating objects using rotation matrices are better than rpy.
            rpy.SetFromRotationMatrix(
              current_link_pose.rotation()); //*rot_mat_init.inverse()*rot_mat_obj_init);
          
            // drake::log()->info("rpy: {}\n{}\n{}\n",rpy.roll_angle(), rpy.pitch_angle(), rpy.yaw_angle());
            // drake::log()->info("pos: {}\n{}\n{}\n",current_pos[0],current_pos[1],current_pos[2]);
            quat = rpy.ToQuaternion();
            object_state_.joint_position_measured[correct_order*7+0] = quat.w();
            object_state_.joint_position_measured[correct_order*7+1] = quat.x();
            object_state_.joint_position_measured[correct_order*7+2] = quat.y();
            object_state_.joint_position_measured[correct_order*7+3] = quat.z();
            object_state_.joint_position_measured[correct_order*7+4] = current_pos[0];
            object_state_.joint_position_measured[correct_order*7+5] = current_pos[1];
            object_state_.joint_position_measured[correct_order*7+6] = current_pos[2];
          }

          // Only publish object state when moved (maybe?)
          lcm_.publish(kLcmMultiObjectChannel, &object_state_);
        }

        #if WRITE_MODE
          for(int i = 0; i < num_of_objects; i++){
              output_matrix_multi_object(step, i*7+0) = object_state_.joint_position_measured[i*7+0];
              output_matrix_multi_object(step, i*7+1) = object_state_.joint_position_measured[i*7+1];
              output_matrix_multi_object(step, i*7+2) = object_state_.joint_position_measured[i*7+2];
              output_matrix_multi_object(step, i*7+3) = object_state_.joint_position_measured[i*7+3];
              output_matrix_multi_object(step, i*7+4) = object_state_.joint_position_measured[i*7+4];
              output_matrix_multi_object(step, i*7+5) = object_state_.joint_position_measured[i*7+5];
              output_matrix_multi_object(step, i*7+6) = object_state_.joint_position_measured[i*7+6];
          }
        #endif
      }
      #endif

      lcm_.publish(kLcmIiwaChannel, &iiwa_status);
      lcm_.publish(kLcmSchunkChannel, &wsg_status);
    }

    #if WRITE_MODE
      drake::log()->info("writing to log file : {}", step);
      std::ofstream output_iiwa_file;
      std::string output_file_name = root["path_to_drake"].asString() + root["log_iiwa_1"].asString();
      output_iiwa_file.open(output_file_name);
      for(int i = 0; i < step; i++){
        output_iiwa_file << 
          output_matrix_iiwa(i,0) << " " << 
          output_matrix_iiwa(i,1) << " " <<  
          output_matrix_iiwa(i,2) << " " <<  
          output_matrix_iiwa(i,3) << " " <<  
          output_matrix_iiwa(i,4) << " " <<  
          output_matrix_iiwa(i,5) << " " <<  
          output_matrix_iiwa(i,6) << "\n"; 
      } 
      output_iiwa_file.close();

      std::ofstream output_wsg_file;
      output_file_name = root["path_to_drake"].asString() + root["log_wsg_1"].asString();
      output_wsg_file.open(output_file_name);
      for(int i = 0; i < step; i++){
        output_wsg_file << 
          output_matrix_wsg(i,0) << " " << 
          output_matrix_wsg(i,1) << "\n"; 
      } 
      output_wsg_file.close();

      std::ofstream output_multi_object_file;
      output_file_name = root["path_to_drake"].asString() + root["log_multi_object_1"].asString();
      output_multi_object_file.open(output_file_name);
      std::string object_positions = "";
      for(int i = 0; i < step; i++){
        for(int j = 0; j < num_obj_joints; j++){
          object_positions += std::to_string(output_matrix_multi_object(i,j));
          // drake::log()->info("obj pos: {} {} ",object_positions, output_matrix_multi_object(i,j));
          if(j+1 % 7 != 0){
            object_positions += " ";
            drake::log()->info("j : {}\ntotal: {}\nstatement: {}", j,j+1 % 7, j+1 % 7 != 0 );
          }
          else{
            drake::log()->info("no, there is no space : {}", j);
          }
        }
        output_multi_object_file << object_positions << std::endl;
        object_positions = ""; // Reset
      } 
      output_multi_object_file.close();
      drake::log()->info("finished writing: \n{}\n {} ", std::to_string(output_matrix_multi_object(0,0)), output_matrix_multi_object(0,1));
      
      std::ofstream output_step_file;
      output_file_name = root["path_to_drake"].asString() + root["log_step_1"].asString();
      output_step_file.open(output_file_name);
      output_step_file << step;
      output_step_file.close();
    
    #endif
  }

 private:

  void HandleStart(const ::lcm::ReceiveBuffer*, const std::string&,
                  const lcmt_start_time* status) {
                    drake::log()->info("comes here to handleStart");
        drake::log()->info("comes here to handleStart {}",status->start);
        drake::log()->info("comes here to handleStart {} {}",counter_, send_to_kuka_1_);

    if(counter_ < 2000 && status->start){
      send_to_kuka_1_ = true;
      counter_++;
    }
                        drake::log()->info("comes here to handleStart finished");

  }

  void HandleTime(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_robot_time* status) {
    iiwa_time_ = *status;
  }

  void HandleTarget(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_object_target* status) {
      drake::log()->info("comes here to handleTarget");

    object_target_ = status->target;
    object_grasp_height_ = status->grasp_height;
    object_grasp_roll_angle_ = status->roll_angle; 
    object_grasp_pitch_angle_ = status->pitch_angle;
    object_grasp_yaw_angle_ = status->yaw_angle;
                    drake::log()->info("comes here to handleTarget");

    lcm_.publish(kLcmMultiObjectChannel, &object_state_);

  }


  // This functions listens BEFORE kuka1 starts moving. 
  void HandleMultiObjectStatus(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_object_status* status){
      drake::log()->info("comes here to HandleMultiObjectStatus");

    if(!send_to_kuka_1_){ 
      if(first_handle_called_){
        init_multi_object_states_ = VectorX<double>::Zero(status->num_joints);
        first_handle_called_ = false;
        for(int i = 0; i < status->num_joints; i++){
          drake::log()->info("object position: {}", status->joint_position_measured[i]);
        }
      }
      for(int i = 0; i < status->num_joints; i++){
        init_multi_object_states_[i] = status->joint_position_measured[i];
      }
    }
  }

  void HandlePlan(const ::lcm::ReceiveBuffer*, const std::string&,
                  const robotlocomotion::robot_plan_t* plan) {
    // std::cout << "New plan received." << std::endl;
    if (iiwa_time_.utime == -1) {
      std::cout << "Discarding plan, no status message received yet"
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
        if (!plant_.HasJointNamed(state.joint_name[j])) {
          continue;
        }
        const multibody::Joint<double>& joint =
            plant_.GetJointByName(state.joint_name[j]);
        DRAKE_DEMAND(joint.num_positions() == 1);
        const int idx = joint.position_start();
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

  ::lcm::LCM lcm_;
  const multibody::MultibodyPlant<double>& plant_;
  int plan_number_{};
  std::unique_ptr<PiecewisePolynomial<double>> plan_;
  std::unique_ptr<systems::Context<double>> context_;
  std::vector<std::string> object_order_;
  bool send_to_kuka_1_;
  bool first_handle_called_;
  double object_grasp_height_;
  double object_grasp_roll_angle_;
  double object_grasp_pitch_angle_;
  double object_grasp_yaw_angle_;
  double athena_cassie_downstairs_box_x_;
  double athena_cassie_downstairs_box_y_;
  double athena_cassie_downstairs_box_z_;
  int counter_ = 0;
  int object_target_;
  lcmt_object_status object_state_;
  lcmt_robot_time iiwa_time_;
  VectorX<double> init_multi_object_states_;


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

  RobotPlanRunner runner(plant);
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
