
#include <gflags/gflags.h>
#include <fstream>
#include <jsoncpp/json/json.h>

#include "lcm/lcm-cpp.hpp"
#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/lcmt_robot_time.hpp"
#include "drake/lcmt_cassie_status.hpp"
#include "drake/lcmt_object_target.hpp"
#include "drake/lcmt_object_status.hpp"
#include "drake/lcmt_start_time.hpp"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/solve.h"

using namespace std;
using namespace Eigen;

namespace drake {
namespace examples {
namespace athena_cassie {
namespace {

const char* const kLcmStatusChannel = "ATHENA_CASSIE_STATUS";
const char* const kLcmTimeChannel = "ATHENA_CASSIE_TIME";
const char* const kLcmObjectTargetChannel = "OBJECT_TARGET_2";
const char* const kLcmMultiObjectStatusChannel = "MULTI_OBJECT_STATUS";
const char* const kLcmStartTimeChannel = "START_TIME_IIWA_1";


using drake::math::RigidTransform;
using Eigen::Vector3d;


class AthenaCassiePositionPublisher {
 public:

  explicit AthenaCassiePositionPublisher(const multibody::MultibodyPlant<double>& plant)
      : plant_(plant) {
    lcm_.subscribe(kLcmTimeChannel,
                    &AthenaCassiePositionPublisher::HandleRobotTime, this);
    lcm_.subscribe(kLcmObjectTargetChannel,
                    &AthenaCassiePositionPublisher::HandleTarget, this);
    lcm_.subscribe(kLcmMultiObjectStatusChannel,
                    &AthenaCassiePositionPublisher::HandleMultiObjectStatus, this);
  }

  void Run(Json::Value root, Json::Value collab_root, std::vector<Eigen::Matrix<double, 82, 1>> solution) 
  {
    context_ = plant_.CreateDefaultContext();
    const double num_joints = plant_.num_positions();
    drake::log()->info("\nnum_pos: {}\nnum_vels: {}\nnum_dof: {}\nnum_mbp_sts: {}\nnum_acts: {}"
                      "\nnum_joints: {}\nnum_bodies: {}",
                      num_joints, plant_.num_velocities(), plant_.num_actuated_dofs(),
                      plant_.num_multibody_states(), plant_.num_actuators(),plant_.num_joints(), plant_.num_bodies());

    lcmt_cassie_status athena_cassie_state;
    athena_cassie_state.num_joints = num_joints;
    athena_cassie_state.joint_position_measured.resize(num_joints, 0.);
    athena_cassie_state.joint_velocity_estimated.resize(num_joints, 0.);
    athena_cassie_state.joint_position_commanded.resize(num_joints, 0.);
    athena_cassie_state.joint_position_ipo.resize(num_joints, 0.);
    athena_cassie_state.joint_torque_measured.resize(num_joints, 0.);
    athena_cassie_state.joint_torque_commanded.resize(num_joints, 0.);
    athena_cassie_state.joint_torque_external.resize(num_joints, 0.);

    // Initialize Cassie's position in the simulation
    step_ = root["cassie_start_step"].asInt();
    for (int joint = 0; joint < num_joints; joint++) {
      athena_cassie_state.joint_position_measured[joint] = solution[step_](joint,0);
    }

    object_num_set_ = false;
    start_moving_ = false;
    int pub_counter = 0; 

    while (true) {
      if(!start_moving_ && pub_counter % 200==0){
        lcm_.publish(kLcmStatusChannel, &athena_cassie_state);
        drake::log()->info("publishing");
      }
      pub_counter++;

      // Call lcm handle until at least one status message is
      // processed.
      while (0 == lcm_.handleTimeout(10)) { }          

      if(object_num_set_){
        drake::log()->info("object_num_set_ is set");
        break;
      }
    }

    object_status_.num_joints = num_obj_joints_;
    object_status_.joint_position_measured.resize(num_obj_joints_, 0.);   
    object_status_.joint_velocity_estimated.resize(num_obj_joints_, 0.);
    object_status_.joint_position_commanded.resize(num_obj_joints_, 0.);
    object_status_.joint_position_ipo.resize(num_obj_joints_, 0.);
    object_status_.joint_torque_measured.resize(num_obj_joints_, 0.);
    object_status_.joint_torque_commanded.resize(num_obj_joints_, 0.);
    object_status_.joint_torque_external.resize(num_obj_joints_, 0.);

    plan_finished_ = false;
    already_moving_ = false;
    double start_time = 0;
    bool has_moved = false;
    const int object_num = num_obj_joints_/7;
    const int moving_object_num = collab_root["num_of_objects_to_use_1"].asInt();

    math::RigidTransform<double> box_transform;
    std::vector<math::RigidTransform<double>> init_objects_transform(moving_object_num);
    math::RigidTransform<double> new_object_transform;
    Eigen::Quaternion<double> box_quat;
    math::RotationMatrixd box_init_rot_mat;
    math::RotationMatrixd current_box_rot_mat;
    math::RotationMatrixd obj_init_rot_mat;
    std::vector<math::RotationMatrixd> obj_init_rot_mat_vec(moving_object_num);
    std::vector<int> object_correct_order;

    Eigen::VectorXd current_box_position; // quat + trans
    current_box_position.resize(7);
    Eigen::Vector3d box_init_trans = Eigen::Vector3d::Zero(3); 
    Eigen::Quaternion<double> quat;

    std::vector<Eigen::Vector3d> obj_init_trans(moving_object_num);
    Eigen::Vector3d box_trans;
    Eigen::Vector3d new_obj_trans;


    // int counter = 0;
    drake::log()->info("entering while loop");

    while (true) {

      // Call lcm handle until at least one status message is
      // processed.
      while (0 == lcm_.handleTimeout(10) || athena_cassie_state.utime == -1
            || plan_finished_ || !start_moving_) { }

      // Happens only once
      if(!already_moving_){
        start_time = robot_time_.utime;
      }

      // Update status time to simulation time
      athena_cassie_state.utime = robot_time_.utime;
      step_ = (int)(robot_time_.utime - start_time)/1000; 

      if(step_ >= solution.size()){
        drake::log()->info("robto time:  {}",(int)(robot_time_.utime - start_time)/1000);
        drake::log()->info("location list has finished");
        plan_finished_ = true;
        break; //End simulation
      }

      for (int joint = 0; joint < num_joints; joint++) {
        athena_cassie_state.joint_position_measured[joint] = solution[step_](joint,0);
      }
      lcm_.publish(kLcmStatusChannel, &athena_cassie_state);

      // Update every loop
      // Grab box to object transformations
      for(int i = 0; i < 7; i++){
        current_box_position(i) = solution[step_](i+7,0); // quat + trans starting at 7
      }
      box_quat.w() = current_box_position(0);
      box_quat.x() = current_box_position(1);
      box_quat.y() = current_box_position(2);
      box_quat.z() = current_box_position(3);
      current_box_rot_mat.set(box_quat.toRotationMatrix());

      // Happens only once
      if(!already_moving_){
        // start_time = robot_time_.utime;
        already_moving_ = true;
        has_moved = true;

        // Move box positions into their variables
        for(int i = 0; i < 3; i++){
          box_init_trans(i) = current_box_position(i+4);
        }

        //Set object initial translation and rotation of each object
        for(int i = 0 ; i < moving_object_num; i++){
          const Eigen::Quaternion<double> quat_temp(
            object_state_matrix_(i,0),
            object_state_matrix_(i,1),
            object_state_matrix_(i,2),
            object_state_matrix_(i,3)
          );

          // Object init wrt box - Since box is rotated 90, we have to do something to the object translation
          obj_init_rot_mat.set(quat_temp.toRotationMatrix());
          (init_objects_transform[i]).set_rotation(current_box_rot_mat.inverse() * obj_init_rot_mat); 
          (obj_init_trans[i])(0) = object_state_matrix_(i,5) - box_init_trans(1);
          (obj_init_trans[i])(1) = -object_state_matrix_(i,4) + box_init_trans(0);
          (obj_init_trans[i])(2) = object_state_matrix_(i,6) - box_init_trans(2);
          (init_objects_transform[i]).set_translation(obj_init_trans[i]);
        }
      }


      if(has_moved){
        //Set up box rigid transform
        box_trans(0) = current_box_position(4);
        box_trans(1) = current_box_position(5);
        box_trans(2) = current_box_position(6);
        box_transform.set_rotation(current_box_rot_mat);
        box_transform.set_translation(box_trans);

        // For moving objects
        for(int i = 0; i < moving_object_num; i++){

          new_object_transform = box_transform * init_objects_transform[i];
          new_obj_trans = new_object_transform.translation();
          quat = (new_object_transform.rotation()).ToQuaternion();
          object_status_.joint_position_measured[i*7+0] = quat.w();
          object_status_.joint_position_measured[i*7+1] = quat.x();
          object_status_.joint_position_measured[i*7+2] = quat.y();
          object_status_.joint_position_measured[i*7+3] = quat.z();
          object_status_.joint_position_measured[i*7+4] = new_obj_trans(0); //current_box_position(4) //+ obj_init_trans_mat(i,0) - box_init_trans(0);
          object_status_.joint_position_measured[i*7+5] = new_obj_trans(1); //current_box_position(5) //+ obj_init_trans_mat(i,1) - box_init_trans(1);
          object_status_.joint_position_measured[i*7+6] = new_obj_trans(2); //current_box_position(6) //+ obj_init_trans_mat(i,2) - box_init_trans(2);
        }

        // For static objects
        for(int i = moving_object_num; i < object_num; i++){
          for(int j = 0; j < 7; j++){
            object_status_.joint_position_measured[i*7+j] = object_state_matrix_(i,j);
          }
        }
        lcm_.publish(kLcmMultiObjectStatusChannel, &object_status_);
      }
    }

    // Send starting command to Iiwa 1
    lcmt_start_time start_iiwa;
    start_iiwa.start = true;
    int start_counter = 0;
    while(start_counter < 5000){
      while (0 == lcm_.handleTimeout(10)) { }  

      lcm_.publish(kLcmStartTimeChannel, &start_iiwa);
      lcm_.publish(kLcmMultiObjectStatusChannel, &object_status_);

      start_counter++;
    }
    // End of simulation
  }

 private:
  void HandleRobotTime(const ::lcm::ReceiveBuffer*, const std::string&,
                      const lcmt_robot_time* robot_time) {
    robot_time_ = *robot_time;
  }

  void HandleTarget(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_object_target* status) {
    if(status->target == -99 && !start_moving_){
      start_moving_ = true;   //Once this is true, never becomes false
      drake::log()->info("cassie_athena will start moving");
    }
  }

  // This only has to be called once
  void HandleMultiObjectStatus(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_object_status* status) {
    if(!object_num_set_){
      object_num_set_ = true;
      num_obj_joints_ = status->num_joints;                      
      object_state_matrix_.resize(num_obj_joints_/7, 7);
      drake::log()->info("object handler called : {}", num_obj_joints_); // 49
    }
    for(unsigned int i = 0; i < num_obj_joints_/7; i++){  
      for(int j = 0; j < 7; j++){
        object_state_matrix_(i , j) = status->joint_position_measured[i*7+j];                    
      }
    }    
  }

  ::lcm::LCM lcm_;
  const multibody::MultibodyPlant<double>& plant_;
  std::unique_ptr<systems::Context<double>> context_;
  lcmt_robot_time robot_time_;
  bool plan_finished_;
  unsigned int step_;
  unsigned int num_obj_joints_;
  bool start_moving_;
  bool already_moving_;
  bool object_num_set_;
  lcmt_object_status object_status_;
  Eigen::MatrixXd object_state_matrix_;

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

  Matrix<double, 6, 1> LeftArmFK(const Matrix<double, 7, 1>& q_gue_l)
  {
    double theta1, theta2, theta3, theta4, theta5, theta6, theta7, roll, pitch, yaw;
    theta1 = q_gue_l[0];
    theta2 = q_gue_l[1];
    theta3 = q_gue_l[2];
    theta4 = q_gue_l[3];
    theta5 = q_gue_l[4];
    theta6 = q_gue_l[5];
    theta7 = q_gue_l[6];
    double shoulderLength = 0.156165;
    double bitriLength = 0.257165;
    double elbowLength = 0.25947;
    double wristLength = 0.084;
    double shldr_ang = M_PI / 4;
    Matrix<double, 6, 1> q;
    Matrix4d R0_1, R1_2, R2_3, R3_4, R4_5, R5_6, R6_7, R7_8, R8_9, R9_10, R10_11, R11_12, R12_13, HTM;

    // Rotating Inertial Frame towards the Shoulder Joint
    R0_1 << 1, 0, 0, 0,
         0, cos(shldr_ang), sin(shldr_ang), 0,
         0, -sin(shldr_ang), cos(shldr_ang), 0,
         0, 0, 0, 1;
    // Translating the Intertial Frame towards the Shoulder Joint
    R1_2 << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, -shoulderLength,
         0, 0, 0, 1;
    // Shoulder Theta
    R2_3 << cos(theta1), sin(theta1), 0, 0,
         -sin(theta1), cos(theta1), 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
    // Upper Arm Theta
    R4_5 << cos(theta2), 0, sin(theta2), 0,
           0, 1, 0, 0,
           -sin(theta2), 0, cos(theta2), 0,
           0, 0, 0, 1;
    // Upper Arm Phi
    R5_6 << 1, 0, 0, 0,
         0, cos(theta3), -sin(theta3), 0,
         0, sin(theta3), cos(theta3), 0,
         0, 0, 0, 1;
    // Translation To The Elbow Joint
    R6_7 << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, -bitriLength,
         0, 0, 0, 1;
    // Elbow Theta
    R7_8 << cos(theta4), 0, sin(theta4), 0,
           0, 1, 0, 0,
           -sin(theta4), 0, cos(theta4), 0,
           0, 0, 0, 1;
    // Forearm Theta
    R8_9 << cos(theta5), sin(theta5), 0, 0,
         -sin(theta5), cos(theta5), 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
    // Translation To The Wrist Joint
    R9_10 << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, -elbowLength,
          0, 0, 0, 1;
    // Wrist Theta
    R10_11 << 1, 0, 0, 0,
         0, cos(theta6), -sin(theta6), 0,
         0, sin(theta6), cos(theta6), 0,
         0, 0, 0, 1;
    // Wrist Phi
    R11_12 << cos(theta7), 0, sin(theta7), 0,
           0, 1, 0, 0,
           -sin(theta7), 0, cos(theta7), 0,
           0, 0, 0, 1;
    // Translation To The Wrist Length
    R12_13 << 1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 1, -wristLength,
           0, 0, 0, 1;

    HTM = R0_1 * R1_2 * R2_3 * R4_5 * R5_6 * R6_7 * R7_8 * R8_9 * R9_10 * R10_11 * R11_12 * R12_13;
    roll = atan2(HTM(2,1),HTM(2,2));
    yaw = atan2(HTM(1,0),HTM(0,0));
    pitch = atan2(-HTM(2,0), sqrt(HTM(2,2) * HTM(2,2) + HTM(2,1) * HTM(2,1)));
    q << HTM(0, 3), HTM(1, 3), HTM(2, 3), roll, pitch, yaw;
    return q;
  }

  Matrix<double, 6, 1> RightArmFK(const Matrix<double, 7, 1>& q_gue_r)
  {
    double theta1, theta2, theta3, theta4, theta5, theta6, theta7, roll, pitch, yaw;
    theta1 = q_gue_r[0];
    theta2 = q_gue_r[1];
    theta3 = q_gue_r[2];
    theta4 = q_gue_r[3];
    theta5 = q_gue_r[4];
    theta6 = q_gue_r[5];
    theta7 = q_gue_r[6];
    double shoulderLength = 0.156165;
    double bitriLength = 0.257165;
    double elbowLength = 0.25947;
    double wristLength = 0.084;
    double shldr_ang = M_PI / 4;
    Matrix<double, 6, 1> q;
    Matrix4d R0_1, R1_2, R2_3, R3_4, R4_5, R5_6, R6_7, R7_8, R8_9, R9_10, R10_11, R11_12, R12_13, HTM;

    // Rotating Inertial Frame towards the Shoulder Joint
    R0_1 << 1, 0, 0, 0,
         0, cos(shldr_ang), -sin(shldr_ang), 0,
         0, sin(shldr_ang), cos(shldr_ang), 0,
         0, 0, 0, 1;
    // Translating the Intertial Frame towards the Shoulder Joint
    R1_2 << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, -shoulderLength,
         0, 0, 0, 1;
    // Shoulder Theta
    R2_3 << cos(-theta1), -sin(-theta1), 0, 0,
         sin(-theta1), cos(-theta1), 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
    // Upper Arm Theta
    R4_5 << cos(-theta2), 0, sin(-theta2), 0,
         0, 1, 0, 0,
         -sin(-theta2), 0, cos(-theta2), 0,
         0, 0, 0, 1;
    // Upper Arm Phi
    R5_6 << 1, 0, 0, 0,
         0, cos(theta3), -sin(theta3), 0,
         0, sin(theta3), cos(theta3), 0,
         0, 0, 0, 1;
   // Translation To The Elbow Joint
    R6_7 << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, -bitriLength,
         0, 0, 0, 1;
    // Elbow Theta
    R7_8 << cos(-theta4), 0, sin(-theta4), 0,
         0, 1, 0, 0,
    	 -sin(-theta4), 0, cos(-theta4), 0,
	 0, 0, 0, 1;
    // Forearm Theta
    R8_9 << cos(-theta5), -sin(-theta5), 0, 0,
         sin(-theta5), cos(-theta5), 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
    // Translation To The Wrist Joint
    R9_10 << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, -elbowLength,
          0, 0, 0, 1;
    // Wrist Theta
    R10_11 << 1, 0, 0, 0,
         0, cos(theta6), -sin(theta6), 0,
         0, sin(theta6), cos(theta6), 0,
         0, 0, 0, 1;
    // Wrist Phi
    R11_12 << cos(theta7), 0, sin(theta7), 0,
         0, 1, 0, 0,
         -sin(theta7), 0, cos(theta7), 0,
         0, 0, 0, 1;
    // Translation To The Wrist Length
    R12_13 << 1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 1, -wristLength,
           0, 0, 0, 1;

    HTM = R0_1 * R1_2 * R2_3 * R4_5 * R5_6 * R6_7 * R7_8 * R8_9 * R9_10 * R10_11 * R11_12 * R12_13;
    roll = atan2(HTM(2,1),HTM(2,2));
    yaw = atan2(HTM(1,0),HTM(0,0));
    pitch = atan2(-HTM(2,0), sqrt(HTM(2,2) * HTM(2,2) + HTM(2,1) * HTM(2,1)));
    q << HTM(0, 3), HTM(1, 3), HTM(2, 3), roll, pitch, yaw;
    return q;
  }

  Matrix<double, 6, 7> LeftArmJacobianFunction(const Matrix<double, 7, 1>& q_in_l)
  {
    int i, j;
    double hh = 0.0001;
    Matrix<double, 6, 1> FK0, FK1;
    Matrix<double, 7, 1> q;
    q = q_in_l;
    FK0 = LeftArmFK(q);
    Matrix<double, 6, 7> Jq;
    for (j=0; j < 7; j++){
        q[j] += hh;
        FK1 = LeftArmFK(q);
        q[j] -= hh;
        for (i=0; i < 6; i++){
            Jq(i, j) = (FK1[i] - FK0[i]) / hh;
        }
    }
    return Jq;
  }

  Matrix<double, 6, 7> RightArmJacobianFunction(const Matrix<double, 7, 1>& q_in_r)
  {
    int i, j;
    double hh = 0.0001;
    Matrix<double, 6, 1> FK0, FK1;
    Matrix<double, 7, 1> q;
    q = q_in_r;
    FK0 = RightArmFK(q);
    Matrix<double, 6, 7> Jq;
    for (j=0; j < 7; j++){
        q[j] += hh;
        FK1 = RightArmFK(q);
        q[j] -= hh;
        for (i=0; i < 6; i++){
            Jq(i, j) = (FK1[i] - FK0[i]) / hh;
        }
    }
    return Jq;
  }

  Matrix<double, 7, 1> DoIKLeftArm(const Matrix<double, 7, 1>& q_guess_l,
                             const Matrix<double, 6, 1>& x_d_l)
  {
    int i;
    double thresh = 0.005;
    Matrix<double, 6, 1> X_guess, error, mask;
    Matrix<double, 7, 1> q, q_0 = q_guess_l, q_1 = q_guess_l;
    Matrix<double, 6, 7> J;

    X_guess = LeftArmFK(q_guess_l);
    error = x_d_l - X_guess;
    for (i=0; i<6; i++){
        mask[i] = fabs(error[i]) > thresh;
    }

    while (mask.sum() > 0) {
        J = LeftArmJacobianFunction(q_0);
        q_1 = q_0 + J.transpose()*(J*J.transpose()).inverse()*(x_d_l - X_guess);
        q_0 = q_1;

        X_guess = LeftArmFK(q_0);
        error = x_d_l - X_guess;
        for (i=0; i<6; i++){
            mask[i] = fabs(error[i]) > thresh;
        }
    }
    for (i=0; i<7; i++){
        q[i] = q_1[i] -  2 * M_PI * floor((q_1[i] + M_PI ) / 2 / M_PI);
    }

    return q;
  }

  Matrix<double, 7, 1> DoIKRightArm(const Matrix<double, 7, 1>& q_guess_r,
                             const Matrix<double, 6, 1>& x_d_r)
  {
    int i;
    double thresh = 0.005;
    Matrix<double, 6, 1> X_guess, error, mask;
    Matrix<double, 7, 1> q, q_0 = q_guess_r, q_1 = q_guess_r;
    Matrix<double, 6, 7> J;

    X_guess = RightArmFK(q_guess_r);
    error = x_d_r - X_guess;
    for (i=0; i<6; i++){
        mask[i] = fabs(error[i]) > thresh;
    }

    while (mask.sum() > 0) {
        J = RightArmJacobianFunction(q_0);
        q_1 = q_0 + J.transpose()*(J*J.transpose()).inverse()*(x_d_r - X_guess);
        q_0 = q_1;

        X_guess = RightArmFK(q_0);
        error = x_d_r - X_guess;
        for (i=0; i<6; i++){
            mask[i] = fabs(error[i]) > thresh;
        }
    }
    for (i=0; i<7; i++){
        q[i] = q_1[i] -  2 * M_PI * floor((q_1[i] + M_PI ) / 2 / M_PI);
    }

    return q;
  }

  Matrix<double, 27, 1> DoIKCassie(const Matrix<double, 9, 1>& COM,
                                                      const Matrix<double, 9, 1>& l_foot,
                                                      const Matrix<double, 9, 1>& r_foot,
                                                      const Matrix<double, 1, 1>& heading)
  {
    Matrix<double, 27, 1> res;

    // Floating base quaternions
    res(0, 0) = std::cos(heading(0, 0) / 2);
    res(1, 0) = 0;
    res(2, 0) = 0;
    res(3, 0) = std::sin(heading(0, 0) / 2);

    // Floating base XYZ
    res(4, 0) = COM(0, 0);
    res(5, 0) = COM(1, 0);
    res(6, 0) = COM(2, 0);

    // this IK uses the analytical method by adding enough reasonable constraints
    // 1) hip yaw = 0;
    // 2) toe parallel to ground
    // 3) thigh and tarsus remain 13 degrees from parallel

    // comments and suggestions are welcomed - Jialin
    // all parameters are from: https://github.com/agilityrobotics/agility-cassie-doc/wiki/Kinematic-Model
    // this link is provided by Hongchang 

    // left leg
    // hip-roll
    double x_roll_left = COM(0, 0) + 0.021*std::cos(heading(0, 0)) - 0.135*std::sin(heading(0, 0));
    double y_roll_left = COM(1, 0) + 0.021*std::sin(heading(0, 0)) + 0.135*std::cos(heading(0, 0));
    double z_roll_left = COM(2, 0); // absolute coordinates of hip-roll joint

    double x_yaw_left = COM(0, 0) + (0.021 - 0.07)*std::cos(heading(0, 0)) - 0.135*std::sin(heading(0, 0));
    double y_yaw_left = COM(1, 0) + (0.021 - 0.07)*std::sin(heading(0, 0)) + 0.135*std::cos(heading(0, 0));
    double z_yaw_left = COM(2, 0); // absolute coordinates of hip-yaw joint

    // normal vector of the plane (plane A) of the 3 points: hip-roll joint, hip yaw joint, left foot
    double x_cross_left = (y_yaw_left - l_foot(1, 0))*(z_roll_left - l_foot(2, 0)) -
                          (z_yaw_left - l_foot(2, 0))*(y_roll_left - l_foot(1, 0));
    double y_cross_left = (z_yaw_left - l_foot(2, 0))*(x_roll_left - l_foot(0, 0)) -
                          (x_yaw_left - l_foot(0, 0))*(z_roll_left - l_foot(2, 0));
    double z_cross_left = (x_yaw_left - l_foot(0, 0))*(y_roll_left - l_foot(1, 0)) -
                          (y_yaw_left - l_foot(1, 0))*(x_roll_left - l_foot(0, 0));

    // note that from the hip-pitch joint to the toe joint, links do 2-D rotation.
    // those links' motion remains in a family of parallel planes, denoted by plane B
    // only hip-roll joint can change plane B's angle of slope

    double alpha_left = std::atan2(z_cross_left, std::sqrt(x_cross_left*x_cross_left + y_cross_left*y_cross_left)); // M_PI / 2 - plane A's angle of slope
    double beta_left = std::asin(0.0045 / (COM(2, 0) - l_foot(2, 0)) * std::cos(alpha_left)); // angle between plane A and B 
    res(7, 0) = alpha_left + beta_left; // hip-roll joint angle, namely M_PI / 2 - plane B's angle of slope


    // hip-yaw set 0
    res(9, 0) = 0;


    // hip-pitch, knee motor, knee, ankle, ankle spring

    // normalized vector pointing from hip-yaw joint to hip-roll joint
    double tx = (x_roll_left - x_yaw_left) / 0.07, ty = (y_roll_left - y_yaw_left) / 0.07;
    // left foot 2-D cordinates when projected to plane B, w.r.t hip-pitch joint
    double p_left = (l_foot(0, 0) - x_yaw_left)*tx + (l_foot(1, 0) - y_yaw_left)*ty;
    double q_left = 0.0045 / std::tan(beta_left)-0.090;
    // toe joint 2-D cordinates when projected to plane B, w.r.t hip-pitch joint
    double p_toe_left = p_left/* - 0.02865*/, q_toe_left = q_left/* - 0.04704*/; // not sure about the offsets, so they are not included yet

    // after calculating p, q_toe_left, remaining work is to solve the triangles
    double a = 0.500, b = 0.120, c = 0.410; // a: distance between knee joint and ankle joint, b: length of thigh, c: length of tarsus
    double d = std::sqrt(b*b + c*c + 2*b*c*std::cos(13 * M_PI / 180)); // tarsus and thigh are always 13 degrees from parallel when assuming no deformation of leaf springs
    double l_left = std::sqrt(p_toe_left*p_toe_left + q_toe_left*q_toe_left);
    double gamma_left = std::acos((l_left*l_left + d*d - a*a) / 2 / l_left / d);
    double delta_left = std::acos((d*d + c*c - b*b) / 2 / d / c);
    res(11, 0) = M_PI / 2 - 13 * M_PI / 180 - (std::atan2(q_toe_left, p_toe_left) - gamma_left - delta_left); // hip-pitch joint angle
    double phi_left = M_PI / 2 - std::atan2(q_toe_left, p_toe_left) - std::acos((l_left*l_left + a*a - d*d) / 2 / l_left / a);
    res(15, 0) = phi_left - res(11, 0) - 5.11 * M_PI / 180; // knee motor joint angle
    res(12, 0) = 0.089 + res(11, 0) + res(15, 0) + 0.05; // left achilles rod
    res(17, 0) = 0; // knee joint angle, 0 cuz no deformation
    res(19, 0) = 13 * M_PI / 180 - res(15, 0); // ankle joint angle
    res(21, 0) = 0; // left_toe_crank
    res(22, 0) = 0.4 - 13 * M_PI / 180 - 0.1; // ankle spring joint angle

    // toe, always parallel to ground
    res(23,0) = - 0.88 - res(19, 0) - res(15, 0) - res(11, 0); // toe joint angle, 0.88 got by trial (when other joint angles are 0, angle the toe needs to rotate to be parallel to ground)



    // right leg (very similar to left leg)
    double x_roll_right = COM(0, 0) + 0.021*std::cos(heading(0, 0)) + 0.135*std::sin(heading(0, 0));
    double y_roll_right = COM(1, 0) + 0.021*std::sin(heading(0, 0)) - 0.135*std::cos(heading(0, 0));
    double z_roll_right = COM(2, 0);

    double x_yaw_right = COM(0, 0) + (0.021 - 0.07)*std::cos(heading(0, 0)) + 0.135*std::sin(heading(0, 0));
    double y_yaw_right = COM(1, 0) + (0.021 - 0.07)*std::sin(heading(0, 0)) - 0.135*std::cos(heading(0, 0));
    double z_yaw_right = COM(2, 0);

    double x_cross_right = (y_yaw_right - r_foot(1, 0))*(z_roll_right - r_foot(2, 0)) -
                           (z_yaw_right - r_foot(2, 0))*(y_roll_right - r_foot(1, 0));
    double y_cross_right = (z_yaw_right - r_foot(2, 0))*(x_roll_right - r_foot(0, 0)) -
                           (x_yaw_right - r_foot(0, 0))*(z_roll_right - r_foot(2, 0));
    double z_cross_right = (x_yaw_right - r_foot(0, 0))*(y_roll_right - r_foot(1, 0)) -
                           (y_yaw_right - r_foot(1, 0))*(x_roll_right - r_foot(0, 0));

    double alpha_right = std::atan2(z_cross_right, std::sqrt(x_cross_right*x_cross_right + y_cross_right*y_cross_right));
    double beta_right = std::asin(0.0045 / (COM(2, 0) - r_foot(2, 0)) * std::cos(alpha_right));
    res(8, 0) = alpha_right - beta_right;


    res(10, 0) = 0;


    double p_right = (r_foot(0, 0) - x_yaw_right)*tx + (r_foot(1, 0) - y_yaw_right)*ty;
    double q_right = 0.0045 / std::tan(beta_right) - 0.090;
    double p_toe_right = p_right /*- 0.02865*/, q_toe_right = q_right/* - 0.04704*/;

    double l_right = std::sqrt(p_toe_right*p_toe_right + q_toe_right*q_toe_right);
    double gamma_right = std::acos((l_right*l_right + d*d - a*a) / 2 / l_right / d);
    double delta_right = std::acos((d*d + c*c - b*b) / 2 / d / c);
    res(13, 0) = M_PI / 2-13 * M_PI / 180 - (std::atan2(q_toe_right, p_toe_right) - gamma_right - delta_right);
    double phi_right = M_PI / 2 - std::atan2(q_toe_right, p_toe_right) - std::acos((l_right*l_right + a*a - d*d) / 2 / l_right / a);
    res(16, 0) = phi_right - res(13, 0) - 5.11 * M_PI / 180;
    res(14, 0) = 0.089 + res(13, 0) + res(16, 0) + 0.05;
    res(18, 0) = 0;
    res(20, 0) = 13 * M_PI / 180 - res(16, 0);
    res(24, 0) = 0;
    res(25, 0) = 0.4 - 13 * M_PI / 180 - 0.1;


    res(26,0) = - 0.88 - res(20, 0) - res(16, 0) - res(13, 0);

    return res;
  }

int do_main() {
  // Load in config.json file first
  Json::Value root = json_parser("drake/examples/"
                                 "athena_cassie/config.json");

  Json::Value collab_root = json_parser("drake/examples/"
                                        "collaboration_station/config.json");

  // Load cassie path files
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

  const int list_size = heading_list.size();

  std::vector<Eigen::Matrix<double, 82, 1>> solution;
  solution.resize(list_size);
  Eigen::VectorXd ang_l = VectorXd::Constant(7, 0.0);
  Eigen::VectorXd ang_r = VectorXd::Constant(7, 0.0);
  drake::log()->info("before for loop");

  // std::vector<Eigen::VectorXd> solution;
  for(int step = 0; step < list_size; step++)
  {
    if(step == 0){
      ang_r << -0.4657, -0.725099, 0.378249, -0.737147, -1.23264, -0.184066, -0.190421;
      ang_l << 0.140704, 0.76357, -0.123621,  0.818773, 1.45755, 0.272982, -0.0609805;
    }

    Eigen::VectorXd box1(7);
    box1 << std::cos(heading_list[step](0, 0) / 2), 0, 0, 
            std::sin(heading_list[step](0, 0) / 2), box_list[step];

    Eigen::VectorXd p = DoIKCassie(COM_list[step], l_foot_list[step], 
                                   r_foot_list[step], heading_list[step]);
    Eigen::VectorXd r = DoIKLeftArm(ang_l, l_wrist_list[step]);
    Eigen::VectorXd s = DoIKRightArm(ang_r, r_wrist_list[step]);

    Eigen::VectorXd hand = Eigen::VectorXd::Constant(22, 0.0); //hands
    hand[5] = 1.3;
    hand[15] = -0.35;
    solution[step] << p[0], p[1], p[2], p[3], p[4], p[5], p[6] + 0.10, 
                      box1, p[7], p[8], p[9], p[10], 0,  p[11], p[12], 
                      p[13], p[14], 0, 0, 0, p[15], p[16], 0, p[17], 
                      p[18], 0, p[19], p[20], p[21], p[22], p[23], p[24], 
                      p[25], p[26], r[0], 0, 0, 0, s[0], 0, 0, 0, r[1], 
                      s[1], r[2], s[2], r[3], s[3], r[4], s[4], r[5], 
                      s[5], r[6], s[6], hand;  
  }
  drake::log()->info("after for loop");

  multibody::MultibodyPlant<double> plant(1e-5);
  multibody::Parser(&plant).AddModelFromFile(
    FindResourceOrThrow(root["robot_athena_cassie_path"].asString()));
  multibody::Parser(&plant).AddModelFromFile(
    FindResourceOrThrow(root["robot_box_path"].asString()));
  plant.Finalize();
  drake::log()->info("running");

  AthenaCassiePositionPublisher position_publisher(plant);
  position_publisher.Run(root,collab_root, solution);
  return 0;
}

} // namespace no-dependent-files
} // namespace athena_cassie
} // namespace examples
} // namespace drake

int main() {
  return drake::examples::athena_cassie::do_main();
}