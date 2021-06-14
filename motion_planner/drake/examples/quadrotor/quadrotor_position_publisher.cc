
#include <gflags/gflags.h>
#include <fstream>
#include <jsoncpp/json/json.h>

#include "lcm/lcm-cpp.hpp"
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/lcmt_object_target.hpp"
#include "drake/lcmt_robot_time.hpp"
#include "drake/lcmt_quadrotor_status.hpp"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"

namespace drake {
namespace examples {
namespace quadrotor {
namespace {

const char* const kLcmStatusChannel = "QUADROTOR_TEAM_STATUS";
const char* const kLcmTimeChannel = "QUADROTOR_TEAM_TIME";
const char* const kLcmObjectTargetChannel = "OBJECT_TARGET_2";

using drake::math::RigidTransform;
using Eigen::Vector3d;
using trajectories::PiecewisePolynomial;
typedef PiecewisePolynomial<double> PPType;
typedef Polynomial<double> PPPoly;
typedef PPType::PolynomialMatrix PPMatrix;

class QuadrotorPositionPublisher {
 public:

  explicit QuadrotorPositionPublisher(const multibody::MultibodyPlant<double>& plant)
      : plant_(plant) {
    lcm_.subscribe(kLcmTimeChannel,
                    &QuadrotorPositionPublisher::HandleRobotTime, this);
    lcm_.subscribe(kLcmObjectTargetChannel,
                    &QuadrotorPositionPublisher::HandleTarget, this);
  }

  void Run(Json::Value root, 
          std::vector<Eigen::Matrix<double, 3, 1>> quadrotor_list,
          std::vector<Eigen::Matrix<double, 3, 1>> box_1_list,
          std::vector<Eigen::Matrix<double, 3, 1>> box_2_list,
          std::vector<Eigen::Matrix<double, 3, 1>> box_3_list,
          std::vector<Eigen::Matrix<double, 3, 1>> box_4_list,
          std::vector<Eigen::Matrix<double, 1, 1>> heading_quadrotor_list,
          std::vector<Eigen::Matrix<double, 1, 1>> heading_box_1_list,
          std::vector<Eigen::Matrix<double, 1, 1>> heading_box_2_list,
          std::vector<Eigen::Matrix<double, 1, 1>> heading_box_3_list,
          std::vector<Eigen::Matrix<double, 1, 1>> heading_box_4_list,
          const int num_joints) {

    lcmt_quadrotor_status quadrotor_state;
    quadrotor_state.num_joints = num_joints;
    quadrotor_state.joint_position_measured.resize(num_joints, 0.);   
    quadrotor_state.joint_velocity_estimated.resize(num_joints, 0.);
    quadrotor_state.joint_position_commanded.resize(num_joints, 0.);
    quadrotor_state.joint_position_ipo.resize(num_joints, 0.);
    quadrotor_state.joint_torque_measured.resize(num_joints, 0.);
    quadrotor_state.joint_torque_commanded.resize(num_joints, 0.);
    quadrotor_state.joint_torque_external.resize(num_joints, 0.);


    drake::log()->info("RUNNING: {}", quadrotor_list.size());
    plan_finished_ = false;
    bool already_moving = false;
    double start_time = 0;
    start_moving_ = false;

  while (true) {
      // Call lcm handle until at least one status message is 
      // processed.
      while (0 == lcm_.handleTimeout(10) || quadrotor_state.utime == -1 
              || plan_finished_ || !start_moving_) { }

      // Happens only once
      if(!already_moving){
        start_time = robot_time_.utime;
        already_moving = true;
        drake::log()->info("quadrotor team started moving");
      }

      // Update status time to simulation time
      quadrotor_state.utime = robot_time_.utime;
      step_ = (int)(robot_time_.utime - start_time)/1000; 
      // quadrotor_step = step_ - 1000;
      
      if(step_ >= quadrotor_list.size())
      {
        drake::log()->info("location list has finished");
        plan_finished_ = true;
        break;
      }

      for (int joint = 0; joint < 7; joint++) 
      {
        if(step_ > 51125)
        {
          if(joint == 0)
          {
            // Must set quaternion w to a non-zero value when rest are 0
            quadrotor_state.joint_position_measured[joint] = 
              std::cos(heading_quadrotor_list[step_](0,0) / 2);
            quadrotor_state.joint_position_measured[joint+7] = 
              std::cos(heading_box_1_list[51125](0,0) / 2);
            quadrotor_state.joint_position_measured[joint+14] = 
              std::cos(heading_box_2_list[51125](0,0) / 2);
            quadrotor_state.joint_position_measured[joint+21] = 
              std::cos(heading_box_3_list[51125](0,0) / 2);
            quadrotor_state.joint_position_measured[joint+28] = 
              std::cos(heading_box_4_list[step_](0,0) / 2);
          }
          else if(joint == 1 || joint == 2){
            quadrotor_state.joint_position_measured[joint] = 0;
            quadrotor_state.joint_position_measured[joint+7] = 0;
            quadrotor_state.joint_position_measured[joint+14] = 0;
            quadrotor_state.joint_position_measured[joint+21] = 0;
            quadrotor_state.joint_position_measured[joint+28] = 0;
          }
          else if(joint == 3)
          {
            quadrotor_state.joint_position_measured[joint] = 
              std::sin(heading_quadrotor_list[step_](0,0) / 2);
            quadrotor_state.joint_position_measured[joint+7] = 
              std::sin(heading_box_1_list[51125](0,0) / 2);
            quadrotor_state.joint_position_measured[joint+14] = 
              std::sin(heading_box_2_list[51125](0,0) / 2);
            quadrotor_state.joint_position_measured[joint+21] = 
              std::sin(heading_box_3_list[51125](0,0) / 2);
            quadrotor_state.joint_position_measured[joint+28] = 
              std::sin(heading_box_4_list[step_](0,0) / 2);
          }
          else
          {
            quadrotor_state.joint_position_measured[joint] = quadrotor_list[step_](joint-4, 0);
            quadrotor_state.joint_position_measured[joint+7] = box_1_list[51125](joint-4, 0);
            quadrotor_state.joint_position_measured[joint+14] = box_2_list[51125](joint-4, 0);
            quadrotor_state.joint_position_measured[joint+21] = box_3_list[51125](joint-4, 0);
            quadrotor_state.joint_position_measured[joint+28] = box_4_list[step_](joint-4, 0);
          }

        }
        else
        {
          if(joint == 0)
          {
            // Must set quaternion w to a non-zero value when rest are 0
            quadrotor_state.joint_position_measured[joint] = 
              std::cos(heading_quadrotor_list[step_](0,0) / 2);
            quadrotor_state.joint_position_measured[joint+7] = 
              std::cos(heading_box_1_list[step_](0,0) / 2);
            quadrotor_state.joint_position_measured[joint+14] = 
              std::cos(heading_box_2_list[step_](0,0) / 2);
            quadrotor_state.joint_position_measured[joint+21] = 
              std::cos(heading_box_3_list[step_](0,0) / 2);
            quadrotor_state.joint_position_measured[joint+28] = 
              std::cos(heading_box_4_list[step_](0,0) / 2);
          }
          else if(joint == 1 || joint == 2){
            quadrotor_state.joint_position_measured[joint] = 0;
            quadrotor_state.joint_position_measured[joint+7] = 0;
            quadrotor_state.joint_position_measured[joint+14] = 0;
            quadrotor_state.joint_position_measured[joint+21] = 0;
            quadrotor_state.joint_position_measured[joint+28] = 0;
          }
          else if(joint == 3)
          {
            quadrotor_state.joint_position_measured[joint] = 
              std::sin(heading_quadrotor_list[step_](0,0) / 2);
            quadrotor_state.joint_position_measured[joint+7] = 
              std::sin(heading_box_1_list[step_](0,0) / 2);
            quadrotor_state.joint_position_measured[joint+14] = 
              std::sin(heading_box_2_list[step_](0,0) / 2);
            quadrotor_state.joint_position_measured[joint+21] = 
              std::sin(heading_box_3_list[step_](0,0) / 2);
            quadrotor_state.joint_position_measured[joint+28] = 
              std::sin(heading_box_4_list[step_](0,0) / 2);
          }
          else
          {
            quadrotor_state.joint_position_measured[joint] = quadrotor_list[step_](joint-4, 0);
            quadrotor_state.joint_position_measured[joint+7] = box_1_list[step_](joint-4, 0);
            quadrotor_state.joint_position_measured[joint+14] = box_2_list[step_](joint-4, 0);
            quadrotor_state.joint_position_measured[joint+21] = box_3_list[step_](joint-4, 0);
            quadrotor_state.joint_position_measured[joint+28] = box_4_list[step_](joint-4, 0);
          }
        }
      }

      lcm_.publish(kLcmStatusChannel, &quadrotor_state);
    }
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

  ::lcm::LCM lcm_;
  const multibody::MultibodyPlant<double>& plant_;
  lcmt_robot_time robot_time_;
  bool plan_finished_;
  int step_;
  double num_joints;
  bool start_moving_;

};



Json::Value json_parser(){
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

int do_main() {
  // Load in config.json file first
  Json::Value root = json_parser();

  // Load path file
  std::string full_path = root["path_to_drake"].asString() + 
                          root["log_quadrotor_team"].asString();
  std::vector<Eigen::Matrix<double, 3, 1>> quadrotor_list;
  read_data<3>(quadrotor_list, full_path);

  full_path = root["path_to_drake"].asString() + 
              root["log_box_1"].asString();
  std::vector<Eigen::Matrix<double, 3, 1>> box_1_list;
  read_data<3>(box_1_list, full_path);

  full_path = root["path_to_drake"].asString() + 
              root["log_box_2"].asString();
  std::vector<Eigen::Matrix<double, 3, 1>> box_2_list;
  read_data<3>(box_2_list, full_path);

  full_path = root["path_to_drake"].asString() + 
              root["log_box_3"].asString();
  std::vector<Eigen::Matrix<double, 3, 1>> box_3_list;
  read_data<3>(box_3_list, full_path);

  full_path = root["path_to_drake"].asString() + 
              root["log_box_4"].asString();
  std::vector<Eigen::Matrix<double, 3, 1>> box_4_list;
  read_data<3>(box_4_list, full_path);

  full_path = root["path_to_drake"].asString() + 
              root["log_heading_quadrotor_team"].asString();
  std::vector<Eigen::Matrix<double, 1, 1>> heading_quadrotor_list;
  read_data<1>(heading_quadrotor_list, full_path);

  full_path = root["path_to_drake"].asString() + 
              root["log_heading_box_1"].asString();
  std::vector<Eigen::Matrix<double, 1, 1>> heading_box_1_list;
  read_data<1>(heading_box_1_list, full_path);

  full_path = root["path_to_drake"].asString() + 
              root["log_heading_box_2"].asString();
  std::vector<Eigen::Matrix<double, 1, 1>> heading_box_2_list;
  read_data<1>(heading_box_2_list, full_path);

  full_path = root["path_to_drake"].asString() + 
              root["log_heading_box_3"].asString();
  std::vector<Eigen::Matrix<double, 1, 1>> heading_box_3_list;
  read_data<1>(heading_box_3_list, full_path);

  full_path = root["path_to_drake"].asString() + 
              root["log_heading_box_4"].asString();
  std::vector<Eigen::Matrix<double, 1, 1>> heading_box_4_list;
  read_data<1>(heading_box_4_list, full_path);

  multibody::MultibodyPlant<double> plant(1e-5);
  multibody::Parser(&plant).AddModelFromFile(
    FindResourceOrThrow(root["robot_quadrotor_team_path"].asString()));
  multibody::Parser(&plant).AddModelFromFile(
    FindResourceOrThrow(root["quadrotor_box_1_path"].asString()));
  multibody::Parser(&plant).AddModelFromFile(
    FindResourceOrThrow(root["quadrotor_box_2_path"].asString()));
  multibody::Parser(&plant).AddModelFromFile(
    FindResourceOrThrow(root["quadrotor_box_3_path"].asString()));
  multibody::Parser(&plant).AddModelFromFile(
    FindResourceOrThrow(root["quadrotor_box_4_path"].asString()));
  plant.Finalize();

  QuadrotorPositionPublisher position_publisher(plant);
  position_publisher.Run(root,
                          quadrotor_list,
                          box_1_list,
                          box_2_list,
                          box_3_list,
                          box_4_list,
                          heading_quadrotor_list,
                          heading_box_1_list,
                          heading_box_2_list,
                          heading_box_3_list,
                          heading_box_4_list,
                          plant.num_positions());
  return 0;
}

} // namespace no-dependent-files
} // namespace quadrotor
} // namespace examples
} // namespace drake

int main() {
  return drake::examples::quadrotor::do_main();
}