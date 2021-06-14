
#include <gflags/gflags.h>
#include <fstream>
#include <jsoncpp/json/json.h>

#include "lcm/lcm-cpp.hpp"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/lcmt_object_target.hpp"
#include "drake/lcmt_robot_time.hpp"
#include "drake/lcmt_turtle_status.hpp"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"

namespace drake {
namespace examples {
namespace mobile_robots {
namespace turtle_bot {
namespace {

const char* const kLcmStatusChannel = "TURTLE_STATUS";
const char* const kLcmTimeChannel = "TURTLE_TIME";
const char* const kLcmObjectTargetChannel = "OBJECT_TARGET_2";

using drake::math::RigidTransform;
using Eigen::Vector3d;

class TurtlePositionPublisher {
 public:

  explicit TurtlePositionPublisher(const multibody::MultibodyPlant<double>& plant)
      : plant_(plant) {
    lcm_.subscribe(kLcmTimeChannel,
                    &TurtlePositionPublisher::HandleRobotTime, this);
    lcm_.subscribe(kLcmObjectTargetChannel,
                    &TurtlePositionPublisher::HandleTarget, this);
  }

  void Run(Json::Value root, std::vector<Eigen::Matrix<double, 3, 1>> location_list, 
          const int num_joints) {

    lcmt_turtle_status turtle_state;
    turtle_state.num_joints = num_joints;
    turtle_state.joint_position_measured.resize(num_joints, 0.);   
    turtle_state.joint_velocity_estimated.resize(num_joints, 0.);
    turtle_state.joint_position_commanded.resize(num_joints, 0.);
    turtle_state.joint_position_ipo.resize(num_joints, 0.);
    turtle_state.joint_torque_measured.resize(num_joints, 0.);
    turtle_state.joint_torque_commanded.resize(num_joints, 0.);
    turtle_state.joint_torque_external.resize(num_joints, 0.);

    drake::log()->info("RUNNING: {}", location_list.size());
    plan_finished_ = false;
    bool already_moving = false;
    double start_time = 0;
    start_moving_ = false;

    while (true) {
      // Call lcm handle until at least one status message is 
      // processed.
      while (0 == lcm_.handleTimeout(10) || turtle_state.utime == -1 
              || plan_finished_ || !start_moving_) { }

      // Happens only once
      if(!already_moving){
        start_time = robot_time_.utime;
        already_moving = true;
        drake::log()->info("turtle started moving");
      }

      // Update status time to simulation time
      turtle_state.utime = robot_time_.utime;
      step_ = (int)(robot_time_.utime - start_time)/1000; 

      if(step_ >= location_list.size())
      {
        drake::log()->info("location list has finished");
        plan_finished_ = true;
        break;
      }

      for (int joint = 0; joint < num_joints; joint++) 
      {
        if(joint == 0)
        {
          // Must set quaternion w to a non-zero value when rest are 0
          turtle_state.joint_position_measured[joint] = 1;
        }
        else if(joint < 4)
        {
          turtle_state.joint_position_measured[joint] = 0;
        }
        else
        {
          turtle_state.joint_position_measured[joint] = location_list[step_](joint-4, 0);
        }
      }

      lcm_.publish(kLcmStatusChannel, &turtle_state);
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
  unsigned int step_;
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
                          root["log_obstacle"].asString();
  std::vector<Eigen::Matrix<double, 3, 1>> location_list;
  read_data<3>(location_list, full_path);

  multibody::MultibodyPlant<double> plant(1e-5);
  multibody::Parser(&plant).AddModelFromFile(
    FindResourceOrThrow(root["robot_turtle_path"].asString()));
  plant.Finalize();

  TurtlePositionPublisher position_publisher(plant);
  position_publisher.Run(root,location_list, plant.num_positions());
  return 0;
}

} // namespace no-dependent-files
} // namespace turtle_bot
} // namespace mobile_robots
} // namespace examples
} // namespace drake

int main() {
  return drake::examples::mobile_robots::turtle_bot::do_main();
}