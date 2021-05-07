
#include <gflags/gflags.h>
#include <fstream>
#include <jsoncpp/json/json.h>

#include "lcm/lcm-cpp.hpp"
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/lcmt_robot_time.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"

namespace drake {
namespace examples {
namespace collaboration_station {
namespace {

const char* const kLcmStatusChannel = "IIWA_STATUS";
const char* const kLcmTimeChannel = "IIWA_TIME";

using drake::math::RigidTransform;
using Eigen::Vector3d;
using trajectories::PiecewisePolynomial;
typedef PiecewisePolynomial<double> PPType;
typedef Polynomial<double> PPPoly;
typedef PPType::PolynomialMatrix PPMatrix;

class IiwaPositionPublisher {
 public:

  explicit IiwaPositionPublisher(const multibody::MultibodyPlant<double>& plant)
      : plant_(plant) {
    lcm_.subscribe(kLcmTimeChannel,
                    &IiwaPositionPublisher::HandleRobotTime, this);
  }

  void Run(Json::Value root, std::vector<Eigen::Matrix<double, 3, 1>> location_list) {

    // Get plant info
    num_joints_ = plant_.num_positions();
    drake::log()->info("\nnum_pos: {}\nnum_vels: {}\nnum_dof: {}\nnum_mbp_sts: {}\nnum_acts: {}"
                      "\nnum_joints: {}\nnum_bodies: {}",
                      num_joints_, plant_.num_velocities(), plant_.num_actuated_dofs(),
                      plant_.num_multibody_states(), plant_.num_actuators(),plant_.num_joints(), plant_.num_bodies());

    lcmt_iiwa_status iiwa_state;
    iiwa_state.num_joints = num_joints_;
    iiwa_state.joint_position_measured.resize(num_joints_, 0.);   
    iiwa_state.joint_velocity_estimated.resize(num_joints_, 0.);
    iiwa_state.joint_position_commanded.resize(num_joints_, 0.);
    iiwa_state.joint_position_ipo.resize(num_joints_, 0.);
    iiwa_state.joint_torque_measured.resize(num_joints_, 0.);
    iiwa_state.joint_torque_commanded.resize(num_joints_, 0.);
    iiwa_state.joint_torque_external.resize(num_joints_, 0.);

    drake::log()->info("RUNNING: {}", location_list.size());
    plan_finished_ = false;


  while (true) {
      // Call lcm handle until at least one status message is 
      // processed.
      while (0 == lcm_.handleTimeout(10) || iiwa_state.utime == -1 
              || plan_finished_) { }

      // Update status time to simulation time
      iiwa_state.utime = robot_time_.utime;

      step_ = int(robot_time_.utime / 1000); 
      
      if(step_ >= location_list.size())
      {
        drake::log()->info("location list has finished");
        plan_finished_ = true;
        continue;
      }

      for (int joint = 0; joint < num_joints_; joint++) 
      {
        if(joint == 0)
        {
          // Must set quaternion w to a non-zero value when rest are 0
          iiwa_state.joint_position_measured[joint] = 1;
        }
        else if(joint < 4)
        {
          iiwa_state.joint_position_measured[joint] = 0;
        }
        else
        {
          iiwa_state.joint_position_measured[joint] = location_list[step_](joint-4, 0);
        }
      }

      // Temporary Hack: 
      iiwa_state.joint_position_measured[4] = location_list[step_](0, 0)+2;
      iiwa_state.joint_position_measured[5] = location_list[step_](1, 0)-6;

      lcm_.publish(kLcmStatusChannel, &iiwa_state);
    }
  }
  


 private:
  void HandleRobotTime(const ::lcm::ReceiveBuffer*, const std::string&,
                      const lcmt_robot_time* robot_time) {
    robot_time_ = *robot_time;
  }

  ::lcm::LCM lcm_;
  const multibody::MultibodyPlant<double>& plant_;
  lcmt_robot_time robot_time_;
  bool plan_finished_;
  int step_;
  double num_joints_;
  std::vector<double> current_position_;
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
                          root["location_list"].asString();
  std::vector<Eigen::Matrix<double, 3, 1>> location_list;
  read_data<3>(location_list, full_path);

  multibody::MultibodyPlant<double> plant(1e-5);
  multibody::ModelInstanceIndex iiwa_instance = 
    multibody::Parser(&plant).AddModelFromFile(
      FindResourceOrThrow(root["robot_iiwa_path"].asString()));

  // Weld iiwa
  const RigidTransform<double> X_WI(
      AngleAxis<double>(M_PI/2, Vector3<double>::UnitZ()), Vector3d(2, 0.5, 0.3));
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(
      "iiwa_link_0", iiwa_instance), X_WI);

  plant.Finalize();

  IiwaPositionPublisher position_publisher(plant);
  position_publisher.Run(root,location_list);
  return 0;
}

} // namespace no-dependent-files
} // namespace collaboration_station
} // namespace examples
} // namespace drake

int main() {
  return drake::examples::collaboration_station::do_main();
}