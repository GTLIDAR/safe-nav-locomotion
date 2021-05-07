
#include <gflags/gflags.h>
#include <fstream>
#include <jsoncpp/json/json.h>

#include "lcm/lcm-cpp.hpp"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/lcmt_robot_time.hpp"
#include "drake/lcmt_cassie_status.hpp"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"

namespace drake {
namespace examples {
namespace cassie {
namespace {

const char* const kLcmStatusChannel = "CASSIE_STATUS";
const char* const kLcmTimeChannel = "CASSIE_TIME";

using drake::math::RigidTransform;
using Eigen::Vector3d;


class CassiePositionPublisher {
 public:

  explicit CassiePositionPublisher(const multibody::MultibodyPlant<double>& plant)
      : plant_(plant) {
    lcm_.subscribe(kLcmTimeChannel,
                    &CassiePositionPublisher::HandleRobotTime, this);
  }

  void Run( Json::Value root, std::vector<Eigen::Matrix<double, 27, 1>> solution) 
  {

    const double num_joints = plant_.num_positions();
    drake::log()->info("\nnum_pos: {}\nnum_vels: {}\nnum_dof: {}\nnum_mbp_sts: {}\nnum_acts: {}"
                      "\nnum_joints: {}\nnum_bodies: {}",
                      num_joints, plant_.num_velocities(), plant_.num_actuated_dofs(),
                      plant_.num_multibody_states(), plant_.num_actuators(),plant_.num_joints(), plant_.num_bodies());

    lcmt_cassie_status cassie_state;
    cassie_state.num_joints = num_joints;
    cassie_state.joint_position_measured.resize(num_joints, 0.);
    cassie_state.joint_velocity_estimated.resize(num_joints, 0.);
    cassie_state.joint_position_commanded.resize(num_joints, 0.);
    cassie_state.joint_position_ipo.resize(num_joints, 0.);
    cassie_state.joint_torque_measured.resize(num_joints, 0.);
    cassie_state.joint_torque_commanded.resize(num_joints, 0.);
    cassie_state.joint_torque_external.resize(num_joints, 0.);

    drake::log()->info("running");
    plan_finished_ = false;

    while (true) {
      // Call lcm handle until at least one status message is
      // processed.
      while (0 == lcm_.handleTimeout(10) || cassie_state.utime == -1
            || plan_finished_) { }

      // Update status time to simulation time
      cassie_state.utime = robot_time_.utime;

      step_ = int(robot_time_.utime / 1000); 
      
      if(step_ >= solution.size())
      {
        drake::log()->info("location list has finished");
        plan_finished_ = true;
        continue;
      }

      for (int joint = 0; joint < num_joints; joint++) 
      {
        cassie_state.joint_position_measured[joint] = solution[step_](joint,0);
      }

      lcm_.publish(kLcmStatusChannel, &cassie_state);
      drake::log()->info("status sent: {} ",cassie_state.num_joints);
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
  unsigned int step_;
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

Eigen::Matrix<double, 27, 1> DoIK(const Eigen::Matrix<double, 9, 1>& COM,
                                            const Eigen::Matrix<double, 9, 1>& l_foot,
                                            const Eigen::Matrix<double, 9, 1>& r_foot,
                                            const Eigen::Matrix<double, 1, 1>& heading)
  {
    Eigen::Matrix<double, 27, 1> res;

    // Floating base quaternions
    res(0, 0) = std::cos(heading(0, 0) / 2);;
    res(1, 0) = 0;
    res(2, 0) = 0;
    res(3, 0) = std::sin(heading(0, 0) / 2);

    // Floating base XYZ
    res(4, 0) = COM(0, 0);
    res(5, 0) = COM(1, 0);
    res(6, 0) = COM(2, 0);

    // this IK uses the analytical method by adding enough reasonable constraints
    // comments and suggestions are welcomed - Jialin
    // all parameters are from: https://github.com/agilityrobotics/agility-cassie-doc/wiki/Kinematic-Model
    // this link is provided by Hongchang 

    // hip-roll: res(7, 0) & res(8, 0)
    // hip-yaw: res(9, 0) & res(10, 0)
    // hip-pitch: res(11, 0) & res(12, 0)
    // knee motor: res(13, 0) & res(14, 0)
    // knee: res(15, 0) & res(16, 0)
    // ankle: res(17, 0) & res(18, 0)
    // ankle spring: res(19, 0) & res(21, 0)
    // toe: res(20, 0) & res(22, 0)
    // <joint_name: left & right>

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

    double alpha_left = std::atan2(z_cross_left, std::sqrt(x_cross_left*x_cross_left + y_cross_left*y_cross_left)); // PI / 2 - plane A's angle of slope
    double beta_left = std::asin(0.0045 / (COM(2, 0) - l_foot(2, 0)) * std::cos(alpha_left)); // angle between plane A and B 
    res(7, 0) = alpha_left + beta_left; // hip-roll joint angle, namely PI / 2 - plane B's angle of slope


    // hip-yaw set 0
    res(9, 0) = 0;


    // hip-pitch, knee motor, knee, ankle, ankle spring

    // normalized vector pointing from hip-yaw joint to hip-roll joint
    double tx = (x_roll_left - x_yaw_left) / 0.07, ty = (y_roll_left - y_yaw_left) / 0.07;
    // left foot 2-D cordinates when projected to plane B, w.r.t hip-pitch joint
    double p_left = (l_foot(0, 0) - x_yaw_left)*tx + (l_foot(1, 0) - y_yaw_left)*ty;
    double q_left = 0.0045 / std::tan(beta_left)-0.090;
    // foot joint 2-D cordinates when projected to plane B, w.r.t hip-pitch joint
    double p_toe_left = p_left/* - 0.02865*/, q_toe_left = q_left/* - 0.04704*/; // not sure about the offsets

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
    res(22, 0) = 0.4 - 13 * M_PI / 180 - 0.1; // ankle spring joint angle, 0 cuz no deformation

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
    drake::log()->info("finished IK");

    return res;
  }

int do_main() {
  // Load in config.json file first
  Json::Value root = json_parser();

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

  full_path = root["path_to_drake"].asString() + root["heading_list"].asString();
  std::vector<Eigen::Matrix<double, 1, 1>> heading_list;
  read_data<1>(heading_list, full_path);

  int list_size = heading_list.size();

  std::vector<Eigen::Matrix<double, 27, 1>> solution;
  solution.resize(list_size);
  for(int i = 0; i < list_size; i++)
  {
    solution[i] << DoIK(COM_list[i], l_foot_list[i], r_foot_list[i], heading_list[i]);
  }
  drake::log()->info("plant now");

  multibody::MultibodyPlant<double> plant(1e-5);
  multibody::Parser(&plant).AddModelFromFile(
    FindResourceOrThrow(root["robot_cassie_path"].asString()));
  plant.Finalize();
      drake::log()->info("running");

  CassiePositionPublisher position_publisher(plant);
  position_publisher.Run(root, solution);
  return 0;
}

} // namespace no-dependent-files
} // namespace collaboration_station
} // namespace examples
} // namespace drake

int main() {
  return drake::examples::cassie::do_main();
}