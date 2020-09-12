#pragma once

#include "drake/CDC/include/utils.h"


namespace phase_space_planner 
{

class PhaseSpacePlanner 
{

private:
  // Keyframe: X_apex = [x_apex, y_apex, z_apex, theta_apex, v_apex]
  Eigen::Matrix<double, 5, 1> X_apex;
  // Keyframe: X_d = [x_d, y_d, z_d, theta_d, v_d]
  Eigen::Matrix<double, 5, 1> X_d;
  // Switch: X_switch = [x_switch, y_switch, z_switch, dx_switch, dy_switch, dz_switch]
  Eigen::Matrix<double, 6, 1> X_switch;

  // Foot: p_foot = [x_foot, y_foot, z_foot]
  Eigen::Matrix<double, 3, 1> p_foot;

  // Primitive: prim = [step_length, dheading, dheight, v_apex, h_apex]
  Eigen::Matrix<double, 5, 1> prim;

public:
  // Step
  int step = 0;
  int stance = 2;
  double startflag;
  // Safety Indicator
  double foot_dis = 0;
  
  // Discrete Trajectory
  std::vector<Eigen::Matrix<double, 5, 1>> apex_list;
  std::vector<Eigen::Matrix<double, 5, 1>> d_list;
  std::vector<Eigen::Matrix<double, 6, 1>> switch_list;

  std::vector<Eigen::Matrix<double, 3, 1>> p_foot_list;

  std::vector<Eigen::Matrix<double, 3, 1>> direction_list;

  std::vector<Eigen::Matrix<double, 2, 1>> step_period;
  std::vector<Eigen::Matrix<double, 2, 1>> step_surface;
  
  // Continuous Trajectory
  std::vector<Eigen::Matrix<double, 9, 1>> COM_list;

  std::vector<Eigen::Matrix<double, 9, 1>> l_foot_list;
  std::vector<Eigen::Matrix<double, 9, 1>> r_foot_list;

  std::vector<Eigen::Matrix<double, 1, 1>> heading_list;

  // Moving Obstacle
  std::vector<Eigen::Matrix<double, 3, 1>> obstacle_list;

    // wsq
  std::vector<Eigen::Matrix<double, 2, 1>> WSQlist;


private:
  Eigen::Matrix<double, 3, 1> ForwardProp(
      double p_f, double p, double p_dot, 
      double h, double aq, double eps, double count);

public:
  void Init(Eigen::Matrix<double, 5, 1>& apex_init,
            Eigen::Matrix<double, 5, 1>& d_init,
            Eigen::Matrix<double, 3, 1>& p_foot_init);

  void UpdatePrimitive(Primitive& primitive);
  void UpdateKeyframe();
  void UpdateTrajectory();

  void Start(int stance_start);
  void FirstStep();
  void LastStep();
  void End();

  void InitObstacle(Eigen::Matrix<double, 3, 1> obs_init) ;
  void UpdateObstacle(Eigen::Matrix<double, 3, 1> obs);

  void Stand(Eigen::Matrix<double, 3, 1> obs);


};

}  // namespace phase_space_planner
