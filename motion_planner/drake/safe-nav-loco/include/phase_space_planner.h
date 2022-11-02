#pragma once

#include "drake/safe-nav-loco/include/utils.h"
#include "drake/common/find_resource.h"

namespace phase_space_planner 
{

class PhaseSpacePlanner 
{

private:


public:
  // Step
  int step = 0;
  int stance = 2;
  double startflag;
  //double goodlateral = 1;
  // Safety Indicator
  double foot_dis = 0;

  double d_t;
  double Tstep=0;
  double sag;
  // Discrete Trajectory
  std::vector<Eigen::Matrix<double, 5, 1>> apex_list;
  std::vector<Eigen::Matrix<double, 5, 1>> d_list;

  std::vector<Eigen::Matrix<double, 5, 1>> waypoint_list;

  std::vector<Eigen::Matrix<double, 6, 1>> switch_list;
  std::vector<Eigen::Matrix<double, 6, 1>> apextraj_list;

  std::vector<Eigen::Matrix<double, 3, 1>> p_foot_list;

  std::vector<Eigen::Matrix<double, 3, 1>> direction_list;

  std::vector<Eigen::Matrix<double, 2, 1>> step_period;
  std::vector<Eigen::Matrix<double, 2, 1>> step_surface;
  
  // Continuous Trajectory
  std::vector<Eigen::Matrix<double, 10, 1>> COM_list;

  std::vector<Eigen::Matrix<double, 10, 1>> l_foot_list;
  std::vector<Eigen::Matrix<double, 10, 1>> r_foot_list;

  std::vector<Eigen::Matrix<double, 1, 1>> heading_list;

  // Moving Obstacle
  std::vector<Eigen::Matrix<double, 6, 1>> obstacle_list;

  double N,S,E,W;

    // wsq
  std::vector<Eigen::Matrix<double, 2, 1>> WSQlist;
  // timelist
  std::vector<Eigen::Matrix<double, 3, 1>> Tlist;
  // delta_y2, delta theta
  std::vector<Eigen::Matrix<double, 3, 1>> sim_list;
    // step length sagittal and lateral
  std::vector<Eigen::Matrix<double, 2, 1>> step_list;
 

   // PSP local length sagittal and lateral [x1 xf1 x1d x2 xf2 x2d ]
  std::vector<Eigen::Matrix<double, 13, 1>> psp_list;
 

   // Keyframe: X_apex = [x_apex, y_apex, z_apex, theta_apex, v_apex]
  Eigen::Matrix<double, 5, 1> X_apex;
  // Keyframe: X_d = [x_d, y_d, z_d, theta_d, v_d]
  Eigen::Matrix<double, 5, 1> X_d;
  // Switch: X_switch = [x_switch, y_switch, z_switch, dx_switch, dy_switch, dz_switch]

  Eigen::Matrix<double, 5, 1> waypoint;
  // Switch: X_switch = [x_switch, y_switch, z_switch, dx_switch, dy_switch, dz_switch]
  Eigen::Matrix<double, 6, 1> X_switch;
  
  // apextraj = [x_apex, y_apex, z_apex, dx_apex, dy_apex, dz_apex]
  Eigen::Matrix<double, 6, 1> apextraj;
  // Foot: p_foot = [x_foot, y_foot, z_foot]
  Eigen::Matrix<double, 3, 1> p_foot;

  // Primitive: prim = [step_length, dheading, dheight, v_apex, h_apex]
  Eigen::Matrix<double, 5, 1> prim;

  //
  Eigen::Matrix<double, 1, 1> goodlateral;

  //std::vector<double> w2_bkwrd ;
  //std::vector<double> w1_frwrd;
  std::vector<Eigen::Matrix<double, 1, 1>> w2_bkwrd;
  std::vector<Eigen::Matrix<double, 1, 1>> w1_frwrd;
  std::vector<Eigen::Matrix<double, 2, 1>> noise1_list;
  std::vector<Eigen::Matrix<double, 2, 1>> noise2_list;
  std::map<std::pair<int,int>, double> map_of_control_fhws;
  std::map<std::pair<int,int>, double> map_of_control_shws;
  
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
  /*
  void stand2apex(Eigen::Matrix<double, 3, 1> foot,
                  Eigen::Matrix<double, 3, 1> COM);
  */
  void LastStep();
  void End();
   

  void InitObstacle(Eigen::Matrix<double, 3, 1> obs_init,
                    Eigen::Matrix<double, 3, 1> obs2_init) ;
  void UpdateObstacle(Eigen::Matrix<double, 3, 1> obs,
                      Eigen::Matrix<double, 3, 1> obs2);

  void Stand(Eigen::Matrix<double, 3, 1> obs,
             Eigen::Matrix<double, 3, 1> obs2);

  void InitControlMap();

  void perturbed_traj();

  void UpdateKeyframe_pert();


};


}  // namespace phase_space_planner
