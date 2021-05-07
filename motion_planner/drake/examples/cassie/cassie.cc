#include "drake/examples/cassie/cassie.h"


namespace drake {
namespace examples {
namespace cassie{

using drake::systems::BasicVector;

  Cassie::Cassie()
  {  
    DeclareDiscreteState(27); 

    // When you change this port to the actual number of actuators on the robot, 
    // this class will become useful. Otherwise currently, this is just a passthrough.
    // For more info, check the header file.
    // position_measured_input_port_ = 
    //     DeclareVectorInputPort("cassie_position_measured", BasicVector<double>(23))
    //     .get_index();

    position_measured_output_port_ = 
        DeclareVectorOutputPort("cassie_floating_base_state", BasicVector<double>(27),
        &Cassie::CopyDiscreteStateOut).get_index();

    DeclarePeriodicDiscreteUpdate(0.001);
  }

  void Cassie::InitSystem(
    std::vector<Eigen::Matrix<double, 9, 1>> COM_l,
    std::vector<Eigen::Matrix<double, 9, 1>> l_foot_l,
    std::vector<Eigen::Matrix<double, 9, 1>> r_foot_l,
    std::vector<Eigen::Matrix<double, 1, 1>> heading_l)
  {
      COM_list = COM_l;
      l_foot_list = l_foot_l;
      r_foot_list = r_foot_l;
      heading_list = heading_l;
  }

  Eigen::Matrix<double, 27, 1> Cassie::DoIK(const Eigen::Matrix<double, 9, 1>& COM,
                                                  const Eigen::Matrix<double, 9, 1>& l_foot,
                                                  const Eigen::Matrix<double, 9, 1>& r_foot,
                                                  const Eigen::Matrix<double, 1, 1>& heading) const
  {
  Eigen::Matrix<double, 27, 1> res;

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

  return res;
  }

  void Cassie::DoCalcDiscreteVariableUpdates(
      const drake::systems::Context<double>& context,
      const std::vector<const drake::systems::DiscreteUpdateEvent<double>*>& ,
      drake::systems::DiscreteValues<double>* updates) const
  {
   
    t = context.get_time();

    step = int(t / h);

    Eigen::VectorXd state(27);
   
    Eigen::VectorXd p = DoIK(COM_list[step], l_foot_list[step], r_foot_list[step], heading_list[step]);
    
    state << p; 
   
    updates->get_mutable_vector().SetFromVector(state);





    //  state <<   p[0], p[1], p[2], p[3], p[4], p[5] + 0.5, p[6], p[7], p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15], p[16], p[17], p[18], p[19], p[20], p[21], p[22], p[23], p[24], p[25], p[26];

    //const auto& input_state = get_position_measured_input_port().Eval(context);
    // drake::log()->info("port name: {}",get_position_measured_input_port().get_name());

    // TODO: Get rid of this hack and put a throw statement after ensuring
    //        that position publisher does not send bad quaternion values
    // if(input_state[0] == 0 && input_state[1] == 0 && 
    //     input_state[2] == 0 && input_state[3] == 0)
    // {
    //   // drake::log()->info("Cassie: Either plan finished or Bad quaternion value detected. Setting state to default.");
    //   Eigen::VectorXd state(27);

    //   // state <<  1, 1, 1, 1, 1, 1, 1, 1, 1,
    //   //           0, 0, 1, 0, 0, 1, 0, 0, 1,
    //   //           0, 0, 1, 0, 0, 0, 0, 0, 0;
    //   state <<  3, 3, 3, 3, 3, 3, 3, 3, 3,
    //             3, 3, 3, 3, 3, 3, 3, 3, 3,
    //             3, 3, 3, 3, 3, 3, 3, 3, 3;


    //   updates->get_mutable_vector().SetFromVector(state);
    // }
    // else
    // {
    //   // Eigen::Matrix<double, 9, 1> COM_list;
    //   // COM_list << input_state[0],input_state[1],input_state[2],input_state[3],input_state[4],input_state[5],input_state[6],input_state[7],input_state[8];

    //   // Eigen::Matrix<double, 9, 1> l_foot_list;
    //   // l_foot_list << input_state[9],input_state[10],input_state[11],input_state[12],input_state[13],input_state[14],input_state[15],input_state[16],input_state[17];
      
    //   // Eigen::Matrix<double, 9, 1> r_foot_list;
    //   // r_foot_list << input_state[18],input_state[19],input_state[20],input_state[21],input_state[22],input_state[23],input_state[24],input_state[25],input_state[26];
      
    //   // Eigen::Matrix<double, 1, 1> heading_list;
    //   // heading_list << input_state[27];

    //   updates->get_mutable_vector().SetFromVector(input_state);
    // }


  }

  void Cassie::CopyDiscreteStateOut(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const 
  {
    auto d_state = context.get_discrete_state().get_vector().CopyToVector();
    output->SetFromVector(d_state);
  }

}  // namespace cassie
}  // namespace examples
}  // namespace drake
