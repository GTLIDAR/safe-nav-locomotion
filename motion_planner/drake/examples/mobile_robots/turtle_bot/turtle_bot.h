#pragma once

#include <cmath>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

#include "drake/common/find_resource.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace examples {
namespace mobile_robots{
namespace turtle_bot{

class TurtleBot : public drake::systems::LeafSystem<double>
{

DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TurtleBot);

private:
  // Traj
  std::vector<Eigen::Matrix<double, 3, 1>> location_list;

  mutable int step = 0;

  // Simulation
  double h = 0.001;
  mutable double t = 0;
  const int num_joints_ = 7;

public:
  explicit TurtleBot();
  void InitSystem(std::vector<Eigen::Matrix<double, 3, 1>> location_l);

  const systems::InputPort<double>& get_position_measured_input_port() const {
    return get_input_port(position_measured_input_port_);
  }

  const systems::OutputPort<double>& get_position_measured_output_port() const {
    return get_output_port(position_measured_output_port_);
  }

private:
  systems::InputPortIndex position_measured_input_port_{};
  systems::OutputPortIndex position_measured_output_port_{};

  void DoCalcDiscreteVariableUpdates(
      const drake::systems::Context<double>& context,
      const std::vector<const drake::systems::DiscreteUpdateEvent<double>*>& ,
      drake::systems::DiscreteValues<double>* updates) const override;
  void CopyDiscreteStateOut(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;

};

}  // namespace turtle_bot
}  // namespace mobile_robots
}  // namespace examples
}  // namespace drake
