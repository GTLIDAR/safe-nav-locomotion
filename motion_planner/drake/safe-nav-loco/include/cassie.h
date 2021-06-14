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
namespace cassie{

class Cassie : public drake::systems::LeafSystem<double>
{

DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Cassie);

private:
  // Traj
  std::vector<Eigen::Matrix<double, 9, 1>> COM_list;
  std::vector<Eigen::Matrix<double, 9, 1>> l_foot_list;
  std::vector<Eigen::Matrix<double, 9, 1>> r_foot_list;
  std::vector<Eigen::Matrix<double, 1, 1>> heading_list;
  std::vector<Eigen::Matrix<double, 6, 1>> location_list;

  mutable int step = 0;

  // Simulation
  double h = 0.001;
  mutable double t = 0;

public:
  explicit Cassie();
  void InitSystem(std::vector<Eigen::Matrix<double, 9, 1>> COM_l,
                  std::vector<Eigen::Matrix<double, 9, 1>> l_foot_l,
                  std::vector<Eigen::Matrix<double, 9, 1>> r_foot_l,
                  std::vector<Eigen::Matrix<double, 1, 1>> heading_l,
                  std::vector<Eigen::Matrix<double, 6, 1>> location_l);

private:
  Eigen::Matrix<double, 27, 1> DoIK(const Eigen::Matrix<double, 9, 1>& COM,
                                    const Eigen::Matrix<double, 9, 1>& l_foot,
                                    const Eigen::Matrix<double, 9, 1>& r_foot,
                                    const Eigen::Matrix<double, 1, 1>& heading) const;
  void DoCalcDiscreteVariableUpdates(
      const drake::systems::Context<double>& context,
      const std::vector<const drake::systems::DiscreteUpdateEvent<double>*>& ,
      drake::systems::DiscreteValues<double>* updates) const override;
  void CopyDiscreteStateOut(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;

};

}  // namespace cassie
}  // namespace examples
}  // namespace drake
