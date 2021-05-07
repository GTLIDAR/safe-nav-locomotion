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
namespace kuka_iiwa_arm{

class Iiwa : public drake::systems::LeafSystem<double>
{

DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Iiwa);

public:
  explicit Iiwa();

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

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
