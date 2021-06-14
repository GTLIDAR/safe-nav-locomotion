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

namespace drake {
namespace examples {
namespace collaboration_station{

class Object : public drake::systems::LeafSystem<double>
{

DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Object);

private:
  // Traj
  std::vector<Eigen::Matrix<double, 3, 1>> location_list;

  mutable int step = 0;

  // Simulation
  double h = 0.001;
  mutable double t = 0;
  const int num_joints_ = 7;

public:
  explicit Object();
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

}  // namespace collaboration_station
}  // namespace examples
}  // namespace drake
