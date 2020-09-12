#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

namespace drake {
namespace multibody {
namespace internal {
bool AreAutoDiffVecXdEqual(const Eigen::Ref<const VectorX<AutoDiffXd>>& a,
                           const Eigen::Ref<const VectorX<AutoDiffXd>>& b) {
  if (a.rows() != b.rows()) {
    return false;
  }
  if (math::autoDiffToValueMatrix(a) != math::autoDiffToValueMatrix(b)) {
    return false;
  }
  const Eigen::MatrixXd a_gradient = math::autoDiffToGradientMatrix(a);
  const Eigen::MatrixXd b_gradient = math::autoDiffToGradientMatrix(b);
  if (a_gradient.rows() != b_gradient.rows() ||
      a_gradient.cols() != b_gradient.cols()) {
    return false;
  }
  return a_gradient == b_gradient;
}

void UpdateContextConfiguration(drake::systems::Context<double>* context,
                                const MultibodyPlant<double>& plant,
                                const Eigen::Ref<const VectorX<double>>& q) {
  DRAKE_ASSERT(context);
  if (q != plant.GetPositions(*context)) {
    plant.SetPositions(context, q);
  }
}

void UpdateContextConfiguration(drake::systems::Context<double>* context,
                                const MultibodyPlant<double>& plant,
                                const Eigen::Ref<const AutoDiffVecXd>& q) {
  return UpdateContextConfiguration(context, plant,
                                    math::autoDiffToValueMatrix(q));
}

void UpdateContextConfiguration(systems::Context<AutoDiffXd>* context,
                                const MultibodyPlant<AutoDiffXd>& plant,
                                const Eigen::Ref<const AutoDiffVecXd>& q) {
  DRAKE_ASSERT(context);
  if (!AreAutoDiffVecXdEqual(q, plant.GetPositions(*context))) {
    plant.SetPositions(context, q);
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake