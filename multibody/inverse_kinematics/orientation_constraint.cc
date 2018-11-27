#include "drake/multibody/inverse_kinematics/orientation_constraint.h"

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

namespace drake {
namespace multibody {
namespace internal {
OrientationConstraint::OrientationConstraint(
    const multibody_plant::MultibodyPlant<double>& plant,
    const Frame<double>& frameAbar, const math::RotationMatrix<double>& R_AbarA,
    const Frame<double>& frameBbar, const math::RotationMatrix<double>& R_BbarB,
    double theta_bound, systems::Context<double>* context)
    : solvers::Constraint(1, plant.num_positions(),
          Vector1d(2 * std::cos(theta_bound) + 1), Vector1d(3)),
      plant_{plant},
      frameAbar_{frameAbar},
      frameBbar_{frameBbar},
      R_AbarA_{R_AbarA},
      R_BbarB_{R_BbarB},
      context_(context) {
  frameBbar.HasThisParentTreeOrThrow(&plant_.tree());
  frameAbar.HasThisParentTreeOrThrow(&plant_.tree());
  DRAKE_DEMAND(context);
  // TODO(avalenzu): Switch to analytical Jacobian and drop nq == nv requirement
  // when MBT provides the API for computing analytical Jacobian.
  if (theta_bound < 0) {
    throw std::invalid_argument(
        "OrientationConstraint: theta_bound should be non-negative.\n");
  }
}

void OrientationConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                   Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(math::initializeAutoDiff(x), &y_t);
  *y = math::autoDiffToValueMatrix(y_t);
}

void OrientationConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  y->resize(1);
  UpdateContextConfiguration(context_, plant_, math::autoDiffToValueMatrix(x));
  const Matrix3<double> R_AbarBbar =
      plant_.tree()
          .CalcRelativeTransform(*context_, frameAbar_, frameBbar_)
          .linear();
  const Matrix3<double> R_AB =
      R_AbarA_.inverse().matrix() * R_AbarBbar * R_BbarB_.matrix();
  Eigen::MatrixXd Jv_AbarBbar(6, plant_.num_positions());
  plant_.tree().CalcRelativeFrameGeometricJacobian(*context_, frameBbar_,
      Eigen::Vector3d::Zero() /* p_BQ */, frameAbar_, frameAbar_, &Jv_AbarBbar);
  Eigen::RowVectorXd constraint_jacobian =
      Eigen::RowVector3d(R_AB(1, 2) - R_AB(2, 1), R_AB(2, 0) - R_AB(0, 2),
          R_AB(0, 1) - R_AB(1, 0)) *
      R_AbarA_.inverse().matrix() * Jv_AbarBbar.topRows<3>();
  *y = math::initializeAutoDiffGivenGradientMatrix(
      Eigen::Matrix<double, 1, 1>(R_AB.trace()), constraint_jacobian);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
