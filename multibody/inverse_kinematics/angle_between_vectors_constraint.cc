#include "drake/multibody/inverse_kinematics/angle_between_vectors_constraint.h"

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

namespace drake {
namespace multibody {
namespace internal {
AngleBetweenVectorsConstraint::AngleBetweenVectorsConstraint(
    const multibody_plant::MultibodyPlant<double>& plant,
    const Frame<double>& frameA, const Eigen::Ref<const Eigen::Vector3d>& na_A,
    const Frame<double>& frameB, const Eigen::Ref<const Eigen::Vector3d>& nb_B,
    double angle_lower, double angle_upper, systems::Context<double>* context)
    : solvers::Constraint(1, plant.num_positions(),
          Vector1d(std::cos(angle_upper)), Vector1d(std::cos(angle_lower))),
      plant_(plant),
      frameA_(frameA),
      na_unit_A_(NormalizeVector(na_A)),
      frameB_(frameB),
      nb_unit_B_(NormalizeVector(nb_B)),
      context_(context) {
  // TODO(avalenzu): Switch to analytical Jacobian and drop nq == nv requirement
  // when MBT provides the API for computing analytical Jacobian.
  if (!(angle_lower >= 0 && angle_upper >= angle_lower &&
        angle_upper <= M_PI)) {
    throw std::invalid_argument(
        "AngleBetweenVectorsConstraint: should satisfy 0 <= angle_lower <= "
        "angle_upper <= pi");
  }
}

void AngleBetweenVectorsConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(math::initializeAutoDiff(x), &y_t);
  *y = math::autoDiffToValueMatrix(y_t);
}

void AngleBetweenVectorsConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  y->resize(1);
  UpdateContextConfiguration(context_, plant_, math::autoDiffToValueMatrix(x));
  const Matrix3<double> R_AB =
      plant_.tree().CalcRelativeTransform(*context_, frameA_, frameB_).linear();
  Eigen::MatrixXd Jv_AB(6, plant_.num_positions());
  plant_.tree().CalcRelativeFrameGeometricJacobian(*context_, frameB_,
      Eigen::Vector3d::Zero() /* p_BQ */, frameA_, frameA_, &Jv_AB);
  Eigen::Vector3d nb_unit_A = R_AB * nb_unit_B_;
  *y = math::initializeAutoDiffGivenGradientMatrix(
      na_unit_A_.transpose() * nb_unit_A,
      -na_unit_A_.transpose() * math::VectorToSkewSymmetric(nb_unit_A) * Jv_AB);
}
}  // namespace internal
}  // namespace multibody
}  // namespace drake
