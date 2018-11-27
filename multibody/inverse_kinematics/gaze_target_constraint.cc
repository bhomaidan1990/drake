#include "drake/multibody/inverse_kinematics/gaze_target_constraint.h"

#include <limits>

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

namespace drake {
namespace multibody {
namespace internal {
GazeTargetConstraint::GazeTargetConstraint(
    const multibody_plant::MultibodyPlant<double>& plant,
    const Frame<double>& frameA, const Eigen::Ref<const Eigen::Vector3d>& p_AS,
    const Eigen::Ref<const Eigen::Vector3d>& n_A, const Frame<double>& frameB,
    const Eigen::Ref<const Eigen::Vector3d>& p_BT, double cone_half_angle,
    systems::Context<double>* context)
    : solvers::Constraint(2, plant.num_positions(), Eigen::Vector2d::Zero(),
          Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity())),
      plant_{plant},
      frameA_{frameA},
      p_AS_{p_AS},
      n_A_{NormalizeVector(n_A)},
      frameB_{frameB},
      p_BT_{p_BT},
      cone_half_angle_{cone_half_angle},
      cos_cone_half_angle_{std::cos(cone_half_angle_)},
      context_{context} {
  // TODO(avalenzu): Switch to analytical Jacobian and drop nq == nv requirement
  // when MBT provides the API for computing analytical Jacobian.
  if (cone_half_angle < 0 || cone_half_angle > M_PI) {
    throw std::invalid_argument(
        "GazeTargetConstraint: cone_half_angle should be within [0, pi]");
  }
}

void GazeTargetConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                  Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(math::initializeAutoDiff(x), &y_t);
  *y = math::autoDiffToValueMatrix(y_t);
}

void GazeTargetConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                  AutoDiffVecXd* y) const {
  y->resize(2);
  UpdateContextConfiguration(context_, plant_, math::autoDiffToValueMatrix(x));
  // position of target point T measured and expressed in A frame.
  Vector3<double> p_AT;
  plant_.tree().CalcPointsPositions(*context_, frameB_, p_BT_, frameA_, &p_AT);
  Eigen::MatrixXd Jv_ABt(6, plant_.num_positions());
  plant_.tree().CalcRelativeFrameGeometricJacobian(
      *context_, frameB_, p_BT_, frameA_, frameA_, &Jv_ABt);
  const Vector3<AutoDiffXd> p_ST_A =
      math::initializeAutoDiffGivenGradientMatrix(
          p_AT, Jv_ABt.bottomRows<3>()) -
      p_AS_;
  (*y)(0) = p_ST_A.dot(n_A_);
  (*y)(1) = (*y)(0) * (*y)(0) -
            std::pow(cos_cone_half_angle_, 2) * p_ST_A.dot(p_ST_A);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
