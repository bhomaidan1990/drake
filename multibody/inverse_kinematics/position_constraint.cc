#include "drake/multibody/inverse_kinematics/position_constraint.h"

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

namespace drake {
namespace multibody {
namespace internal {
PositionConstraint::PositionConstraint(
    const multibody_plant::MultibodyPlant<double>& plant,
    const FrameIndex& frameB_idx, const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
    const FrameIndex& frameA_idx,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper,
    systems::Context<double>* context)
    : solvers::Constraint(3, plant.num_positions(), p_AQ_lower, p_AQ_upper),
      plant_(plant),
      frameB_{plant_.tree().get_frame(frameB_idx)},
      frameA_{plant_.tree().get_frame(frameA_idx)},
      p_BQ_{p_BQ},
      context_{context} {
  DRAKE_DEMAND(context);
}

void PositionConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(math::initializeAutoDiff(x), &y_t);
  *y = math::autoDiffToValueMatrix(y_t);
}

void PositionConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                AutoDiffVecXd* y) const {
  y->resize(3);
  UpdateContextConfiguration(context_, plant_, math::autoDiffToValueMatrix(x));
  Eigen::Vector3d p_AQ{};
  plant_.tree().CalcPointsPositions(*context_, frameB_, p_BQ_, frameA_, &p_AQ);
  Eigen::MatrixXd Jv_WBq(3, plant_.num_positions());
  Eigen::MatrixXd Jv_WA(3, plant_.num_positions());
  plant_.tree().CalcPointsGeometricJacobianExpressedInWorld(*context_, frameB_,
                                                            p_BQ_, &Jv_WBq);
  plant_.tree().CalcPointsGeometricJacobianExpressedInWorld(
      *context_, frameA_, Eigen::Vector3d::Zero(), &Jv_WA);
  Eigen::Matrix3d R_AW =
      plant_.tree()
          .CalcRelativeTransform(*context_, frameA_, plant_.world_frame())
          .linear();
  Eigen::Matrix3Xd Jv_ABq = R_AW * (Jv_WBq - Jv_WA);
  *y = math::initializeAutoDiffGivenGradientMatrix(p_AQ, Jv_ABq);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
