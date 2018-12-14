#pragma once

#include <memory>

#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace multibody {
/**
 * Constrains a target point T to be within a cone K. The point T ("T" stands
 * for "target") is fixed in a frame B, with position p_BT. The cone
 * originates from a point S ("S" stands for "source"), fixed in frame A with
 * position p_AS, with the axis of the cone being n, also fixed
 * in frame A. The half angle of the cone is θ. A common usage of this
 * constraint is that a camera should gaze at some target; namely the target
 * falls within a gaze cone, originating from the camera eye.
 *
 * Mathematically the constraint is
 *    p_ST_Aᵀ * n_unit_A   ≥ 0
 *   (p_ST_Aᵀ * n_unit_A)² ≥ (cosθ)²p_ST_Aᵀ * p_ST_A
 * where p_ST_A is the vector from S to T, expressed in frame A. n_unit_A is the
 * unit length directional vector representing the center ray of the cone.
 */
class GazeTargetConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GazeTargetConstraint)

  /**
   * @param plant The MultibodyPlant on which the constraint is imposed.
   *   @p plant should be alive during the lifetime of this constraint.
   * @param frameA The frame to which the gaze cone is fixed.
   * @param p_AS The position of the cone source point S, measured and expressed
   *   in frame A.
   * @param n_A The directional vector representing the center ray of the cone,
   *   expressed in frame A.
   * @param frameB The frame to which the target point T is fixed.
   * @param p_BT The position of the target point T, measured and expressed in
   *   frame B.
   * @param cone_half_angle The half angle of the cone. We denote it as θ in the
   *   class documentation. @p cone_half_angle is in radians.
   * @param context The Context that has been allocated for this @p plant. We
   *   will update the context when evaluating the constraint. @p context should
   *   be alive during the lifetime of this constraint.
   * @pre @p n_A cannot be a zero vector.
   * @pre @p 0 <= cone_half_angle <= pi.
   * @throws std::invalid_argument is n_A is close to a zero vector.
   * @throws std::invalid_argument if cone_half_angle is outside of the bound.
   */
  GazeTargetConstraint(
      const multibody_plant::MultibodyPlant<double>* const plant,
      const Frame<double>& frameA,
      const Eigen::Ref<const Eigen::Vector3d>& p_AS,
      const Eigen::Ref<const Eigen::Vector3d>& n_A, const Frame<double>& frameB,
      const Eigen::Ref<const Eigen::Vector3d>& p_BT, double cone_half_angle,
      systems::Context<double>* context);

  ~GazeTargetConstraint() override{};

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::logic_error(
        "GazeTargetConstraint::DoEval() does not work for symbolic variables.");
  }
  const multibody_plant::MultibodyPlant<double>& plant_;
  const Frame<double>& frameA_;
  const Eigen::Vector3d p_AS_;
  const Eigen::Vector3d n_A_;
  const Frame<double>& frameB_;
  const Eigen::Vector3d p_BT_;
  const double cone_half_angle_;
  const double cos_cone_half_angle_;
  systems::Context<double>* const context_;
};
}  // namespace multibody
}  // namespace drake
