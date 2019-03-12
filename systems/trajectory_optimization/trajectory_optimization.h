#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
class TrajectoryOptimization : public solvers::MathematicalProgram {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TrajectoryOptimization)

  TrajectoryOptimization() = default;

  virtual solvers::VectorXDecisionVariable NewTimeVaryingVariables(
      int rows, const std::string& name) = 0;

  virtual void AddRunningCost(const symbolic::Formula& cost) = 0;

  virtual void AddFinalCost(const symbolic::Formula& cost) = 0;
};

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
