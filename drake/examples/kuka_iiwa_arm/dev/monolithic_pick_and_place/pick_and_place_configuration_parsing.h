#pragma once

#include <string>

#include "drake/common/type_safe_index.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/pick_and_place_configuration.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

using RobotBaseIndex = TypeSafeIndex<class RobotBaseTag>;
using TargetIndex = TypeSafeIndex<class TargetTag>;

PlannerConfiguration ParsePlannerConfigurationOrThrow(
    std::string filename, RobotBaseIndex robot_base_index = RobotBaseIndex(0),
    TargetIndex target_index = TargetIndex(0));

SimulatedPlantConfiguration ParseSimulatedPlantConfigurationOrThrow(
    const std::string& filename);

OptitrackConfiguration ParseOptitrackConfigurationOrThrow(
    const std::string& filename);

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
