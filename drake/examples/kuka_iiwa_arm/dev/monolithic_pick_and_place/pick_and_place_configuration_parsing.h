#pragma once

#include <string>

#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_configuration.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

pick_and_place::PlannerConfiguration ParsePlannerConfigurationOrThrow(
    std::string filename, pick_and_place::RobotBaseIndex robot_base_index =
                              pick_and_place::RobotBaseIndex(0),
    pick_and_place::TargetIndex target_index = pick_and_place::TargetIndex(0));

pick_and_place::SimulatedPlantConfiguration
ParseSimulatedPlantConfigurationOrThrow(const std::string& filename);

pick_and_place::OptitrackConfiguration ParseOptitrackConfigurationOrThrow(
    const std::string& filename);

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
