#pragma once

#include <string>
#include <vector>

#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

struct PlannerConfiguration {
  std::string model_path;
  std::string end_effector_name;
  int iiwa_base_optitrack_id{};
  int target_optitrack_id{};
  Vector3<double> target_dimensions;
  std::vector<int> table_optitrack_ids;
  std::vector<double> table_radii;
  double period_sec{0.01};
};

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
