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

struct SimulatedPlantConfiguration {
  std::vector<Isometry3<double>> robot_base_poses;
  std::vector<std::string> table_models;
  std::vector<Isometry3<double>> table_poses;
  std::vector<std::string> object_models;
  std::vector<Isometry3<double>> object_poses;
  double static_friction_coef{0.9};
  double dynamic_friction_coef{0.5};
  double v_stiction_tolerance{0.01};
  double stiffness{10000};
  double dissipation{2};
};

struct OptitrackConfiguration {
  std::vector<int> robot_base_optitrack_ids{};
  std::vector<int> object_optitrack_ids{};
  std::vector<int> table_optitrack_ids;
};

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
