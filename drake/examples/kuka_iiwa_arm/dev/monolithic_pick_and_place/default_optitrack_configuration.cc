#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/default_optitrack_configuration.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

OptitrackConfiguration DefaultOptitrackConfiguration() {
  OptitrackConfiguration optitrack_configuration{};
  optitrack_configuration.AddObject("cube", 3, "simple_cuboid.urdf",
      Eigen::Vector3d(0.06, 0.06, 0.06));
  optitrack_configuration.AddObject("big_yellow_robot", 7, "big_robot_toy.urdf",
      Eigen::Vector3d(0.1, 0.035, 0.18));
  optitrack_configuration.AddObject("big_blue_robot", 5, "big_robot_toy.urdf",
      Eigen::Vector3d(0.1, 0.035, 0.18));
  optitrack_configuration.AddObject("round_table_1", 8, "round_table.urdf");
  optitrack_configuration.AddObject("round_table_2", 9, "round_table.urdf");
  optitrack_configuration.AddObject("round_table_3", 10, "round_table.urdf");
  optitrack_configuration.AddObject("round_table_4", 11, "round_table.urdf");
  optitrack_configuration.AddObject("folding_table_1", 12, "round_table.urdf");
  optitrack_configuration.AddObject("folding_table_2", 13, "round_table.urdf");
  optitrack_configuration.AddObject("folding_table_3", 14, "round_table.urdf");
  optitrack_configuration.AddObject("folding_table_4", 15, "round_table.urdf");
  optitrack_configuration.AddObject("iiwa_base_right", 1);
  optitrack_configuration.AddObject("iiwa_base_left", 6);
  return optitrack_configuration;
}

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
