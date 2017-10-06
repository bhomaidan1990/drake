#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/optitrack_configuration.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

OptitrackConfiguration DefaultOptitrackConfiguration();

const std::vector<std::string> kOptitrackTableNames{
      "round_table_1",   "round_table_2",   "round_table_3",
      "round_table_4",   "folding_table_1", "folding_table_2",
      "folding_table_3", "folding_table_4"};

const std::vector<std::string> kOptitrackIiwaBaseNames{"iiwa_base_right",
                                                       "iiwa_base_left"};

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
