#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/pick_and_place_planner.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

PickAndPlacePlanner::PickAndPlacePlanner(const std::string& model_path,
		const std::string& end_effector_name,
		const Isometry3<double>& iiwa_base, int num_tables,
		const Vector3<double>& object_dimensions,
		const double period_sec) {
}

}  // namespace drake
}  // namespace examples
}  // namespace kuka_iiwa_arm
