#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/pick_and_place_configuration_parsing.h"

#include <algorithm>

#include "google/protobuf/text_format.h"

#include "drake/common/drake_throw.h"
#include "drake/common/proto/protobuf.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/pick_and_place_configuration.pb.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

PlannerConfiguration ParsePlannerConfigurationOrThrow(
    std::string filename, RobotBaseIndex robot_base_index,
    TargetIndex target_index) {
  PlannerConfiguration planner_configuration;
  auto istream = drake::MakeFileInputStreamOrThrow(filename);
  proto::PickAndPlaceConfiguration configuration;
  google::protobuf::TextFormat::Parse(istream.get(), &configuration);

  // Extract Optitrack Ids for tables
  std::transform(configuration.table().begin(), configuration.table().end(),
                 std::back_inserter(planner_configuration.table_optitrack_ids),
                 [](const proto::Model& model) -> int {
                   return model.optitrack_info().id();
                 });

  // Extract table radii
  std::transform(configuration.table().begin(), configuration.table().end(),
                 std::back_inserter(planner_configuration.table_radii),
                 [&configuration](const proto::Model& model) -> double {
                   DRAKE_THROW_UNLESS(
                       configuration.planning_geometry().find(model.name()) !=
                       configuration.planning_geometry().end());
                   const proto::Geometry& geometry =
                       configuration.planning_geometry().at(model.name());
                   double radius{};
                   switch (geometry.geometry_case()) {
                     case proto::Geometry::kBox: {
                       radius = *std::min_element(geometry.box().size().begin(),
                                                  geometry.box().size().end());
                     } break;
                     case proto::Geometry::kCylinder: {
                       radius = geometry.cylinder().radius();
                     } break;
                     case proto::Geometry::GEOMETRY_NOT_SET: {
                       DRAKE_THROW_UNLESS(geometry.geometry_case() !=
                                          proto::Geometry::GEOMETRY_NOT_SET);
                     }
                   }
                   return radius;
                 });

  // Extract IIWA Optitrack ID
  DRAKE_THROW_UNLESS(robot_base_index < configuration.robot_base_size());
  planner_configuration.iiwa_base_optitrack_id =
      configuration.robot_base(robot_base_index).optitrack_info().id();

  // Extract target Optitrack ID
  DRAKE_THROW_UNLESS(target_index < configuration.object_size());
  const proto::Model& target = configuration.object(target_index);
  planner_configuration.target_optitrack_id = target.optitrack_info().id();

  // Extract target dimensions
  DRAKE_THROW_UNLESS(configuration.planning_geometry().find(target.name()) !=
                     configuration.planning_geometry().end());
  const proto::Geometry& target_geometry =
      configuration.planning_geometry().at(target.name());
  switch (target_geometry.geometry_case()) {
    case proto::Geometry::kBox: {
      const proto::Geometry::Box& box = target_geometry.box();
      planner_configuration.target_dimensions =
          Vector3<double>(box.size(0), box.size(1), box.size(2));
    } break;
    case proto::Geometry::kCylinder: {
      proto::Geometry::Cylinder cylinder = target_geometry.cylinder();
      planner_configuration.target_dimensions = Vector3<double>(
          2 * cylinder.radius(), 2 * cylinder.radius(), cylinder.length());
    } break;
    case proto::Geometry::GEOMETRY_NOT_SET: {
      DRAKE_THROW_UNLESS(target_geometry.geometry_case() !=
                         proto::Geometry::GEOMETRY_NOT_SET);
    }
  }
  planner_configuration.model_path = configuration.planning_model_path();
  planner_configuration.end_effector_name =
      configuration.planning_end_effector_name();
  return planner_configuration;
}

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
