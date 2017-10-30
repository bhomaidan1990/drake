#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/pick_and_place_configuration_parsing.h"

#include <algorithm>

#include "google/protobuf/text_format.h"

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/proto/protobuf.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/pick_and_place_configuration.pb.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

using math::rpy2rotmat;
using manipulation::util::WorldSimTreeBuilder;

namespace {
proto::PickAndPlaceConfiguration ReadProtobufFileOrThrow(
    const std::string& filename) {
  auto istream =
      drake::MakeFileInputStreamOrThrow(FindResourceOrThrow(filename));
  proto::PickAndPlaceConfiguration configuration;
  google::protobuf::TextFormat::Parse(istream.get(), &configuration);
  return configuration;
}

Isometry3<double> ParsePose(const proto::Pose& pose) {
  Isometry3<double> X{Isometry3<double>::Identity()};
  if (!pose.xyz().empty()) {
    DRAKE_THROW_UNLESS(pose.xyz_size() == 3);
    X.translation() = Vector3<double>(pose.xyz(0), pose.xyz(1), pose.xyz(2));
  }
  if (!pose.rpy().empty()) {
    DRAKE_THROW_UNLESS(pose.rpy_size() == 3);
    X.linear() =
        rpy2rotmat(Vector3<double>(pose.rpy(0), pose.rpy(1), pose.rpy(2)));
  }
  return X;
}

const std::string& GetModelPathOrThrow(
    const proto::PickAndPlaceConfiguration& configuration,
    const std::string& model_name) {
  DRAKE_THROW_UNLESS(configuration.model_path().find(model_name) !=
                     configuration.model_path().end());
  return configuration.model_path().at(model_name);
}

void ExtractBasePosesForModels(
    const google::protobuf::RepeatedPtrField<proto::Model>& models,
    std::vector<Isometry3<double>>* poses) {
  DRAKE_DEMAND(poses != nullptr);
  std::transform(models.cbegin(), models.cend(), std::back_inserter(*poses),
                 [](const proto::Model& table) -> Isometry3<double> {
                   return ParsePose(table.pose());
                 });
}

void ExtractOptitrackInfoForModels(
    const google::protobuf::RepeatedPtrField<proto::Model>& models,
    std::vector<pick_and_place::OptitrackInfo>* optitrack_info) {
  DRAKE_DEMAND(optitrack_info != nullptr);
  std::transform(
      models.cbegin(), models.cend(), std::back_inserter(*optitrack_info),
      [](const proto::Model& model) -> pick_and_place::OptitrackInfo {
        return pick_and_place::OptitrackInfo(
            {model.optitrack_info().id(),
             ParsePose(model.optitrack_info().x_mf())});
      });
}

void ExtractModelPathsForModels(
    const proto::PickAndPlaceConfiguration& configuration,
    const google::protobuf::RepeatedPtrField<proto::Model>& models,
    std::vector<std::string>* model_paths) {
  DRAKE_DEMAND(model_paths != nullptr);
  std::transform(models.cbegin(), models.cend(),
                 std::back_inserter(*model_paths),
                 [&configuration](const proto::Model& table) -> std::string {
                   return GetModelPathOrThrow(configuration, table.name());
                 });
}
}  // namespace

pick_and_place::PlannerConfiguration ParsePlannerConfigurationOrThrow(
    std::string filename, pick_and_place::RobotBaseIndex robot_index,
    pick_and_place::TargetIndex target_index) {
  pick_and_place::PlannerConfiguration planner_configuration;
  planner_configuration.robot_index = robot_index;
  planner_configuration.target_index = target_index;

  // Read configuration file
  const proto::PickAndPlaceConfiguration configuration{
      ReadProtobufFileOrThrow(filename)};

  // Check that the robot base and target indices are valid.
  DRAKE_THROW_UNLESS(robot_index < configuration.robot_size());
  DRAKE_THROW_UNLESS(target_index < configuration.object_size());

  // Extract number of tables
  planner_configuration.num_tables = configuration.table_size();

  // Extract target dimensions
  WorldSimTreeBuilder<double> tree_builder;
  tree_builder.StoreModel(
      "target",
      GetModelPathOrThrow(
          configuration,
          configuration.object(planner_configuration.target_index).name()));
  tree_builder.AddFixedModelInstance("target", Vector3<double>::Zero());
  auto target = tree_builder.Build();
  Vector3<double> max_corner{-std::numeric_limits<double>::infinity() *
                             Vector3<double>::Ones()};
  Vector3<double> min_corner{std::numeric_limits<double>::infinity() *
                             Vector3<double>::Ones()};
  // The target dimensions for planning will be the side lengths of the
  // axis-aligned bounding box for all VISUAL geometry of the target. Ideally,
  // this would be the collision geometry, but RigidBody doesn't give const
  // access to collision geometries.
  for (const auto& visual_element :
       target->get_body(target->FindBaseBodies()[0]).get_visual_elements()) {
    Matrix3X<double> bounding_box_points;
    visual_element.getGeometry().getBoundingBoxPoints(bounding_box_points);
    const Isometry3<double> X_BV = visual_element.getLocalTransform();
    max_corner =
        max_corner.cwiseMax((X_BV * bounding_box_points).rowwise().maxCoeff());
    min_corner =
        min_corner.cwiseMin((X_BV * bounding_box_points).rowwise().minCoeff());
  }
  planner_configuration.target_dimensions = max_corner - min_corner;

  planner_configuration.model_path = configuration.planning_model_path();
  planner_configuration.end_effector_name =
      configuration.planning_end_effector_name();
  return planner_configuration;
}

pick_and_place::SimulatedPlantConfiguration
ParseSimulatedPlantConfigurationOrThrow(const std::string& filename) {
  pick_and_place::SimulatedPlantConfiguration plant_configuration;

  // Read configuration file
  const proto::PickAndPlaceConfiguration configuration{
      ReadProtobufFileOrThrow(filename)};

  // Extract base positions
  ExtractBasePosesForModels(configuration.robot(),
                            &plant_configuration.robot_poses);

  // Extract table model paths
  ExtractModelPathsForModels(configuration, configuration.table(),
                             &plant_configuration.table_models);

  // Extract table poses
  ExtractBasePosesForModels(configuration.table(),
                            &plant_configuration.table_poses);

  // Extract object model paths
  ExtractModelPathsForModels(configuration, configuration.object(),
                             &plant_configuration.object_models);

  // Extract object poses
  ExtractBasePosesForModels(configuration.object(),
                            &plant_configuration.object_poses);

  if (configuration.static_friction_coefficient() > 0) {
    plant_configuration.static_friction_coef =
        configuration.static_friction_coefficient();
  }
  if (configuration.dynamic_friction_coefficient() > 0) {
    plant_configuration.dynamic_friction_coef =
        configuration.dynamic_friction_coefficient();
  }
  if (configuration.v_stiction_tolerance() > 0) {
    plant_configuration.v_stiction_tolerance =
        configuration.v_stiction_tolerance();
  }
  if (configuration.stiffness() > 0) {
    plant_configuration.stiffness = configuration.stiffness();
  }
  if (configuration.dissipation() > 0) {
    plant_configuration.dissipation = configuration.dissipation();
  }

  return plant_configuration;
}

pick_and_place::OptitrackConfiguration ParseOptitrackConfigurationOrThrow(
    const std::string& filename) {
  pick_and_place::OptitrackConfiguration optitrack_configuration;

  // Read configuration file
  const proto::PickAndPlaceConfiguration configuration{
      ReadProtobufFileOrThrow(filename)};

  // Extract Optitrack info for tables
  ExtractOptitrackInfoForModels(configuration.table(),
                                &optitrack_configuration.table_optitrack_info);

  // Extract Optitrack Ids for objects
  ExtractOptitrackInfoForModels(configuration.object(),
                                &optitrack_configuration.object_optitrack_info);

  // Extract Optitrack Ids for robot bases
  ExtractOptitrackInfoForModels(
      configuration.robot(),
      &optitrack_configuration.robot_base_optitrack_info);

  return optitrack_configuration;
}

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
