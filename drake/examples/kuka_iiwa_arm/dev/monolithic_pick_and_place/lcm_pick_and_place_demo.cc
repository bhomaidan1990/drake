#include <memory>
#include <string>
#include <vector>

#include <gflags/gflags.h>
#include "bot_core/robot_state_t.hpp"
#include "google/protobuf/text_format.h"
#include "optitrack/optitrack_frame_t.hpp"
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/find_resource.h"
#include "drake/common/proto/protobuf.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/common/eigen_types.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/pick_and_place_planner.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/state_machine_system.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/pick_and_place_configuration.pb.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/lcmtypes/drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmtypes/drake/lcmt_viewer_geometry_data.hpp"
#include "drake/lcmtypes/drake/lcmt_viewer_link_data.hpp"
#include "drake/manipulation/perception/optitrack_pose_extractor.h"
#include "drake/multibody/shapes/geometry.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_driven_loop.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/util/lcmUtil.h"

DEFINE_int32(target, 0, "Index of the target to pick.");
DEFINE_int32(iiwa_index, 0, "Index of the iiwa to use.");
DEFINE_int32(end_position, -1, "Position index to end at");
DEFINE_bool(use_channel_suffix, true,
            "If true, append a suffix to channel names");
DEFINE_string(configuration_file,
              "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/"
              "configuration/default.pick_and_place_configuration",
              "Path to the configuration file.");

using DrakeShapes::Geometry;
using robotlocomotion::robot_plan_t;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {
namespace {

const char kIiwaUrdf[] =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";
const char kIiwaEndEffectorName[] = "iiwa_link_ee";

int DoMain(void) {
  std::string suffix =
      (FLAGS_use_channel_suffix) ? "_" + std::to_string(FLAGS_iiwa_index) : "";

  lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;

  // Parse configuration file
  auto istream = drake::MakeFileInputStreamOrThrow(
      FindResourceOrThrow(FLAGS_configuration_file));
  proto::PickAndPlaceConfiguration configuration;
  google::protobuf::TextFormat::Parse(istream.get(), &configuration);

  // Extract Optitrack Ids for tables
  std::vector<int> table_optitrack_ids;
  std::transform(configuration.table().begin(), configuration.table().end(),
                 std::back_inserter(table_optitrack_ids),
                 [](const proto::Model& model) -> int {
                   return model.optitrack_info().id();
                 });

  // Extract table radii
  std::vector<double> table_radii;
  std::transform(configuration.table().begin(), configuration.table().end(),
                 std::back_inserter(table_radii),
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
  const int iiwa_optitrack_id =
      configuration.robot_base(FLAGS_iiwa_index).optitrack_info().id();

  // Extract target Optitrack ID
  const proto::Model& target = configuration.object(FLAGS_target);
  const int target_optitrack_id = target.optitrack_info().id();

  // Extract target dimensions
  DRAKE_THROW_UNLESS(configuration.planning_geometry().find(target.name()) !=
                     configuration.planning_geometry().end());
  Vector3<double> target_dimensions;
  const proto::Geometry& target_geometry =
      configuration.planning_geometry().at(target.name());
  switch (target_geometry.geometry_case()) {
    case proto::Geometry::kBox: {
      const proto::Geometry::Box& box = target_geometry.box();
      target_dimensions = Vector3<double>(box.size(0), box.size(1), box.size(2));
    } break;
    case proto::Geometry::kCylinder: {
      proto::Geometry::Cylinder cylinder = target_geometry.cylinder();
      target_dimensions = Vector3<double>(
          2 * cylinder.radius(), 2 * cylinder.radius(), cylinder.length());
    } break;
    case proto::Geometry::GEOMETRY_NOT_SET: {
      DRAKE_THROW_UNLESS(target_geometry.geometry_case() !=
                         proto::Geometry::GEOMETRY_NOT_SET);
    }
  }

  // The PickAndPlacePlanner block contains all of the demo logic.
  auto planner = builder.AddSystem<PickAndPlacePlanner>(
      FindResourceOrThrow(kIiwaUrdf), kIiwaEndEffectorName, iiwa_optitrack_id,
      target_optitrack_id, target_dimensions, table_optitrack_ids, table_radii);

  // LCM Subscribers
  auto iiwa_status_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<bot_core::robot_state_t>(
          "EST_ROBOT_STATE" + suffix, &lcm));

  auto wsg_status_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_schunk_wsg_status>(
          "SCHUNK_WSG_STATUS" + suffix, &lcm));

  auto optitrack_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<optitrack::optitrack_frame_t>(
          "OPTITRACK_FRAMES", &lcm));

  // LCM Publishers
  auto iiwa_plan_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<robot_plan_t>(
          "COMMITTED_ROBOT_PLAN" + suffix, &lcm));
  auto wsg_command_pub = builder.AddSystem(
    systems::lcm::LcmPublisherSystem::Make<lcmt_schunk_wsg_command>(
        "SCHUNK_WSG_COMMAND" + suffix, &lcm));

  // Connect subscribers to planner input ports.
  builder.Connect(wsg_status_sub->get_output_port(0),
                  planner->get_input_port_wsg_status());
  builder.Connect(iiwa_status_sub->get_output_port(0),
                  planner->get_input_port_iiwa_state());
  builder.Connect(optitrack_sub->get_output_port(0),
                  planner->get_input_port_optitrack_message());

  // Connect publishers to planner output ports.
  builder.Connect(planner->get_output_port_iiwa_plan(),
                  iiwa_plan_pub->get_input_port(0));
  builder.Connect(planner->get_output_port_wsg_command(),
                  wsg_command_pub->get_input_port(0));

  auto sys = builder.Build();

  systems::lcm::LcmDrivenLoop loop(
      *sys, *iiwa_status_sub, nullptr, &lcm,
      std::make_unique<systems::lcm::UtimeMessageToSeconds<
      bot_core::robot_state_t>>());
  drake::log()->debug("Waiting for optitrack message ...");
  // Wait for the first optitrack message before doing anything else. 
  optitrack_sub->WaitForMessage(0);
  drake::log()->debug("Optitrack message received.");

  // Waits for the first message.
  drake::log()->debug("Waiting for IIWA status message ...");
  const systems::AbstractValue& first_msg = loop.WaitForMessage();
  drake::log()->debug("IIWA status message received.");
  double msg_time =
      loop.get_message_to_time_converter().GetTimeInSeconds(first_msg);
  systems::Context<double>* diagram_context = loop.get_mutable_context();
  // Explicit initialization.
  diagram_context->set_time(msg_time);


  loop.RunToSecondsAssumingInitialized();

  return 0;
}

}  // namespace
}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::kuka_iiwa_arm::monolithic_pick_and_place::DoMain();
}
