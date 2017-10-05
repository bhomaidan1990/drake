#include <memory>
#include <string>
#include <vector>

#include <gflags/gflags.h>
#include "bot_core/robot_state_t.hpp"
#include "optitrack/optitrack_frame_t.hpp"
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/state_machine_system.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/optitrack_configuration.h"
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

DEFINE_int32(target, 0, "ID of the target to pick.");
DEFINE_int32(iiwa_index, 0, "ID of the iiwa to use.");
DEFINE_int32(end_position, -1, "Position index to end at");
DEFINE_bool(use_channel_suffix, true,
            "If true, append a suffix to channel names");

using robotlocomotion::robot_plan_t;
using DrakeShapes::Geometry;
using drake::examples::kuka_iiwa_arm::monolithic_pick_and_place::
    OptitrackConfiguration;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {
namespace {

class OptitrackTranslatorSystem : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OptitrackTranslatorSystem);

  OptitrackTranslatorSystem() {
    this->DeclareAbstractInputPort(
        systems::Value<Isometry3<double>>(Isometry3<double>::Identity()));
    this->DeclareAbstractOutputPort(bot_core::robot_state_t(),
                                    &OptitrackTranslatorSystem::OutputPose);
  }

 private:
  void OutputPose(const systems::Context<double>& context,
                  bot_core::robot_state_t* out) const {
    const Isometry3<double>& in = this->EvalAbstractInput(
        context, 0)->template GetValue<Isometry3<double>>();

    out->utime = 0;
    EncodePose(in, out->pose);
  }
};

class RobotStateSplicer : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RobotStateSplicer);

  RobotStateSplicer() {
    input_port_joint_state_ =
        DeclareAbstractInputPort(
            systems::Value<bot_core::robot_state_t>(bot_core::robot_state_t()))
            .get_index();
    input_port_base_state_ =
        DeclareAbstractInputPort(
            systems::Value<bot_core::robot_state_t>(bot_core::robot_state_t()))
            .get_index();
    DeclareAbstractOutputPort(bot_core::robot_state_t(),
                              &RobotStateSplicer::SpliceStates)
        .get_index();
  }

  const systems::InputPortDescriptor<double>& get_input_port_joint_state() const {
    return get_input_port(input_port_joint_state_);
  }

  const systems::InputPortDescriptor<double>& get_input_port_base_state() const {
    return get_input_port(input_port_base_state_);
  }
 private:
  void SpliceStates(const systems::Context<double>& context,
                    bot_core::robot_state_t* out) const {
    const bot_core::robot_state_t& joint_state =
        this->EvalAbstractInput(context, input_port_joint_state_)
            ->GetValue<bot_core::robot_state_t>();
    const bot_core::robot_state_t& base_state =
        this->EvalAbstractInput(context, input_port_base_state_)
            ->GetValue<bot_core::robot_state_t>();
    *out = joint_state;
    out->pose = base_state.pose;
    out->twist = base_state.twist;
  }

  int input_port_joint_state_{-1};
  int input_port_base_state_{-1};
};

const char kIiwaUrdf[] =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";
const char kIiwaEndEffectorName[] = "iiwa_link_ee";

const OptitrackConfiguration kOptitrackConfiguration;

int DoMain(void) {
  std::string suffix =
      (FLAGS_use_channel_suffix) ? "_" + std::to_string(FLAGS_iiwa_index) : "";
  OptitrackConfiguration::Target target =
      kOptitrackConfiguration.target(FLAGS_target);

  lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;

  const Eigen::Vector3d robot_base(0, 0, 0);
  Isometry3<double> iiwa_base = Isometry3<double>::Identity();
  iiwa_base.translation() = robot_base;

  auto state_machine =
      builder.AddSystem<PickAndPlaceStateMachineSystem>(
          FindResourceOrThrow(kIiwaUrdf), kIiwaEndEffectorName,
          iiwa_base, kOptitrackConfiguration.num_tables(), target.dimensions);

  auto iiwa_status_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<bot_core::robot_state_t>(
          "EST_ROBOT_STATE" + suffix, &lcm));

  auto wsg_status_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_schunk_wsg_status>(
          "SCHUNK_WSG_STATUS" + suffix, &lcm));
  builder.Connect(wsg_status_sub->get_output_port(0),
                  state_machine->get_input_port_wsg_status());

  auto optitrack_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<optitrack::optitrack_frame_t>(
          "OPTITRACK_FRAMES", &lcm));

  const Isometry3<double>& X_WO{kOptitrackConfiguration.optitrack_frame()};

  auto optitrack_obj_pose_extractor =
      builder.AddSystem<manipulation::perception::OptitrackPoseExtractor>(
          target.object_id, X_WO, 1. / 120.);
  optitrack_obj_pose_extractor->set_name("Optitrack object pose extractor");
  builder.Connect(optitrack_sub->get_output_port(0),
                  optitrack_obj_pose_extractor->get_input_port(0));

  auto obj_optitrack_translator =
      builder.AddSystem<OptitrackTranslatorSystem>();
  obj_optitrack_translator->set_name("Optitrack object translator");
  builder.Connect(optitrack_obj_pose_extractor->get_measured_pose_output_port(),
                  obj_optitrack_translator->get_input_port(0));

  auto optitrack_iiwa_pose_extractor =
      builder.AddSystem<manipulation::perception::OptitrackPoseExtractor>(
          kOptitrackConfiguration.iiwa_base(FLAGS_iiwa_index).object_id, X_WO,
          1. / 120.);
  optitrack_iiwa_pose_extractor->set_name("Optitrack IIWA base pose extractor");
  builder.Connect(optitrack_sub->get_output_port(0),
                  optitrack_iiwa_pose_extractor->get_input_port(0));

  auto iiwa_optitrack_translator =
      builder.AddSystem<OptitrackTranslatorSystem>();
  iiwa_optitrack_translator->set_name("Optitrack iiwa base translator");
  builder.Connect(
      optitrack_iiwa_pose_extractor->get_measured_pose_output_port(),
      iiwa_optitrack_translator->get_input_port(0));

  builder.Connect(obj_optitrack_translator->get_output_port(0),
                  state_machine->get_input_port_box_state());

  auto iiwa_state_splicer = builder.AddSystem<RobotStateSplicer>();
  builder.Connect(iiwa_status_sub->get_output_port(0),
                  iiwa_state_splicer->get_input_port_joint_state());
  builder.Connect(iiwa_optitrack_translator->get_output_port(0),
                  iiwa_state_splicer->get_input_port_base_state());
  builder.Connect(iiwa_state_splicer->get_output_port(0),
                  state_machine->get_input_port_iiwa_state());

  for (int i = 0; i < kOptitrackConfiguration.num_tables(); ++i) {
    auto optitrack_table_pose_extractor =
        builder.AddSystem<manipulation::perception::OptitrackPoseExtractor>(
            kOptitrackConfiguration.table(i).object_id, X_WO, 1. / 120.);
    optitrack_table_pose_extractor->set_name(
        "Optitrack table " + std::to_string(i) + "  pose extractor");
    builder.Connect(optitrack_sub->get_output_port(0),
                    optitrack_table_pose_extractor->get_input_port(0));
    builder.Connect(optitrack_table_pose_extractor->get_output_port(0),
                    state_machine->get_input_port_table_state(i));
  }

  auto iiwa_plan_sender = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<robot_plan_t>(
          "COMMITTED_ROBOT_PLAN" + suffix, &lcm));
  auto wsg_command_sender = builder.AddSystem(
    systems::lcm::LcmPublisherSystem::Make<lcmt_schunk_wsg_command>(
        "SCHUNK_WSG_COMMAND" + suffix, &lcm));

  builder.Connect(state_machine->get_output_port_iiwa_plan(),
                  iiwa_plan_sender->get_input_port(0));
  builder.Connect(state_machine->get_output_port_wsg_command(),
                  wsg_command_sender->get_input_port(0));

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
