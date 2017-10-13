#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/pick_and_place_planner.h"

#include "optitrack/optitrack_frame_t.hpp"

#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/state_machine_system.h"
#include "drake/manipulation/perception/optitrack_pose_extractor.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/util/lcmUtil.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

using manipulation::perception::OptitrackPoseExtractor;
using systems::DiagramBuilder;
using systems::PassThrough;
using systems::Value;

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
    const Isometry3<double>& in = this->EvalAbstractInput(context, 0)
                                      ->template GetValue<Isometry3<double>>();

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

  const systems::InputPortDescriptor<double>& get_input_port_joint_state()
      const {
    return get_input_port(input_port_joint_state_);
  }

  const systems::InputPortDescriptor<double>& get_input_port_base_state()
      const {
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
}  // namespace

PickAndPlacePlanner::PickAndPlacePlanner(const PlannerConfiguration& configuration) {
  DiagramBuilder<double> builder;

  auto state_machine = builder.AddSystem<PickAndPlaceStateMachineSystem>(
      configuration.model_path, configuration.end_effector_name,
      Isometry3<double>::Identity() /*iiwa_base*/, configuration.table_radii,
      configuration.target_dimensions);

  // Export input ports for WSG status message.
  input_port_wsg_status_ =
      builder.ExportInput(state_machine->get_input_port_wsg_status());

  // The Optitrack message needs to be passed to multiple OptitrackPoseExtractor
  // blocks, this pass-through block allows that.
  auto optitrack_message_passthrough = builder.AddSystem<PassThrough<double>>(
      Value<optitrack::optitrack_frame_t>());

  // Export input port for the Optitrack message.
  input_port_optitrack_message_ =
      builder.ExportInput(optitrack_message_passthrough->get_input_port());

  const Isometry3<double>& X_WO{Isometry3<double>::Identity()};
  const double kOptitrackLcmStatusPeriod{1.0 / 120.0};

  // Connect blocks for target.
  auto optitrack_target_pose_extractor =
      builder.AddSystem<OptitrackPoseExtractor>(
          configuration.target_optitrack_id, X_WO, kOptitrackLcmStatusPeriod);
  optitrack_target_pose_extractor->set_name("Optitrack target pose extractor");

  auto optitrack_target_translator =
      builder.AddSystem<OptitrackTranslatorSystem>();
  optitrack_target_translator->set_name("Optitrack target translator");

  builder.Connect(optitrack_message_passthrough->get_output_port(),
                  optitrack_target_pose_extractor->get_input_port(0));
  builder.Connect(optitrack_target_pose_extractor->get_output_port(0),
                  optitrack_target_translator->get_input_port(0));
  builder.Connect(optitrack_target_translator->get_output_port(0),
                  state_machine->get_input_port_box_state());

  // Connect Optitrack blocks for tables.
  for (int i = 0;
       i < static_cast<int>(configuration.table_optitrack_ids.size()); ++i) {
    auto optitrack_table_pose_extractor =
        builder.AddSystem<manipulation::perception::OptitrackPoseExtractor>(
            configuration.table_optitrack_ids[i], X_WO,
            kOptitrackLcmStatusPeriod);
    optitrack_table_pose_extractor->set_name(
        "Optitrack table " + std::to_string(i) + "  pose extractor");
    builder.Connect(optitrack_message_passthrough->get_output_port(),
                    optitrack_table_pose_extractor->get_input_port(0));
    builder.Connect(optitrack_table_pose_extractor->get_output_port(0),
                    state_machine->get_input_port_table_state(i));
  }

  // Connect Optitrack blocks for IIWA base.
  auto optitrack_iiwa_base_pose_extractor =
      builder.AddSystem<OptitrackPoseExtractor>(
          configuration.iiwa_base_optitrack_id, X_WO,
          kOptitrackLcmStatusPeriod);
  optitrack_iiwa_base_pose_extractor->set_name(
      "Optitrack IIWA base pose extractor");

  auto optitrack_iiwa_base_translator =
      builder.AddSystem<OptitrackTranslatorSystem>();
  optitrack_iiwa_base_translator->set_name("Optitrack iiwa base translator");

  builder.Connect(optitrack_message_passthrough->get_output_port(),
                  optitrack_iiwa_base_pose_extractor->get_input_port(0));
  builder.Connect(optitrack_iiwa_base_pose_extractor->get_output_port(0),
                  optitrack_iiwa_base_translator->get_input_port(0));

  // Replace the base pose information in the incoming IIWA status message with
  // the pose reported by Optitrack.
  auto iiwa_state_splicer = builder.AddSystem<RobotStateSplicer>();
  builder.Connect(optitrack_iiwa_base_translator->get_output_port(0),
                  iiwa_state_splicer->get_input_port_base_state());
  builder.Connect(iiwa_state_splicer->get_output_port(0),
                  state_machine->get_input_port_iiwa_state());

  // Export input port for IIWA status message.
  input_port_iiwa_state_ =
      builder.ExportInput(iiwa_state_splicer->get_input_port_joint_state());

  // Export output ports.
  output_port_iiwa_plan_ =
      builder.ExportOutput(state_machine->get_output_port_iiwa_plan());
  output_port_wsg_command_ =
      builder.ExportOutput(state_machine->get_output_port_wsg_command());

  // Build the system.
  builder.BuildInto(this);
}

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
