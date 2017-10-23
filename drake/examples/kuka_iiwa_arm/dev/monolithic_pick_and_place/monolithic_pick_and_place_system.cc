#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/monolithic_pick_and_place_system.h"

#include "drake/common/find_resource.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/zero_order_hold.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

using manipulation::planner::InterpolatorType;
using systems::ZeroOrderHold;

MonolithicPickAndPlaceSystem::MonolithicPickAndPlaceSystem(
    const pick_and_place::SimulatedPlantConfiguration& plant_configuration,
    const pick_and_place::OptitrackConfiguration& optitrack_configuration,
    const std::vector<pick_and_place::PlannerConfiguration>&
        planner_configurations,
    bool single_move) {
  systems::DiagramBuilder<double> builder;

  // Add the plant.
  plant_ = builder.AddSystem<PickAndPlacePlant>(plant_configuration,
                                                optitrack_configuration);

  // Add blocks to publish contact results.
  auto contact_viz =
      builder.AddSystem<systems::ContactResultsToLcmSystem<double>>(
          plant_->get_tree());
  builder.Connect(plant_->get_output_port_contact_results(),
                  contact_viz->get_input_port(0));

  // Export outputs for visualization
  output_port_contact_results_ =
      builder.ExportOutput(contact_viz->get_output_port(0));
  output_port_plant_state_ =
      builder.ExportOutput(plant_->get_output_port_plant_state());

  // Loop over arms to add planners and plan interpolators.
  const int kNumIiwa(plant_configuration.robot_poses.size());
  DRAKE_THROW_UNLESS(kNumIiwa ==
                     static_cast<int>(planner_configurations.size()));
  for (int i = 0; i < kNumIiwa; ++i) {
    // Add planner.
    auto planner = builder.AddSystem<PickAndPlacePlanner>(
        planner_configurations[i], optitrack_configuration, single_move);
    planners_.push_back(planner);
    // Add plan interpolator.
    auto plan_interpolator = builder.AddSystem<LcmPlanInterpolator>(
        FindResourceOrThrow(planner_configurations[i].model_path),
        InterpolatorType::Cubic);
    plan_interpolators_.push_back(plan_interpolator);
    // Add a zero-order hold between the plant and the interpolator
    auto iiwa_status_zoh = builder.AddSystem<ZeroOrderHold<double>>(
        kIiwaLcmStatusPeriod, systems::Value<lcmt_iiwa_status>());
    // Connect planner, plan interpolator, and plant.
    // Plant --> Planner
    builder.Connect(plant_->get_output_port_wsg_status(i),
                    planner->get_input_port_wsg_status());
    builder.Connect(plant_->get_output_port_iiwa_status(i),
                    planner->get_input_port_iiwa_status());
    builder.Connect(plant_->get_output_port_optitrack_frame(),
                    planner->get_input_port_optitrack_message());
    // Plant --> plan interpolator
    builder.Connect(plant_->get_output_port_iiwa_status(i),
                    iiwa_status_zoh->get_input_port());
    builder.Connect(iiwa_status_zoh->get_output_port(),
                    plan_interpolator->get_input_port_iiwa_status());
    // Planner --> Plant
    builder.Connect(planner->get_output_port_wsg_command(),
                    plant_->get_input_port_wsg_command(i));
    // Planner --> Plan Interpolator
    builder.Connect(planner->get_output_port_iiwa_plan(),
                    plan_interpolator->get_input_port_iiwa_plan());
    // plan interpolator --> plant
    builder.Connect(plan_interpolator->get_output_port_iiwa_command(),
                    plant_->get_input_port_iiwa_command(i));
  }

  builder.BuildInto(this);
}

bool MonolithicPickAndPlaceSystem::is_done(
    const systems::Context<double>& context) const {
  for (const auto& planner : planners_) {
    if (planner->state(this->GetSubsystemContext(*planner, context)) !=
        pick_and_place::PickAndPlaceState::kDone) {
      return false;
    }
  }
  return true;
}

void MonolithicPickAndPlaceSystem::Initialize(
    systems::Context<double>* context) {
  for (auto& plan_interpolator : plan_interpolators_) {
    const VectorX<double> q0{
        VectorX<double>::Zero(plan_interpolator->num_joints())};
    auto& plan_interpolator_context =
        this->GetMutableSubsystemContext(*plan_interpolator, context);
    plan_interpolator->Initialize(context->get_time(), q0,
                                  &plan_interpolator_context);
  }
}

const pick_and_place::WorldState& MonolithicPickAndPlaceSystem::world_state(
    const systems::Context<double>& context, int index) const {
  DRAKE_DEMAND(0 <= index && index < static_cast<int>(planners_.size()));
  return this->planners_[index]->world_state(
      this->GetSubsystemContext(*planners_[index], context));
}

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
