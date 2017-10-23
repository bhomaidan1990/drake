#pragma once

#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/pick_and_place_planner.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/pick_and_place_plant.h"
#include "drake/examples/kuka_iiwa_arm/lcm_plan_interpolator.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_configuration.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

/// A custom `systems::Diagram`
//
//     +-------------------------------------------------------------+
//     |                                                             |
//     |    +-------+  wsg command                                   |
//     +--->|Planner|---------------+    +--------+                  |
//          |       |               |    |        | iiwa robot_state |
//  +------>|       |               |    |        |------------------+
//  |       |       |  plan         |    |        |
//  |  +--->|       |-------+       |    |        | vis info   +----------+
//  |  |    +-------+       |       +--->|  Plant |----------->|Visualizer|
//  |  |                    |            |        |            +----------+
//  |  |  +-----------------+            |        | optitrack
//  |  |  |                              |        |------------------+
//  |  |  |   +------------+     iiwa    |        |                  |
//  |  |  +-->|            |  command    |        | wsg_status       |
//  |  |      |            |------------>|        |--------------+   |
//  |  |      |            |             |        |              |   |
//  |  |  +-->|Interpolator|             |        | iiwa_status  |   |
//  |  |  |   +------------+             |        |------------+ |   |
//  |  |  |                              +--------+            | |   |
//  |  |  |                    +------+                        | |   |
//  |  |  +--------------------| ZOH  |<-----------------------+ |   |
//  |  |                       +------+                          |   |
//  |  +---------------------------------------------------------+   |
//  |                                                                |
//  +----------------------------------------------------------------+
//
//
class MonolithicPickAndPlaceSystem : public systems::Diagram<double> {
 public:
  MonolithicPickAndPlaceSystem(
      const pick_and_place::SimulatedPlantConfiguration& plant_configuration,
      const pick_and_place::OptitrackConfiguration& optitrack_configuration,
      const std::vector<pick_and_place::PlannerConfiguration>&
          planner_configurations,
      bool single_move);

  const systems::OutputPort<double>& get_output_port_contact_results() const {
    return this->get_output_port(output_port_contact_results_);
  }

  const systems::OutputPort<double>& get_output_port_plant_state() const {
    return this->get_output_port(output_port_plant_state_);
  }

  const RigidBodyTree<double>& get_tree() const {
    return this->plant_->get_tree();
  }

  bool is_done(const systems::Context<double>& context) const;

  void Initialize(systems::Context<double>* context);

  const pick_and_place::WorldState& world_state(
      const systems::Context<double>& context, int index) const;

 private:
  // Output ports.
  int output_port_contact_results_{-1};
  int output_port_plant_state_{-1};

  // Subsystems
  PickAndPlacePlant* plant_{};
  std::vector<PickAndPlacePlanner*> planners_{};
  std::vector<LcmPlanInterpolator*> plan_interpolators_{};
};
}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
