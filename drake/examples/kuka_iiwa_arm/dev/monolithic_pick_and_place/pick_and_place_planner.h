#pragma once

#include "drake/systems/framework/diagram.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/pick_and_place_configuration.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

/// A custom `systems::Diagram` composed of a `PickAndPlaceStateMachineSystem`
/// and `drake::manipulation::OptitrackPoseExtractor` systems.
class PickAndPlacePlanner : public systems::Diagram<double> {
 public:
  PickAndPlacePlanner(const PlannerConfiguration& configuration);

  /**
   * Getter for the input port corresponding to the abstract input with iiwa
   * state message (LCM `robot_state_t` message).
   * @return The corresponding `sytems::InputPortDescriptor`.
   */
  const systems::InputPortDescriptor<double>& get_input_port_iiwa_state()
      const {
    return this->get_input_port(input_port_iiwa_state_);
  }

  /**
   * Getter for the input port corresponding to the abstract input with the wsg
   * status message (LCM `lcmt_schunk_wsg_status` message).
   * @return The corresponding `sytems::InputPortDescriptor`.
   */
  const systems::InputPortDescriptor<double>& get_input_port_wsg_status()
      const {
    return this->get_input_port(input_port_wsg_status_);
  }

  /**
   * Getter for the input port corresponding to the abstract input with the
   * optitrack message (LCM `optitrack::optitrack_frame_t` message).
   * @return The corresponding `sytems::InputPortDescriptor`.
   */
  const systems::InputPortDescriptor<double>& get_input_port_optitrack_message()
      const {
    return get_input_port(input_port_optitrack_message_);
  }

  const systems::OutputPort<double>& get_output_port_iiwa_plan() const {
    return this->get_output_port(output_port_iiwa_plan_);
  }

  const systems::OutputPort<double>& get_output_port_wsg_command() const {
    return this->get_output_port(output_port_wsg_command_);
  }

 private:
  // Input ports.
  int input_port_iiwa_state_{-1};
  int input_port_wsg_status_{-1};
  int input_port_optitrack_message_{-1};

  // Output ports.
  int output_port_iiwa_plan_{-1};
  int output_port_wsg_command_{-1};
};
}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
