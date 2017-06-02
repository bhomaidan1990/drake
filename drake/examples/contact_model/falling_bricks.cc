#include <iomanip>
#include <limits>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/random_rotation.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace systems {

using drake::lcm::DrakeLcm;
using drake::multibody::joints::kQuaternion;
using Eigen::VectorXd;
using std::make_unique;

// Simple example of the "stiction" properties of the contact model.
// Based on the default values (50 kg brick) and the friction coefficients,
// a force of 260 N is insufficient to move the stationary block.  This is
// because the static friction is too great. However, if its initial velocity is
// 0.1 m/s, the force is sufficient to accelerate the moving box against the
// dynamic friction.
//
// After performing the initial simulation up to `sim_duration`, the example
// will promptly begin infinitely looping playback in wall clock time.

// Simulation parameters.
DEFINE_int32(brick_count, 2, "The number of falling bricks");
DEFINE_bool(random_orientation, true, 
    "If true, randomizes the initial orientations of the bricks");
DEFINE_double(delta_z, 2, "Vertical distance between brick centers");
DEFINE_double(timestep, 1e-4, "The simulator time step");
DEFINE_double(stiffness, 100000, "The contact model's stiffness");
DEFINE_double(us, 0.9, "The static coefficient of friction");
DEFINE_double(ud, 0.5, "The dynamic coefficient of friction");
DEFINE_double(v_tol, 0.01,
              "The maximum slipping speed allowed during stiction");
DEFINE_double(dissipation, 1.0, "The contact model's dissipation");
DEFINE_double(sim_duration, 3, "The simulation duration");
DEFINE_bool(playback, true,
            "If true, enters looping playback after sim finished");

// Simple scenario of two blocks being pushed across a plane.  The first block
// has zero initial velocity.  The second has a small initial velocity in the
// pushing direction.
int main() {
  std::cout << "Parameters:\n";
  std::cout << "\tNumber of bricks: " << FLAGS_brick_count << "\n";
  std::cout << "\tRandomize orientation: " << FLAGS_random_orientation << "\n";
  std::cout << "\tVertical spacing: " << FLAGS_delta_z << "\n";
  std::cout << "\tTime step:        " << FLAGS_timestep << "\n";
  std::cout << "\tStiffness:        " << FLAGS_stiffness << "\n";
  std::cout << "\tStatic friction:  " << FLAGS_us << "\n";
  std::cout << "\tDynamic friction: " << FLAGS_ud << "\n";
  std::cout << "\tSlip Threshold:   " << FLAGS_v_tol << "\n";
  std::cout << "\tDissipation:      " << FLAGS_dissipation << "\n";

  DiagramBuilder<double> builder;

  // Create RigidBodyTree.
  auto tree_ptr = make_unique<RigidBodyTree<double>>();
  for (int i = 0; i < FLAGS_brick_count; ++i) {
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        drake::GetDrakePath() + "/examples/contact_model/brick.urdf",
        kQuaternion, nullptr /* weld to frame */, tree_ptr.get());
  }
  multibody::AddFlatTerrainToWorld(tree_ptr.get(), 100., 10.);

  // Instantiate a RigidBodyPlant from the RigidBodyTree.
  auto& plant = *builder.AddSystem<RigidBodyPlant<double>>(move(tree_ptr));
  plant.set_name("plant");
  // Contact parameters set arbitrarily.
  plant.set_normal_contact_parameters(FLAGS_stiffness, FLAGS_dissipation);
  plant.set_friction_contact_parameters(FLAGS_us, FLAGS_ud, FLAGS_v_tol);
  const auto& tree = plant.get_rigid_body_tree();

  // LCM communication.
  DrakeLcm lcm;

  // Visualizer.
  const auto visualizer_publisher =
      builder.template AddSystem<DrakeVisualizer>(tree, &lcm, true);
  visualizer_publisher->set_name("visualizer_publisher");

  // Raw state vector to visualizer.
  builder.Connect(plant.state_output_port(),
                  visualizer_publisher->get_input_port(0));

  auto diagram = builder.Build();

  // Create simulator.
  auto simulator = std::make_unique<Simulator<double>>(*diagram);
  auto context = simulator->get_mutable_context();
  simulator->reset_integrator<RungeKutta2Integrator<double>>(*diagram,
                                                             FLAGS_timestep,
                                                             context);
  // Set initial state.
  auto plant_context = diagram->GetMutableSubsystemContext(context, &plant);
  // 1 floating quat joint = |xyz|, |q|, |w|, |xyzdot| = 3 + 4 + 3 + 3.
  const int kStateSize = 13 * FLAGS_brick_count;
  DRAKE_DEMAND(plant_context->get_continuous_state_vector().size() ==
               kStateSize);
  VectorX<double> initial_state(kStateSize);

  std::default_random_engine generator;
  for (int i = 0; i < FLAGS_brick_count; ++i) {
  // Place each brick 1.5 m above the previous one
    initial_state.segment<3>(i*7) << 0, 0, FLAGS_delta_z * (i + 0.5);
    if (FLAGS_random_orientation) {
      initial_state.segment<4>(i*7+3) = drake::math::UniformlyRandomQuat(generator);
    } else {
      initial_state.segment<4>(i*7+3) << 1, 0, 0, 0;
    }
    // Zero initial velocities
    initial_state.segment<6>(FLAGS_brick_count*7 + i*6) << 0, 0, 0, 0, 0, 0;
  }


  plant.set_state_vector(plant_context, initial_state);

  simulator->StepTo(FLAGS_sim_duration);

  while (FLAGS_playback) visualizer_publisher->ReplayCachedSimulation();

  return 0;
}
}  // namespace systems
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::systems::main();
}
