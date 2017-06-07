#include <string>

#include <gflags/gflags.h>

#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_world/world_sim_tree_builder.h"
#include "drake/examples/kuka_iiwa_arm/sim_diagram_builder.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/primitives/trajectory_source.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

using drake::systems::RungeKutta2Integrator;
using drake::systems::RigidBodyPlant;
using drake::systems::ImplicitEulerIntegrator;

// Simulation parameters.
DEFINE_double(timestep, 1e-4, "The simulator time step");
DEFINE_double(sim_duration, 3, "The simulation duration");
DEFINE_bool(playback, true,
            "If true, enters looping playback after sim finished");
DEFINE_string(collision_type, "polytope",
              "Type of simulation, valid values are "
              "'polytope','mesh'");
DEFINE_double(stiffness, 100000, "The contact model's stiffness");
DEFINE_double(dissipation, 1.0, "The contact model's dissipation");
DEFINE_double(us, 0.9, "The static coefficient of friction");
DEFINE_double(ud, 0.5, "The dynamic coefficient of friction");
DEFINE_double(v_tol, 0.01,
              "The maximum slipping speed allowed during stiction");
DEFINE_bool(use_implicit, true, "If true, uses imlicit Euler integration");
DEFINE_double(accuracy, 1e-5,
              "Requested simulation accuracy (ignored for time stepping");

enum CollisionType {
  kPolytope = 0,
  kMesh = 1,
};

std::unique_ptr<RigidBodyTree<double>> build_tree(
    std::vector<ModelInstanceInfo<double>>* iiwa,
    ModelInstanceInfo<double>* box,
    CollisionType collision_type) {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world.
  std::string iiwa_urdf{"/manipulation/models/iiwa_description/urdf/"};
  switch (collision_type) {
    case CollisionType::kPolytope: {
      iiwa_urdf += "iiwa14_polytope_collision.urdf";
    } break;
    case CollisionType::kMesh: {
      iiwa_urdf += "iiwa14_mesh_collision.urdf";
    } break;
    default:
      DRAKE_ABORT_MSG("Bad collision type");
      break;
  }
  tree_builder->StoreModel("iiwa", iiwa_urdf);

  tree_builder->StoreModel(
      "box",
      "/examples/kuka_iiwa_arm/models/objects/"
          "heavy_box.urdf");

  iiwa->clear();

  // Left arm.
  int id = tree_builder->AddFixedModelInstance(
      "iiwa", Vector3<double>(0, 0, 0));
  iiwa->push_back(tree_builder->get_model_info_for_instance(id));

  // Right arm.
  id = tree_builder->AddFixedModelInstance(
      "iiwa", Vector3<double>(0, -0.8, 0));
  iiwa->push_back(tree_builder->get_model_info_for_instance(id));

  // Box location, box sides is 0.2 0.2 0.2
  id = tree_builder->AddFloatingModelInstance("box",
      Vector3<double>(0.6, -0.4, 0.26));
  *box = tree_builder->get_model_info_for_instance(id);

  auto tree = tree_builder->Build();
  multibody::AddFlatTerrainToWorld(tree.get(), 100., 10.);

  // Make a stand for the box.
  Isometry3<double> stand(Isometry3<double>::Identity());
  stand.translation() << 0.6, -0.4, -0.1;
  const Vector3<double> stand_sides(0.3, 0.3, 0.3);
  multibody::AddBoxToWorld(stand, stand_sides, "stand", tree.get());

  return tree;
}

int main() {
  std::cout << "Parameters:\n";
  std::cout << "\tTime step:        " << FLAGS_timestep << "\n";
  std::cout << "\tuse_implicit:     " << FLAGS_use_implicit << "\n";
  std::cout << "\taccuracy:         " << FLAGS_accuracy << "\n";
  std::cout << "\tSim duration:     " << FLAGS_sim_duration << "\n";
  std::cout << "\tCollision type:   " << FLAGS_collision_type << "\n";
  std::cout << "\tStiffness:        " << FLAGS_stiffness << "\n";
  std::cout << "\tDissipation:      " << FLAGS_dissipation << "\n";
  std::cout << "\tStatic friction:  " << FLAGS_us << "\n";
  std::cout << "\tDynamic friction: " << FLAGS_ud << "\n";
  std::cout << "\tSlip Threshold:   " << FLAGS_v_tol << "\n";

  drake::lcm::DrakeLcm lcm;
  std::vector<ModelInstanceInfo<double>> iiwa_info;
  ModelInstanceInfo<double> box_info;
  SimDiagramBuilder<double> builder;
  systems::DiagramBuilder<double>* diagram_builder =
      builder.get_mutable_builder();

  // Create the iiwas and add them to the diagram
  RigidBodyPlant<double>* plant;
  if (FLAGS_collision_type == "polytope") {
    plant = builder.AddPlant(build_tree(&iiwa_info, &box_info, CollisionType::kPolytope));
  } else if (FLAGS_collision_type == "mesh") {
    plant = builder.AddPlant(build_tree(&iiwa_info, &box_info, CollisionType::kMesh));
  } else {
    std::cerr << "Invalid collision type '" << FLAGS_collision_type
              << "'; note that types are case sensitive." << std::endl;
    return -1;
  }

  // Contact parameters set arbitrarily.
  plant->set_normal_contact_parameters(FLAGS_stiffness, FLAGS_dissipation);
  plant->set_friction_contact_parameters(FLAGS_us, FLAGS_ud, FLAGS_v_tol);

  const auto visualizer_publisher = builder.AddVisualizer(&lcm, true);

  std::vector<systems::TrajectorySource<double>*> iiwa_traj_src(iiwa_info.size());

  // Left arm traj.
  {
    std::vector<double> times = {0, 2, 4};
    std::vector<MatrixX<double>> knots(times.size(),
                                       MatrixX<double>::Zero(7, 1));
    std::vector<MatrixX<double>> knotsd(times.size(),
                                       MatrixX<double>::Zero(7, 1));
    knots[1] << -21, 102, 90, -24, -10, -65, 0;
    knots[2] << -41, 102, 90, -44, -10, -65, 0;
    for (auto& knot : knots) {
      knot = knot / 180. * M_PI;
    }

    PiecewisePolynomial<double> poly = PiecewisePolynomial<double>::Cubic(
        times, knots, knotsd);

    // Adds a trajectory source for desired state and accelerations.
    iiwa_traj_src[0] =
        diagram_builder->template AddSystem<systems::TrajectorySource<double>>(
            PiecewisePolynomialTrajectory(poly), 1);
  }

  // Right arm traj.
  {
    std::vector<double> times = {0, 2, 4};
    std::vector<MatrixX<double>> knots(times.size(),
                                       MatrixX<double>::Zero(7, 1));
    std::vector<MatrixX<double>> knotsd(times.size(),
                                       MatrixX<double>::Zero(7, 1));
    knots[1] << 21, 102, 90, 24, 10, 65, 0;
    knots[2] << 41, 102, 90, 44, 10, 65, 0;
    for (auto& knot : knots) {
      knot = knot / 180. * M_PI;
    }
    PiecewisePolynomial<double> poly = PiecewisePolynomial<double>::Cubic(
        times, knots, knotsd);

    // Adds a trajectory source for desired state and accelerations.
    iiwa_traj_src[1] =
        diagram_builder->template AddSystem<systems::TrajectorySource<double>>(
            PiecewisePolynomialTrajectory(poly), 1);
  }

  // Adds controllers for all the iiwa arms.
  VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;
  SetPositionControlledIiwaGains(&iiwa_kp, &iiwa_ki, &iiwa_kd);
  int ctr = 0;
  for (const auto& info : iiwa_info) {
    auto controller =
        builder.AddController<systems::InverseDynamicsController<double>>(
            info.instance_id, info.model_path, info.world_offset, iiwa_kp,
            iiwa_ki, iiwa_kd, false /* no feedforward acceleration */);

    controller->set_name("controller" + std::to_string(ctr));

    diagram_builder->Connect(iiwa_traj_src[ctr]->get_output_port(),
                             controller->get_input_port_desired_state());
    ctr++;
  }

  // Simulates.
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);
  auto context = simulator.get_mutable_context();
  if (FLAGS_use_implicit) {
    simulator.reset_integrator<ImplicitEulerIntegrator<double>>(*diagram,
                                                               context);
    simulator.get_mutable_integrator()->set_maximum_step_size(FLAGS_timestep);
    simulator.get_mutable_integrator()->set_target_accuracy(FLAGS_accuracy);
  } else {
    //simulator.reset_integrator<RungeKutta2Integrator<double>>(*diagram,
                                                               //FLAGS_timestep,
                                                               //context);
  }
  simulator.Initialize();
  simulator.set_target_realtime_rate(1.0);
  simulator.StepTo(FLAGS_sim_duration);

  while (FLAGS_playback) visualizer_publisher->ReplayCachedSimulation();
  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kuka_iiwa_arm::main();
}
