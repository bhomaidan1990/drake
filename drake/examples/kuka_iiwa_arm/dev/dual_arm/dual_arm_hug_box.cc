#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_world/world_sim_tree_builder.h"
#include "drake/examples/kuka_iiwa_arm/sim_diagram_builder.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/primitives/trajectory_source.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

std::unique_ptr<RigidBodyTree<double>> build_tree(
    std::vector<ModelInstanceInfo<double>>* iiwa,
    ModelInstanceInfo<double>* box) {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world.
  tree_builder->StoreModel(
      "iiwa",
      "/manipulation/models/iiwa_description/urdf/"
          "iiwa14_polytope_collision.urdf");

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

void main() {
  drake::lcm::DrakeLcm lcm;
  std::vector<ModelInstanceInfo<double>> iiwa_info;
  ModelInstanceInfo<double> box_info;
  SimDiagramBuilder<double> builder;
  systems::DiagramBuilder<double>* diagram_builder =
      builder.get_mutable_builder();

  builder.AddPlant(build_tree(&iiwa_info, &box_info));
  builder.AddVisualizer(&lcm);

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
  simulator.Initialize();
  simulator.set_target_realtime_rate(1.0);
  simulator.StepTo(10000000);
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main() {
  drake::examples::kuka_iiwa_arm::main();

  return 0;
}
