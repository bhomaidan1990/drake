#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

#include "drake/common/find_resource.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/systems/framework/diagram_builder.h"

using drake::geometry::SceneGraph;
using drake::multibody::multibody_plant::MultibodyPlant;

namespace drake {
namespace multibody {
namespace {
template <typename T>
std::unique_ptr<T> ConstructTwoFreeBodiesHelper() {
  auto model = std::make_unique<T>();

  const double mass{1};
  const Eigen::Vector3d p_AoAcm_A(0, 0, 0);
  const RotationalInertia<double> I_AAcm_A{0.001, 0.001, 0.001};
  const SpatialInertia<double> M_AAo_A =
      SpatialInertia<double>::MakeFromCentralInertia(mass, p_AoAcm_A, I_AAcm_A);

  model->AddRigidBody("body1", M_AAo_A);
  model->AddRigidBody("body2", M_AAo_A);

  model->Finalize();

  return model;
}
}  // namespace

namespace internal {
IiwaKinematicConstraintTest::IiwaKinematicConstraintTest()
    : iiwa_autodiff_(benchmarks::kuka_iiwa_robot::MakeKukaIiwaModel<AutoDiffXd>(
          true /* finalized model. */)),
      iiwa_double_(benchmarks::kuka_iiwa_robot::MakeKukaIiwaModel<double>(
          true /* finalized model. */)),
      context_autodiff_(iiwa_autodiff_.CreateDefaultContext()),
      context_double_(iiwa_double_.CreateDefaultContext()) {
  systems::DiagramBuilder<double> builder{};
  plant_ = builder.AddSystem<MultibodyPlant<double>>();
  plant_->RegisterAsSourceForSceneGraph(
      builder.AddSystem<SceneGraph<double>>());
  parsing::AddModelFromSdfFile(
      FindResourceOrThrow("drake/manipulation/models/iiwa_description/sdf/"
                          "iiwa14_no_collision.sdf"),
      "iiwa", plant_);
  plant_->Finalize();

  diagram_ = builder.Build();
  context_ = diagram_->CreateDefaultContext();
}
}

template <typename T>
std::unique_ptr<MultibodyTree<T>> ConstructTwoFreeBodies() {
  return ConstructTwoFreeBodiesHelper<MultibodyTree<T>>();
}

template <typename T>
std::unique_ptr<multibody_plant::MultibodyPlant<T>>
ConstructTwoFreeBodiesPlant() {
  return ConstructTwoFreeBodiesHelper<multibody_plant::MultibodyPlant<T>>();
}

std::unique_ptr<multibody_plant::MultibodyPlant<double>> ConstructIiwaPlant(
    const std::string& iiwa_sdf_name, double time_step) {
  const std::string file_path =
      "drake/manipulation/models/iiwa_description/sdf/" + iiwa_sdf_name;
  auto plant =
      std::make_unique<multibody_plant::MultibodyPlant<double>>(time_step);
  parsing::AddModelFromSdfFile(FindResourceOrThrow(file_path), plant.get());
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("iiwa_link_0"));
  plant->Finalize();
  return plant;
}

Eigen::Vector4d QuaternionToVectorWxyz(const Eigen::Quaterniond& q) {
  return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
}

template std::unique_ptr<MultibodyTree<double>>
ConstructTwoFreeBodies<double>();
template std::unique_ptr<MultibodyTree<AutoDiffXd>>
ConstructTwoFreeBodies<AutoDiffXd>();
template std::unique_ptr<multibody_plant::MultibodyPlant<double>>
ConstructTwoFreeBodiesPlant<double>();
template std::unique_ptr<multibody_plant::MultibodyPlant<AutoDiffXd>>
ConstructTwoFreeBodiesPlant<AutoDiffXd>();
}  // namespace multibody
}  // namespace drake
