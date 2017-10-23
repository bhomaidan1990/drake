#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/pick_and_place_plant.h"

#include "optitrack/optitrack_frame_t.hpp"

#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_controller.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

using manipulation::schunk_wsg::SchunkWsgController;
using manipulation::schunk_wsg::SchunkWsgStatusSender;
using manipulation::util::WorldSimTreeBuilder;
using manipulation::util::ModelInstanceInfo;
using optitrack::optitrack_frame_t;
using systems::ConstantVectorSource;

namespace {

const char kIiwaUrdf[] =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";

// The `z` coordinate of the top of the table in the world frame.
// The quantity 0.736 is the `z` coordinate of the frame associated with the
// 'surface' collision element in the SDF. This element uses a box of height
// 0.057m thus giving the surface height (`z`) in world coordinates as
// 0.736 + 0.057 / 2.
const double kTableTopZInWorld = 0.736 + 0.057 / 2;

std::unique_ptr<systems::RigidBodyPlant<double>> BuildCombinedPlant(
    const pick_and_place::SimulatedPlantConfiguration& configuration,
    std::vector<ModelInstanceInfo<double>>* iiwa_instances,
    std::vector<ModelInstanceInfo<double>>* wsg_instances,
    std::vector<ModelInstanceInfo<double>>* object_instances,
    std::vector<ModelInstanceInfo<double>>* table_instances) {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world.
  tree_builder->StoreModel("iiwa", kIiwaUrdf);
  tree_builder->StoreModel("table",
                           "drake/examples/kuka_iiwa_arm/models/table/"
                           "extra_heavy_duty_table_surface_only_collision.sdf");
  tree_builder->StoreModel("wsg",
                           "drake/manipulation/models/wsg_50_description"
                           "/sdf/schunk_wsg_50_ball_contact.sdf");
  for (int i = 0; i < static_cast<int>(configuration.object_models.size());
       ++i) {
    tree_builder->StoreModel("object_" + std::to_string(i),
                             configuration.object_models[i]);
  }
  for (int i = 0; i < static_cast<int>(configuration.table_models.size());
       ++i) {
    tree_builder->StoreModel("table_" + std::to_string(i),
                             configuration.table_models[i]);
  }

  // The tables that the objects sit on.
  for (int i = 0; i < static_cast<int>(configuration.table_poses.size()); ++i) {
    int table_id = tree_builder->AddFixedModelInstance(
        "table_" + std::to_string(i),
        configuration.table_poses[i].translation(),
        drake::math::rotmat2rpy(configuration.table_poses[i].linear()));
    table_instances->push_back(
        tree_builder->get_model_info_for_instance(table_id));
  }
  tree_builder->AddGround();

  for (const auto& robot_base_pose : configuration.robot_poses) {
    // Add the arm.
    int robot_base_id = tree_builder->AddFixedModelInstance(
        "iiwa", robot_base_pose.translation(),
        drake::math::rotmat2rpy(robot_base_pose.linear()));
    iiwa_instances->push_back(
        tree_builder->get_model_info_for_instance(robot_base_id));
    // Add the gripper.
    auto frame_ee = tree_builder->tree().findFrame(
        "iiwa_frame_ee", iiwa_instances->back().instance_id);
    auto wsg_frame = frame_ee->Clone(frame_ee->get_mutable_rigid_body());
    wsg_frame->get_mutable_transform_to_body()->rotate(
        Eigen::AngleAxisd(-0.39269908, Eigen::Vector3d::UnitY()));
    wsg_frame->get_mutable_transform_to_body()->translate(
        0.04 * Eigen::Vector3d::UnitY());
    int wsg_id = tree_builder->AddModelInstanceToFrame(
        "wsg", wsg_frame, drake::multibody::joints::kFixed);
    wsg_instances->push_back(tree_builder->get_model_info_for_instance(wsg_id));
    // Add the table that the arm sits on.
    const Isometry3<double> X_WT{
        robot_base_pose *
        Isometry3<double>::TranslationType(0.0, 0.0, -kTableTopZInWorld)};
    tree_builder->AddFixedModelInstance("table", X_WT.translation(),
                                        drake::math::rotmat2rpy(X_WT.linear()));
  }

  for (int i = 0; i < static_cast<int>(configuration.object_poses.size());
       ++i) {
    int object_id = tree_builder->AddFloatingModelInstance(
        "object_" + std::to_string(i),
        configuration.object_poses[i].translation(),
        drake::math::rotmat2rpy(configuration.object_poses[i].linear()));
    object_instances->push_back(
        tree_builder->get_model_info_for_instance(object_id));
  }

  return std::make_unique<systems::RigidBodyPlant<double>>(
      tree_builder->Build());
}

typedef std::tuple<const RigidBody<double>*, Isometry3<double>, int>
    OptitrackBodyInfo;

class MockOptitrackSystem : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockOptitrackSystem);

  MockOptitrackSystem(const Isometry3<double>& X_WO,
                      const RigidBodyTree<double>& tree,
                      const std::vector<OptitrackBodyInfo> body_info_vector)
      : X_OW_(X_WO.inverse()),
        tree_(tree),
        body_info_vector_(body_info_vector) {
    this->DeclareInputPort(
        systems::kVectorValued,
        tree_.get_num_positions() + tree_.get_num_velocities());
    DeclareAbstractOutputPort(optitrack_frame_t(),
                              &MockOptitrackSystem::OutputOptitrackFrame);
  }

 private:
  void DoCalcUnrestrictedUpdate(
      const systems::Context<double>& context,
      const std::vector<const systems::UnrestrictedUpdateEvent<double>*>& event,
      systems::State<double>* state) const {
    // Extract internal state
    auto internal_state = state->get_mutable_abstract_state<VectorX<double>>(0);
    // Update world state from inputs.
    const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
    DRAKE_ASSERT(input != nullptr);
    internal_state = input->GetValue<VectorX<double>>();
  }

  void OutputOptitrackFrame(const systems::Context<double>& context,
                            optitrack_frame_t* output) const {
    const VectorX<double> input = this->EvalEigenVectorInput(context, 0);
    VectorX<double> q = input.head(tree_.get_num_positions());
    KinematicsCache<double> cache = tree_.doKinematics(q);

    output->rigid_bodies.clear();
    output->rigid_bodies.reserve(body_info_vector_.size());
    for (const OptitrackBodyInfo& body_info : body_info_vector_) {
      output->rigid_bodies.emplace_back();
      output->rigid_bodies.back().id = std::get<2>(body_info);
      const RigidBody<double>& body = *std::get<0>(body_info);
      const Isometry3<double>& X_BF = std::get<1>(body_info);
      const Isometry3<double> X_WF =
          tree_.CalcFramePoseInWorldFrame(cache, body, X_BF);
      const Isometry3<double> X_OF{X_OW_ * X_WF};
      const Vector3<double>& r_OF = X_OF.translation();
      const Quaternion<double> quat_OF{X_OF.rotation()};
      for (int i : {0, 1, 2}) {
        output->rigid_bodies.back().xyz[i] = r_OF(i);
      }
      // Convert to optitracks X-Y-Z-W order
      output->rigid_bodies.back().quat[0] = quat_OF.x();
      output->rigid_bodies.back().quat[1] = quat_OF.y();
      output->rigid_bodies.back().quat[2] = quat_OF.z();
      output->rigid_bodies.back().quat[3] = quat_OF.w();
    }
    output->num_rigid_bodies = output->rigid_bodies.size();
  }

  const Isometry3<double> X_OW_;
  const RigidBodyTree<double>& tree_;
  const std::vector<OptitrackBodyInfo> body_info_vector_;
};
}  // namespace

PickAndPlacePlant::PickAndPlacePlant(
    const pick_and_place::SimulatedPlantConfiguration& plant_configuration,
    const pick_and_place::OptitrackConfiguration& optitrack_configuration) {
  systems::DiagramBuilder<double> builder;
  std::vector<ModelInstanceInfo<double>> iiwa_instances, wsg_instances,
      box_instances, table_instances;
  std::unique_ptr<systems::RigidBodyPlant<double>> model_ptr =
      BuildCombinedPlant(plant_configuration, &iiwa_instances, &wsg_instances,
                         &box_instances, &table_instances);
  model_ptr->set_normal_contact_parameters(plant_configuration.stiffness,
                                           plant_configuration.dissipation);
  model_ptr->set_friction_contact_parameters(
      plant_configuration.static_friction_coef,
      plant_configuration.dynamic_friction_coef,
      plant_configuration.v_stiction_tolerance);

  iiwa_and_wsg_plant_ =
      builder.AddSystem<IiwaAndWsgPlantWithStateEstimator<double>>(
          std::move(model_ptr), iiwa_instances, wsg_instances, box_instances);
  iiwa_and_wsg_plant_->set_name("iiwa_and_wsg_plant_");

  // Connect to "simulated" optitrack
  std::vector<OptitrackBodyInfo> mocap_info;
  const int kNumRobotBases(
      optitrack_configuration.robot_base_optitrack_info.size());
  for (int i = 0; i < kNumRobotBases; ++i) {
    mocap_info.push_back(std::make_tuple(
        iiwa_and_wsg_plant_->get_tree()
            .FindModelInstanceBodies(iiwa_instances[i].instance_id)
            .front(),
        Isometry3<double>::Identity(),
        optitrack_configuration.robot_base_optitrack_info[i].id));
  }
  const int kNumObjects(optitrack_configuration.object_optitrack_info.size());
  for (int i = 0; i < kNumObjects; ++i) {
    mocap_info.push_back(std::make_tuple(
        iiwa_and_wsg_plant_->get_tree()
            .FindModelInstanceBodies(box_instances[i].instance_id)
            .front(),
        Isometry3<double>::Identity(),
        optitrack_configuration.object_optitrack_info[i].id));
  }
  const int kNumTables(optitrack_configuration.table_optitrack_info.size());
  for (int i = 0; i < kNumTables; ++i) {
    mocap_info.push_back(std::make_tuple(
        iiwa_and_wsg_plant_->get_tree()
            .FindModelInstanceBodies(table_instances.at(i).instance_id)
            .front(),
        optitrack_configuration.table_optitrack_info.at(i).X_MF,
        optitrack_configuration.table_optitrack_info.at(i).id));
  }

  Eigen::Isometry3d X_WO = Eigen::Isometry3d::Identity();
  Eigen::Matrix3d rot_mat;
  rot_mat.col(0) = Eigen::Vector3d::UnitY();
  rot_mat.col(1) = Eigen::Vector3d::UnitZ();
  rot_mat.col(2) = Eigen::Vector3d::UnitX();
  X_WO.linear() = rot_mat;
  auto optitrack = builder.AddSystem<MockOptitrackSystem>(
      X_WO, iiwa_and_wsg_plant_->get_tree(), mocap_info);
  builder.Connect(iiwa_and_wsg_plant_->get_output_port_plant_state(),
                  optitrack->get_input_port(0));
  // Export Optitrack output port.
  output_port_optitrack_frame_ =
      builder.ExportOutput(optitrack->get_output_port(0));

  // Export contact results output port.
  output_port_contact_results_ = builder.ExportOutput(
      iiwa_and_wsg_plant_->get_output_port_contact_results());

  // Export plant state output port.
  output_port_plant_state_ =
      builder.ExportOutput(iiwa_and_wsg_plant_->get_output_port_plant_state());

  std::vector<IiwaCommandReceiver*> iiwa_command_receivers;
  const int kNumIiwas = plant_configuration.robot_poses.size();
  for (int i = 0; i < kNumIiwas; ++i) {
    const std::string suffix{"_" + std::to_string(i)};
    auto iiwa_command_receiver = builder.AddSystem<IiwaCommandReceiver>(7);
    iiwa_command_receivers.push_back(iiwa_command_receiver);
    iiwa_command_receiver->set_name("iiwa_command_receiver" + suffix);

    // Export iiwa command input port.
    input_port_iiwa_command_.push_back(
        builder.ExportInput(iiwa_command_receiver->get_input_port(0)));
    builder.Connect(iiwa_command_receiver->get_output_port(0),
                    iiwa_and_wsg_plant_->get_input_port_iiwa_state_command(i));

    // lcmt_iiwa_command does not include a reference acceleration, so use a
    // zero constant source for the controller's acceleration input.
    auto zero_feedforward_acceleration =
        builder.AddSystem<ConstantVectorSource<double>>(VectorX<double>::Zero(
            iiwa_and_wsg_plant_->get_input_port_iiwa_acceleration_command()
                .size()));
    zero_feedforward_acceleration->set_name("zero");
    builder.Connect(
        zero_feedforward_acceleration->get_output_port(),
        iiwa_and_wsg_plant_->get_input_port_iiwa_acceleration_command(i));

    auto iiwa_status_sender = builder.AddSystem<IiwaStatusSender>(7);
    iiwa_status_sender->set_name("iiwa_status_sender" + suffix);

    builder.Connect(iiwa_and_wsg_plant_->get_output_port_iiwa_state(i),
                    iiwa_status_sender->get_state_input_port());
    builder.Connect(iiwa_command_receiver->get_output_port(0),
                    iiwa_status_sender->get_command_input_port());

    // Export iiwa status output port.
    output_port_iiwa_status_.push_back(
        builder.ExportOutput(iiwa_status_sender->get_output_port(0)));

    auto wsg_controller = builder.AddSystem<SchunkWsgController>();
    wsg_controller->set_name("wsg_controller" + suffix);
    builder.Connect(iiwa_and_wsg_plant_->get_output_port_wsg_state(i),
                    wsg_controller->get_state_input_port());
    builder.Connect(wsg_controller->get_output_port(0),
                    iiwa_and_wsg_plant_->get_input_port_wsg_command(i));

    auto wsg_status_sender = builder.AddSystem<SchunkWsgStatusSender>(
        iiwa_and_wsg_plant_->get_output_port_wsg_state(i).size(), 0, 0);
    wsg_status_sender->set_name("wsg_status_sender" + suffix);
    builder.Connect(iiwa_and_wsg_plant_->get_output_port_wsg_state(i),
                    wsg_status_sender->get_input_port(0));

    // Export wsg status output port.
    output_port_wsg_status_.push_back(
        builder.ExportOutput(wsg_status_sender->get_output_port(0)));

    // Export iiwa robot_state_t output port.
    output_port_iiwa_robot_state_.push_back(builder.ExportOutput(
        iiwa_and_wsg_plant_->get_output_port_iiwa_robot_state_msg(i)));

    // Export WSG command input port.
    input_port_wsg_command_.push_back(
        builder.ExportInput(wsg_controller->get_command_input_port()));
  }

  // Build the system.
  builder.BuildInto(this);
}

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
