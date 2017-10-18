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
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/pick_and_place_configuration.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/pick_and_place_configuration_parsing.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_world/iiwa_wsg_diagram_factory.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/lcmtypes/drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmtypes/drake/lcmt_iiwa_command.hpp"
#include "drake/manipulation/planner/robot_plan_interpolator.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_controller.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"
#include "drake/multibody/rigid_body_plant/create_load_robot_message.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_vector_source.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");
DEFINE_int32(num_iiwas, 2, "Number of IIWA robots to place in the scene.");
DEFINE_string(target_1, "big_yellow_robot", "Name of the target for arm 1.");
DEFINE_string(target_2, "big_blue_robot", "Name of the target for arm 2.");
DEFINE_double(orientation_1, 0, "Yaw angle of the first target.");
DEFINE_double(orientation_2, M_PI_2, "Yaw angle of the second target.");
DEFINE_int32(start_position_1, 1, "Position index to start from");
DEFINE_int32(start_position_2, 3, "Position index to start from");
DEFINE_double(dt, 5e-4, "Integration step size");
DEFINE_double(realtime_rate, 1.0, "Rate at which to run the simulation, "
    "relative to realtime");
DEFINE_bool(quick, false, "Run only a brief simulation and return success "
    "without executing the entire task");
DEFINE_string(configuration_file,
              "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/"
              "configuration/default.pick_and_place_configuration",
              "Path to the configuration file.");

using robotlocomotion::robot_plan_t;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {
namespace {
using manipulation::schunk_wsg::SchunkWsgController;
using manipulation::schunk_wsg::SchunkWsgStatusSender;
using systems::RigidBodyPlant;
using systems::RungeKutta2Integrator;
using systems::Simulator;
using manipulation::util::ModelInstanceInfo;
using manipulation::planner::RobotPlanInterpolator;
using manipulation::util::WorldSimTreeBuilder;
using multibody::CreateLoadRobotMessage;
using optitrack::optitrack_frame_t;

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
    const SimulatedPlantConfiguration& configuration,
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
  tree_builder->StoreModel(
      "wsg",
      "drake/manipulation/models/wsg_50_description"
      "/sdf/schunk_wsg_50_ball_contact.sdf");
  for (int i = 0; i < static_cast<int>(configuration.object_models.size()); ++i) {
    tree_builder->StoreModel("object_" + std::to_string(i), configuration.object_models[i]);
  }
  for (int i = 0; i < static_cast<int>(configuration.table_models.size()); ++i) {
    tree_builder->StoreModel("table_" + std::to_string(i), configuration.table_models[i]);
  }

  // The tables that the objects sit on.
  for (int i = 0; i < static_cast<int>(configuration.table_poses.size()); ++i) {
    int table_id = tree_builder->AddFixedModelInstance(
        "table_" + std::to_string(i), configuration.table_poses[i].translation(),
        drake::math::rotmat2rpy(configuration.table_poses[i].linear()));
    table_instances->push_back(
        tree_builder->get_model_info_for_instance(table_id));
  }
  tree_builder->AddGround();

  for (const auto& robot_base_pose : configuration.robot_base_poses) {
    // Add the arm.
    int robot_base_id = tree_builder->AddFixedModelInstance(
        "iiwa", robot_base_pose.translation(),
        drake::math::rotmat2rpy(robot_base_pose.linear()));
    iiwa_instances->push_back(
        tree_builder->get_model_info_for_instance(robot_base_id));
    // Add the gripper.
    auto frame_ee = tree_builder->tree().findFrame("iiwa_frame_ee", iiwa_instances->back().instance_id);
    auto wsg_frame = frame_ee->Clone(frame_ee->get_mutable_rigid_body());
    wsg_frame->get_mutable_transform_to_body()->rotate(
        Eigen::AngleAxisd(-0.39269908, Eigen::Vector3d::UnitY()));
    wsg_frame->get_mutable_transform_to_body()->translate(0.04*Eigen::Vector3d::UnitY());
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

  for (int i = 0; i < static_cast<int>(configuration.object_poses.size()); ++i) {
    int object_id = tree_builder->AddFloatingModelInstance(
        "object_" + std::to_string(i), configuration.object_poses[i].translation(),
        drake::math::rotmat2rpy(configuration.object_poses[i].linear()));
    object_instances->push_back(
        tree_builder->get_model_info_for_instance(object_id));
  }

  return std::make_unique<systems::RigidBodyPlant<double>>(
      tree_builder->Build());
}

typedef std::tuple<const RigidBody<double>*, Isometry3<double>, int> OptitrackBodyInfo;

class MockOptitrackSystem : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockOptitrackSystem);

  MockOptitrackSystem(
      const Isometry3<double>& X_WO,
      const RigidBodyTree<double>& tree,
      const std::vector<OptitrackBodyInfo> body_info_vector)
      : X_OW_(X_WO.inverse()), tree_(tree), body_info_vector_(body_info_vector) {
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
      const Isometry3<double> X_WF = tree_.CalcFramePoseInWorldFrame(cache, body, X_BF);
      const Isometry3<double> X_OF{X_OW_*X_WF};
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

int DoMain(void) {
  // Parse configuration file
  const SimulatedPlantConfiguration plant_configuration =
      ParseSimulatedPlantConfigurationOrThrow(FLAGS_configuration_file);
  const OptitrackConfiguration optitrack_configuration =
      ParseOptitrackConfigurationOrThrow(FLAGS_configuration_file);

  lcm::DrakeLcm lcm;
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

  auto plant = builder.AddSystem<IiwaAndWsgPlantWithStateEstimator<double>>(
      std::move(model_ptr), iiwa_instances, wsg_instances, box_instances);
  plant->set_name("plant");

  auto contact_viz =
      builder.AddSystem<systems::ContactResultsToLcmSystem<double>>(
          plant->get_tree());
  auto contact_results_publisher = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", &lcm));
  // Contact results to lcm msg.
  builder.Connect(plant->get_output_port_contact_results(),
                  contact_viz->get_input_port(0));
  builder.Connect(contact_viz->get_output_port(0),
                  contact_results_publisher->get_input_port(0));
  contact_results_publisher->set_publish_period(kIiwaLcmStatusPeriod);

  auto drake_visualizer = builder.AddSystem<systems::DrakeVisualizer>(
      plant->get_plant().get_rigid_body_tree(), &lcm);
  drake_visualizer->set_publish_period(kIiwaLcmStatusPeriod);

  builder.Connect(plant->get_output_port_plant_state(),
                  drake_visualizer->get_input_port(0));

  // Connect to "simulated" optitrack
  std::vector<OptitrackBodyInfo> mocap_info;
  const int kNumRobotBases(
      optitrack_configuration.robot_base_optitrack_ids.size());
  for (int i = 0; i < kNumRobotBases; ++i) {
    mocap_info.push_back(std::make_tuple(
        plant->get_tree()
            .FindModelInstanceBodies(iiwa_instances[i].instance_id)
            .front(),
        Isometry3<double>::Identity(),
        optitrack_configuration.robot_base_optitrack_ids[i]));
  }
  const int kNumObjects(optitrack_configuration.object_optitrack_ids.size());
  for (int i = 0; i < kNumObjects; ++i) {
    mocap_info.push_back(std::make_tuple(
        plant->get_tree()
            .FindModelInstanceBodies(box_instances[i].instance_id)
            .front(),
        Isometry3<double>::Identity(),
        optitrack_configuration.object_optitrack_ids[i]));
  }
  const int kNumTables(optitrack_configuration.table_optitrack_ids.size());
  for (int i = 0; i < kNumTables; ++i) {
    mocap_info.push_back(std::make_tuple(
        plant->get_tree()
            .FindModelInstanceBodies(table_instances.at(i).instance_id)
            .front(),
        Isometry3<double>::Identity(),
        optitrack_configuration.table_optitrack_ids[i]));
  }

  Eigen::Isometry3d X_WO = Eigen::Isometry3d::Identity();
  Eigen::Matrix3d rot_mat;
  rot_mat.col(0) = Eigen::Vector3d::UnitY();
  rot_mat.col(1) = Eigen::Vector3d::UnitZ();
  rot_mat.col(2) = Eigen::Vector3d::UnitX();
  X_WO.linear() = rot_mat;
  auto optitrack = builder.AddSystem<MockOptitrackSystem>(X_WO, plant->get_tree(), mocap_info);
  auto optitrack_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<optitrack_frame_t>(
          "OPTITRACK_FRAMES", &lcm));
  optitrack_pub->set_publish_period(kIiwaLcmStatusPeriod);
  builder.Connect(plant->get_output_port_plant_state(),
                  optitrack->get_input_port(0));
  builder.Connect(optitrack->get_output_port(0),
                  optitrack_pub->get_input_port(0));

  std::vector<IiwaCommandReceiver*> iiwa_command_receivers;
  for (int i = 0; i < FLAGS_num_iiwas; ++i) {
    const std::string suffix{"_" + std::to_string(i)};
    auto iiwa_command_sub = builder.AddSystem(
        systems::lcm::LcmSubscriberSystem::Make<lcmt_iiwa_command>(
            "IIWA_COMMAND" + suffix, &lcm));
    iiwa_command_sub->set_name("iiwa_command_subscriber" + suffix);
    auto iiwa_command_receiver = builder.AddSystem<IiwaCommandReceiver>(7);
    iiwa_command_receivers.push_back(iiwa_command_receiver);
    iiwa_command_receiver->set_name("iiwa_command_receiver" + suffix);

    builder.Connect(iiwa_command_sub->get_output_port(0),
                    iiwa_command_receiver->get_input_port(0));
    builder.Connect(iiwa_command_receiver->get_output_port(0),
                    plant->get_input_port_iiwa_state_command(i));

    auto iiwa_status_pub = builder.AddSystem(
        systems::lcm::LcmPublisherSystem::Make<lcmt_iiwa_status>(
            "IIWA_STATUS" + suffix, &lcm));
    iiwa_status_pub->set_name("iiwa_status_publisher" + suffix);
    iiwa_status_pub->set_publish_period(kIiwaLcmStatusPeriod);
    auto iiwa_status_sender = builder.AddSystem<IiwaStatusSender>(7);
    iiwa_status_sender->set_name("iiwa_status_sender" + suffix);

    builder.Connect(plant->get_output_port_iiwa_state(i),
                    iiwa_status_sender->get_state_input_port());
    builder.Connect(iiwa_command_receiver->get_output_port(0),
                    iiwa_status_sender->get_command_input_port());
    builder.Connect(iiwa_status_sender->get_output_port(0),
                    iiwa_status_pub->get_input_port(0));

    auto wsg_controller = builder.AddSystem<SchunkWsgController>();
    wsg_controller->set_name("wsg_controller" + suffix);
    builder.Connect(plant->get_output_port_wsg_state(i),
                    wsg_controller->get_state_input_port());
    builder.Connect(wsg_controller->get_output_port(0),
                    plant->get_input_port_wsg_command(i));

    auto wsg_status_sender = builder.AddSystem<SchunkWsgStatusSender>(
        plant->get_output_port_wsg_state(i).size(), 0, 0);
    wsg_status_sender->set_name("wsg_status_sender" + suffix);
    builder.Connect(plant->get_output_port_wsg_state(i),
                    wsg_status_sender->get_input_port(0));

    auto object_state_pub = builder.AddSystem(
        systems::lcm::LcmPublisherSystem::Make<bot_core::robot_state_t>(
            "OBJECT_STATE_EST" + suffix, &lcm));
    builder.Connect(plant->get_output_port_box_robot_state_msg(i),
                    object_state_pub->get_input_port(0));
    object_state_pub->set_publish_period(kIiwaLcmStatusPeriod);

    auto wsg_status_pub = builder.AddSystem(
        systems::lcm::LcmPublisherSystem::Make<lcmt_schunk_wsg_status>(
            "SCHUNK_WSG_STATUS" + suffix, &lcm));
    builder.Connect(wsg_status_sender->get_output_port(0),
                    wsg_status_pub->get_input_port(0));
    wsg_status_pub->set_publish_period(kIiwaLcmStatusPeriod);

    auto iiwa_robot_state_pub = builder.AddSystem(
        systems::lcm::LcmPublisherSystem::Make<bot_core::robot_state_t>(
            "EST_ROBOT_STATE" + suffix, &lcm));
    builder.Connect(plant->get_output_port_iiwa_robot_state_msg(i),
                    iiwa_robot_state_pub->get_input_port(0));
    iiwa_robot_state_pub->set_publish_period(kIiwaLcmStatusPeriod);

    auto wsg_command_sub = builder.AddSystem(
        systems::lcm::LcmSubscriberSystem::Make<lcmt_schunk_wsg_command>(
            "SCHUNK_WSG_COMMAND" + suffix, &lcm));
    builder.Connect(wsg_command_sub->get_output_port(0),
                    wsg_controller->get_command_input_port());
  }

  auto sys = builder.Build();
  Simulator<double> simulator(*sys);
  simulator.set_target_realtime_rate(FLAGS_realtime_rate);
  simulator.reset_integrator<RungeKutta2Integrator<double>>(*sys,
      FLAGS_dt, simulator.get_mutable_context());
  simulator.get_mutable_integrator()->set_maximum_step_size(FLAGS_dt);
  simulator.get_mutable_integrator()->set_fixed_step_mode(true);

  lcm.StartReceiveThread();
  simulator.set_publish_every_time_step(false);
  simulator.Initialize();

  for (auto& iiwa_command_receiver : iiwa_command_receivers) {
    iiwa_command_receiver->set_initial_position(
        &sys->GetMutableSubsystemContext(*iiwa_command_receiver,
                                         simulator.get_mutable_context()),
        VectorX<double>::Zero(7));
  }

  // Step the simulator in some small increment.  Between steps, check
  // to see if the state machine thinks we're done, and if so that the
  // object is near the target.
  simulator.StepTo(FLAGS_simulation_sec);

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
