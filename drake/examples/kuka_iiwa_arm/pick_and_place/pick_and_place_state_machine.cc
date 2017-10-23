#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_state_machine.h"

#include <random>

#include <spdlog/fmt/ostr.h>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_ik.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {
namespace {

using manipulation::util::WorldSimTreeBuilder;

// Position the gripper 30cm above the object before grasp.
const double kPreGraspHeightOffset = 0.3;

// Finger is 19 cm from end-effector frame.
const double kEndEffectorToMidFingerDepth = 0.19;

PostureInterpolationResult PlanMotionAboveEndPoints(
    const PostureInterpolationRequest& request,
    const RigidBodyTree<double>& original_robot) {
  // Create local references to request member variables
  const VectorX<double>& q_0{request.q_initial};
  const VectorX<double>& q_f{request.q_final};
  const std::vector<double>& times{request.times};
  const double& position_tolerance{request.position_tolerance};

  // Create a local copy of the robot
  std::unique_ptr<RigidBodyTree<double>> robot{original_robot.Clone()};
  int kNumJoints{robot->get_num_positions()};
  const VectorX<int> joint_indices =
      VectorX<int>::LinSpaced(kNumJoints, 0, kNumJoints - 1);
  int end_effector_idx = robot->FindBodyIndex("iiwa_link_ee");
  int world_idx = robot->FindBodyIndex("world");
  Vector3<double> end_effector_points{kEndEffectorToMidFingerDepth, 0, 0};

  // Create vectors to hold the constraint objects
  std::vector<std::unique_ptr<PostureConstraint>> posture_constraints;
  std::vector<std::unique_ptr<WorldPositionInFrameConstraint>> position_in_frame_constraints;
  std::vector<RigidBodyConstraint*> constraint_array;

  const int kNumKnots = times.size();

  // Compute the initial and final end-effector poses
  auto kinematics_cache = robot->CreateKinematicsCache();
  kinematics_cache.initialize(q_0);
  robot->doKinematics(kinematics_cache);
  std::pair<Isometry3<double>, Isometry3<double>> X_WE;
  Isometry3<double> X_LE{Isometry3<double>::Identity()};
  X_LE.translation()[0] = kEndEffectorToMidFingerDepth;
  X_WE.first =
      robot->relativeTransform(kinematics_cache, world_idx, end_effector_idx) *
      X_LE;
  kinematics_cache.initialize(q_f);
  robot->doKinematics(kinematics_cache);
  X_WE.second =
      robot->relativeTransform(kinematics_cache, world_idx, end_effector_idx) *
      X_LE;

  std::pair<Vector3<double>, Vector3<double>> r_WE{X_WE.first.translation(),
                                                   X_WE.second.translation()};

  // Define active times for intermediate and final constraints
  const double& start_time = times.front();
  const double& end_time = times.back();
  const Vector2<double> intermediate_tspan{start_time, end_time};
  const Vector2<double> final_tspan{end_time, end_time};

  // Constrain the configuration at the final knot point to match q_f.
  posture_constraints.emplace_back(
      new PostureConstraint(robot.get(), final_tspan));
  const VectorX<double> q_lb{q_f};
  const VectorX<double> q_ub{q_f};
  posture_constraints.back()->setJointLimits(joint_indices, q_lb, q_ub);
  constraint_array.push_back(posture_constraints.back().get());

  // Construct intermediate constraints for via points
  // We will constrain the z-component of the end-effector position to be above
  // the lower of the two end points at all via points.
  Isometry3<double> X_WL{Isometry3<double>::Identity()}; // World to fLoor
  X_WL.translation() =
      (r_WE.first.z() < r_WE.second.z()) ? r_WE.first : r_WE.second;

  drake::log()->debug("Planning motion from {} to {}, above z = {}",
                      r_WE.first.transpose(), r_WE.second.transpose(),
                      X_WL.translation().z());

  Vector3<double> lb{-std::numeric_limits<double>::infinity(),
                     -std::numeric_limits<double>::infinity(),
                     -position_tolerance};
  Vector3<double> ub{std::numeric_limits<double>::infinity(),
                     std::numeric_limits<double>::infinity(),
                     std::numeric_limits<double>::infinity()};

  position_in_frame_constraints.emplace_back(new WorldPositionInFrameConstraint(
      robot.get(), end_effector_idx, end_effector_points, X_WL.matrix(), lb, ub,
      intermediate_tspan));
  constraint_array.push_back(position_in_frame_constraints.back().get());

  // Set the seed for the first attempt (and the nominal value for all attempts)
  // to be the cubic interpolation between the initial and final
  // configurations.
  VectorX<double> q_dot0{VectorX<double>::Zero(kNumJoints)};
  VectorX<double> q_dotf{VectorX<double>::Zero(kNumJoints)};
  MatrixX<double> q_knots_seed{kNumJoints, kNumKnots};
  PiecewisePolynomial<double> q_seed_traj{PiecewisePolynomial<double>::Cubic(
      {times.front(), times.back()}, {q_0, q_f}, q_dot0, q_dotf)};
  for (int i = 0; i < kNumKnots; ++i) {
    q_knots_seed.col(i) = q_seed_traj.value(times[i]);
  }
  MatrixX<double> q_knots_nom{q_knots_seed};

  // Get the time values into the format required by inverseKinTrajSimple
  VectorX<double> t{kNumKnots};
  for (int i = 0; i < kNumKnots; ++i) t(i) = times[i];

  // Configure ik input and output structs.
  IKoptions ikoptions(robot.get());
  ikoptions.setFixInitialState(true);
  ikoptions.setQa(MatrixX<double>::Identity(robot->get_num_positions(),
                                            robot->get_num_positions()));
  ikoptions.setQv(MatrixX<double>::Identity(robot->get_num_positions(),
                                            robot->get_num_positions()));
  ikoptions.setMajorOptimalityTolerance(1e-4);
  IKResults ik_res;

  // Attempt to solve the ik traj problem multiple times. If falling back to
  // joint-space interpolation is allowed, don't try as hard.
  const int kNumRestarts = 50;
  std::default_random_engine rand_generator{1234};
  for (int i = 0; i < kNumRestarts; ++i) {
    ik_res = inverseKinTrajSimple(robot.get(), t, q_knots_seed, q_knots_nom,
                                   constraint_array, ikoptions);
    if (ik_res.info[0] == 1) {
      break;
    } else {
      VectorX<double> q_mid = robot->getRandomConfiguration(rand_generator);
      q_seed_traj = PiecewisePolynomial<double>::Cubic(
          {times.front(), 0.5 * (times.front() + times.back()), times.back()},
          {q_0, q_mid, q_f}, q_dot0, q_dotf);
      for (int j = 0; j < kNumKnots; ++j) {
        q_knots_seed.col(j) = q_seed_traj.value(times[j]);
      }
    }
  }
  PostureInterpolationResult result;
  result.success = (ik_res.info[0] == 1);
  std::vector<MatrixX<double>> q_sol(kNumKnots);
  if (result.success) {
    for (int i = 0; i < kNumKnots; ++i) {
      q_sol[i] = ik_res.q_sol[i];
    }
    result.q_traj = PiecewisePolynomial<double>::Cubic(times, q_sol, q_dot0, q_dotf);
  } else {
    result.q_traj = PiecewisePolynomial<double>::Cubic(
            {times.front(), times.back()}, {q_0, q_0}, q_dot0, q_dotf);
  }
  return result;
}

// Generates a sequence (@p num_via_points + 1) of key frames s.t. the end
// effector moves in a straight line between @pX_WEndEffector0 and
// @p X_WEndEffector1.
PostureInterpolationResult PlanStraightLineMotion(
    const PostureInterpolationRequest& request,
    const RigidBodyTree<double>& original_robot) {
  // Create local references to request member variables
  const VectorX<double>& q_0{request.q_initial};
  const VectorX<double>& q_f{request.q_final};
  const std::vector<double>& times{request.times};
  const double& position_tolerance{request.position_tolerance};
  const double& orientation_tolerance{request.orientation_tolerance};
  const bool& fall_back_to_joint_space_interpolation{
      request.fall_back_to_joint_space_interpolation};
  const double max_joint_position_change{request.max_joint_position_change};

  const int kNumKnots = times.size();

  drake::log()->debug("Processing interpolation request:");
  drake::log()->debug("\tNumber of knots: {}", kNumKnots);
  drake::log()->debug("\tPosition tolerance: {}", position_tolerance);
  drake::log()->debug("\tOrientation tolerance: {}", orientation_tolerance);
  drake::log()->debug("\tFall back to joint space: {}",
                      fall_back_to_joint_space_interpolation);

  // Create a local copy of the robot
  std::unique_ptr<RigidBodyTree<double>> robot{original_robot.Clone()};
  int kNumJoints{robot->get_num_positions()};
  const VectorX<int> joint_indices =
      VectorX<int>::LinSpaced(kNumJoints, 0, kNumJoints - 1);
  int end_effector_idx = robot->FindBodyIndex("iiwa_link_ee");
  int world_idx = robot->FindBodyIndex("world");
  Vector3<double> end_effector_points{kEndEffectorToMidFingerDepth, 0, 0};

  // Create vectors to hold the constraint objects
  std::vector<std::unique_ptr<PostureConstraint>> posture_constraints;
  std::vector<std::unique_ptr<WorldQuatConstraint>> orientation_constraints;
  std::vector<std::unique_ptr<Point2LineSegDistConstraint>>
      point_to_line_seg_constraints;
  std::vector<std::unique_ptr<WorldGazeDirConstraint>> gaze_dir_constraints;
  std::vector<std::unique_ptr<PostureChangeConstraint>>
      posture_change_constraints;
  std::vector<RigidBodyConstraint*> constraint_array;

  // Compute the initial and final end-effector poses
  auto kinematics_cache = robot->CreateKinematicsCache();
  kinematics_cache.initialize(q_0);
  robot->doKinematics(kinematics_cache);
  std::pair<Isometry3<double>, Isometry3<double>> X_WE;
  Isometry3<double> X_LE{Isometry3<double>::Identity()};
  X_LE.translation()[0] = kEndEffectorToMidFingerDepth;
  X_WE.first =
      robot->relativeTransform(kinematics_cache, world_idx, end_effector_idx) *
      X_LE;
  kinematics_cache.initialize(q_f);
  robot->doKinematics(kinematics_cache);
  X_WE.second =
      robot->relativeTransform(kinematics_cache, world_idx, end_effector_idx) *
      X_LE;

  std::pair<Vector3<double>, Vector3<double>> r_WE{X_WE.first.translation(),
                                                   X_WE.second.translation()};
  std::pair<Quaternion<double>, Quaternion<double>> quat_WE{
      Quaternion<double>(X_WE.first.linear()),
      Quaternion<double>(X_WE.second.linear())};

  drake::log()->debug("Planning straight line from {} {} to {} {}",
                      r_WE.first.transpose(),
                      math::rotmat2rpy(X_WE.first.rotation()).transpose(),
                      r_WE.second.transpose(),
                      math::rotmat2rpy(X_WE.second.rotation()).transpose());

  // Define active times for intermediate and final constraints
  const double& start_time = times.front();
  const double& end_time = times.back();
  const Vector2<double> intermediate_tspan{start_time, end_time};
  const Vector2<double> final_tspan{end_time, end_time};

  // Constrain the configuration at the final knot point to match q_f.
  posture_constraints.emplace_back(
      new PostureConstraint(robot.get(), final_tspan));
  const VectorX<double> q_lb{q_f};
  const VectorX<double> q_ub{q_f};
  posture_constraints.back()->setJointLimits(joint_indices, q_lb, q_ub);
  constraint_array.push_back(posture_constraints.back().get());

  // Construct intermediate constraints for via points
  // We will impose a Point2LineSegDistConstraint and WorldGazeDirConstraint
  // on the via points.
  Eigen::Matrix<double, 3, 2> line_ends_W;
  line_ends_W << r_WE.first, r_WE.second;

  double dist_lb{0.0};
  double dist_ub{position_tolerance};
  point_to_line_seg_constraints.emplace_back(new Point2LineSegDistConstraint(
      robot.get(), end_effector_idx, end_effector_points, world_idx,
      line_ends_W, dist_lb, dist_ub, intermediate_tspan));
  constraint_array.push_back(point_to_line_seg_constraints.back().get());

  // Find axis-angle representation of the rotation from X_WE.first to
  // X_WE.second.
  Isometry3<double> X_second_first = X_WE.second.inverse() * X_WE.first;
  Eigen::AngleAxis<double> aaxis{X_second_first.linear()};
  Vector3<double> axis_E{aaxis.axis()};
  Vector3<double> dir_W{X_WE.first.linear() * axis_E};

  // If the intial and final end-effector orientations are close to each other
  // (to within the orientation tolerance), fix the orientation for all via
  // points. Otherwise, only allow the end-effector to rotate about the axis
  // defining the rotation between the initial and final orientations.
  if (std::abs(aaxis.angle()) < orientation_tolerance) {
    orientation_constraints.emplace_back(new WorldQuatConstraint(
        robot.get(), end_effector_idx,
        Eigen::Vector4d(quat_WE.second.w(), quat_WE.second.x(),
                        quat_WE.second.y(), quat_WE.second.z()),
        orientation_tolerance, intermediate_tspan));
    constraint_array.push_back(orientation_constraints.back().get());
  } else {
    gaze_dir_constraints.emplace_back(new WorldGazeDirConstraint(
        robot.get(), end_effector_idx, axis_E, dir_W,
        orientation_tolerance, intermediate_tspan));
    constraint_array.push_back(gaze_dir_constraints.back().get());
  }

  // Place limits on the change in joint angles between knots
  const VectorX<double> ub_change =
      max_joint_position_change * VectorX<double>::Ones(kNumJoints);
  const VectorX<double> lb_change = -ub_change;
  for (int i = 1; i < kNumKnots; ++i) {
    const Vector2<double> segment_tspan{times[i-1], times[i]};
    posture_change_constraints.emplace_back(new PostureChangeConstraint(
        robot.get(), joint_indices, lb_change, ub_change, segment_tspan));
    constraint_array.push_back(posture_change_constraints.back().get());
  }

  // Set the seed for the first attempt (and the nominal value for all attempts)
  // to be the cubic interpolation between the initial and final
  // configurations.
  VectorX<double> q_dot0{VectorX<double>::Zero(kNumJoints)};
  VectorX<double> q_dotf{VectorX<double>::Zero(kNumJoints)};
  MatrixX<double> q_knots_seed{kNumJoints, kNumKnots};
  PiecewisePolynomial<double> q_seed_traj{PiecewisePolynomial<double>::Cubic(
      {times.front(), times.back()}, {q_0, q_f}, q_dot0, q_dotf)};
  for (int i = 0; i < kNumKnots; ++i) {
    q_knots_seed.col(i) = q_seed_traj.value(times[i]);
  }
  MatrixX<double> q_knots_nom{q_knots_seed};

  // Get the time values into the format required by inverseKinTrajSimple
  VectorX<double> t{kNumKnots};
  for (int i = 0; i < kNumKnots; ++i) t(i) = times[i];

  // Configure ik input and output structs.
  IKoptions ikoptions(robot.get());
  ikoptions.setFixInitialState(true);
  ikoptions.setQa(MatrixX<double>::Identity(robot->get_num_positions(),
                                            robot->get_num_positions()));
  ikoptions.setQv(MatrixX<double>::Identity(robot->get_num_positions(),
                                            robot->get_num_positions()));
  ikoptions.setMajorOptimalityTolerance(1e-6);
  IKResults ik_res;

  // Attempt to solve the ik traj problem multiple times. If falling back to
  // joint-space interpolation is allowed, don't try as hard.
  const int kNumRestarts =
      (fall_back_to_joint_space_interpolation) ? 5 : 50;
  std::default_random_engine rand_generator{1234};
  for (int i = 0; i < kNumRestarts; ++i) {
    ik_res = inverseKinTrajSimple(robot.get(), t, q_knots_seed, q_knots_nom,
                                   constraint_array, ikoptions);
    if (ik_res.info[0] == 1) {
      break;
    } else {
      VectorX<double> q_mid = robot->getRandomConfiguration(rand_generator);
      q_seed_traj = PiecewisePolynomial<double>::Cubic(
          {times.front(), 0.5 * (times.front() + times.back()), times.back()},
          {q_0, q_mid, q_f}, q_dot0, q_dotf);
      for (int j = 0; j < kNumKnots; ++j) {
        q_knots_seed.col(j) = q_seed_traj.value(times[j]);
      }
    }
  }
  PostureInterpolationResult result;
  result.success = (ik_res.info[0] == 1);
  std::vector<MatrixX<double>> q_sol(kNumKnots);
  if (result.success) {
    for (int i = 0; i < kNumKnots; ++i) {
      q_sol[i] = ik_res.q_sol[i];
    }
    result.q_traj = PiecewisePolynomial<double>::Cubic(times, q_sol, q_dot0, q_dotf);
  } else if (fall_back_to_joint_space_interpolation) {
    result = PlanMotionAboveEndPoints(request, original_robot);
  } else {
    result.q_traj = PiecewisePolynomial<double>::Cubic(
            {times.front(), times.back()}, {q_0, q_0}, q_dot0, q_dotf);
  }
  return result;
}

void OpenGripper(const WorldState& env_state, WsgAction* wsg_act,
                 lcmt_schunk_wsg_command* msg) {
  wsg_act->OpenGripper(env_state, msg);
};

void CloseGripper(const WorldState& env_state, WsgAction* wsg_act,
                 lcmt_schunk_wsg_command* msg) {
  wsg_act->CloseGripper(env_state, msg);
};

}  // namespace

std::ostream& operator<<(std::ostream& os, const PickAndPlaceState value) {
  switch (value) {
    case (PickAndPlaceState::kOpenGripper):
      return os << "kOpenGripper";
    case (PickAndPlaceState::kPlan):
      return os << "kPlan";
    case (PickAndPlaceState::kApproachPickPregrasp):
      return os << "kApproachPickPregrasp";
    case (PickAndPlaceState::kApproachPick):
      return os << "kApproachPick";
    case (PickAndPlaceState::kGrasp):
      return os << "kGrasp";
    case (PickAndPlaceState::kLiftFromPick):
      return os << "kLiftFromPick";
    case (PickAndPlaceState::kApproachPlacePregrasp):
      return os << "kApproachPlacePregrasp";
    case (PickAndPlaceState::kApproachPlace):
      return os << "kApproachPlace";
    case (PickAndPlaceState::kPlace):
      return os << "kPlace";
    case (PickAndPlaceState::kLiftFromPlace):
      return os << "kLiftFromPlace";
    case (PickAndPlaceState::kReset):
      return os << "kReset";
    case (PickAndPlaceState::kDone):
      return os << "kDone";
  }
}

bool ComputeInitialAndFinalObjectPoses(
    const WorldState& env_state,
    std::pair<Isometry3<double>, Isometry3<double>>* X_WO_initial_and_final) {
  // W -- planning World frame, coincides with kuka base frame.
  // S -- Sensor world frame
  // O -- Object frame
  // T -- Table frame
  Isometry3<double> X_WS{env_state.get_iiwa_base().inverse()};
  X_WO_initial_and_final->first = X_WS*env_state.get_object_pose();

  drake::log()->debug("r_WO_initial = [{}]",
                      X_WO_initial_and_final->first.translation().transpose());
  drake::log()->debug("R_WO_initial = \n{}",
                      X_WO_initial_and_final->first.linear());
  // Check that the object is oriented correctly
  if (X_WO_initial_and_final->first.linear()(2,2) <
      std::cos(20 * M_PI / 180)) {
    drake::log()->debug(
        "Improper object orientation relative to robot base. Please reset "
        "object and/or check Optitrack markers.");
    return false;
  }

  // Find the destination table
  const std::vector<Isometry3<double>>& table_poses =
      env_state.get_table_poses();
  int destination_table_index = -1;
  const double kMaxReach = 1.1;
  double min_angle = std::numeric_limits<double>::infinity();
  for (int i = 0; i < static_cast<int>(table_poses.size()); ++i) {
    const Isometry3<double> X_WT = X_WS * table_poses[i];
    Vector3<double> r_WT_in_xy_plane{X_WT.translation()};
    r_WT_in_xy_plane.z() = 0;
    drake::log()->debug("Table {}: Distance: {} m", i, r_WT_in_xy_plane.norm());
    if (r_WT_in_xy_plane.norm() < kMaxReach) {
      Vector3<double> r_WO_in_xy_plane{
          X_WO_initial_and_final->first.translation()};
      r_WO_in_xy_plane.z() = 0;
      const Vector3<double> dir_WO_in_xy_plane{r_WO_in_xy_plane.normalized()};
      double x = r_WT_in_xy_plane.dot(-dir_WO_in_xy_plane);
      double y = (r_WT_in_xy_plane - x * (-dir_WO_in_xy_plane))
                     .dot(Vector3<double>::UnitZ().cross(-dir_WO_in_xy_plane));
      double angle = std::atan2(y, x) + M_PI;
      //double angle = std::asin(-r_WO_in_xy_plane.normalized()
                                   //.cross(r_WT_in_xy_plane.normalized())
                                   //.z()) + 2*M_PI;
      drake::log()->debug("Table {}: x = {}, y = {}, Angle: {} degrees", i, x, y, angle*180/M_PI);
      if (angle > 20*M_PI/180 && angle < min_angle) {
        drake::log()->debug("Table {} is the new destination candidate.", i);
        destination_table_index = i;
        min_angle = angle;
      }
    }
  }
  drake::log()->debug("Destination Table Index: {}", destination_table_index);

  if (destination_table_index < 0) {
    drake::log()->debug("Cannot find a suitable destination table.");
    return false;
  }

  // Pose of destination table in world
  const Isometry3<double> X_WT = X_WS * table_poses.at(destination_table_index);
  const Vector3<double> r_WT = X_WT.translation();
  drake::log()->debug("r_WT = [{}]",
                      X_WT.translation().transpose());
  drake::log()->debug("R_WT = \n{}",
                      X_WT.linear());

  Vector3<double> dir_TO_final = -X_WT.linear().inverse()*r_WT;
  dir_TO_final.z() = 0;
  dir_TO_final.normalize();
  Vector3<double> r_TO_final = Vector3<double>::Zero();
  r_TO_final.z() += 0.5*env_state.get_object_dimensions().z();
  Matrix3<double> R_TO_final{Matrix3<double>::Identity()};
  R_TO_final.col(0) = -dir_TO_final;
  R_TO_final.col(2) = Vector3<double>::UnitZ();
  R_TO_final.col(1) = R_TO_final.col(2).cross(R_TO_final.col(0));
  Isometry3<double> X_TO_final;
  X_TO_final.translation() = r_TO_final;
  X_TO_final.linear() = R_TO_final;
  X_WO_initial_and_final->second = X_WT*X_TO_final;
  drake::log()->debug("dir_TO_final = [{}]", dir_TO_final.transpose());
  drake::log()->debug("r_TO_final = [{}]",
                      X_WO_initial_and_final->second.translation().transpose());
  drake::log()->debug("R_WO_final = \n{}",
                      X_WO_initial_and_final->second.linear());
  return true;
}

bool PickAndPlaceStateMachine::ComputeDesiredPoses(
    const WorldState& env_state, double yaw_offset, double pitch_offset) {
  X_WE_desired_.clear();

  //     
  //       (ApproachPickPregrasp,                         (ApproachPlacePregrasp
  //        LiftFromPick ),                                LiftFromPlace)
  //       +--------------------------------------------------------+
  //       |                                                        |
  //       |                                                        |
  //       + (ApproachPick)                         (ApproachPlace) +
  //
  // W  - planning World frame, coincides with kuka base frame.
  // S  - Sensor world frame
  // O  - Object frame
  // Oi - Object frame (initial)
  // Of - Object frame (final)
  // T  - Table frame
  // E  - End-effector frame
  // G  - Gripper frame
  std::pair<Isometry3<double>, Isometry3<double>> X_WO_initial_and_final;
  if (!ComputeInitialAndFinalObjectPoses(env_state, &X_WO_initial_and_final)) {
    return false;
  }

  Isometry3<double>& X_WOi = X_WO_initial_and_final.first;
  Isometry3<double>& X_WOf = X_WO_initial_and_final.second;

  X_WOi.rotate(AngleAxis<double>(yaw_offset, Vector3<double>::UnitZ()));

  // Gripper pose should some distance in from the edge of the object, and
  // should be rotated about the y-axis by pitch_offset
  Isometry3<double> X_OG{Isometry3<double>::Identity()};
  X_OG.rotate(AngleAxis<double>(pitch_offset, Vector3<double>::UnitY()));
  X_OG.translation().x() = std::max<double>(
      -0.5 * env_state.get_object_dimensions().x() + 0.02,
      std::min<double>(-0.5 * env_state.get_object_dimensions().x() +
                           0.07 * std::cos(pitch_offset),
                       0));

  // Gripper is rotated relative to the end effector link.
  Isometry3<double> X_GE{Isometry3<double>::Identity()};
  X_GE.rotate(Eigen::AngleAxisd(0.39269908, Eigen::Vector3d::UnitX()));


  // Set ApproachPick pose
  Isometry3<double> X_OiO{Isometry3<double>::Identity()};
  X_WE_desired_.emplace(PickAndPlaceState::kApproachPick,
                        X_WOi * X_OiO * X_OG * X_GE);

  // Set ApproachPickPregrasp pose
  Isometry3<double> X_GGoffset{Isometry3<double>::Identity()};
  //X_GGoffset.translate(Vector3<double>(-0.75*kPreGraspHeightOffset, 0.0, 0.0));
  X_OiO.setIdentity();
  const double approach_angle = 70.0 * M_PI / 180.0;
  X_OiO.translation()[0] = -cos(approach_angle)*kPreGraspHeightOffset;
  X_OiO.translation()[2] = sin(approach_angle)*kPreGraspHeightOffset;
  X_WE_desired_.emplace(PickAndPlaceState::kApproachPickPregrasp,
                        X_WOi * X_OiO * X_OG * X_GGoffset* X_GE);

  // Set LiftFromPick pose
  X_OiO.setIdentity();
  X_OiO.translation()[2] = kPreGraspHeightOffset;
  X_WE_desired_.emplace(PickAndPlaceState::kLiftFromPick,
                        X_WOi * X_OiO * X_OG * X_GE);

  // Set ApproachPlace pose
  Isometry3<double> X_OfO{Isometry3<double>::Identity()};
  X_WE_desired_.emplace(PickAndPlaceState::kApproachPlace,
                        X_WOf * X_OfO * X_OG * X_GE);

  // Set ApproachPlacePregrasp pose
  X_OfO.setIdentity();
  X_OfO.translation()[2] = kPreGraspHeightOffset;
  X_WE_desired_.emplace(PickAndPlaceState::kApproachPlacePregrasp,
                        X_WOf * X_OfO * X_OG * X_GE);

  // Set LiftFromPlace pose
  X_OfO.setIdentity();
  X_OfO.translation()[0] = -cos(approach_angle)*kPreGraspHeightOffset;
  X_OfO.translation()[2] = sin(approach_angle)*kPreGraspHeightOffset;
  X_WE_desired_.emplace(PickAndPlaceState::kLiftFromPlace,
                        X_WOf * X_OfO * X_OG * X_GE);
  return true;
}

PostureInterpolationRequest
PickAndPlaceStateMachine::CreatePostureInterpolationRequest(
    const WorldState& env_state, PickAndPlaceState state, double duration,
    bool fall_back_to_joint_space_interpolation) {
  PostureInterpolationRequest request;
  request.max_joint_position_change = 0.5*M_PI_4;
  request.q_initial = env_state.get_iiwa_q();
  request.q_final = nominal_q_map_.at(state);
  double max_delta_q{
      (request.q_final - request.q_initial).cwiseAbs().maxCoeff()};
  int num_via_points =
      std::ceil(2*max_delta_q / request.max_joint_position_change);
  double dt{duration/static_cast<double>(num_via_points)};
  request.times.resize(num_via_points + 1);
  request.times.front() = 0.0;
  for (int i = 1; i < static_cast<int>(request.times.size()) - 1; ++i) {
    request.times[i] = i*dt;
  }
  request.times.back() = duration;
  request.position_tolerance = tight_pos_tol_(0);
  request.orientation_tolerance = tight_rot_tol_;
  request.fall_back_to_joint_space_interpolation =
      fall_back_to_joint_space_interpolation;
  drake::log()->debug("Interpolation request has {} knots.",
                      request.times.size());
  return request;
}

PickAndPlaceStateMachine::PickAndPlaceStateMachine(
    const pick_and_place::PlannerConfiguration& configuration, bool single_move)
    : single_move_(single_move),
      state_(PickAndPlaceState::kOpenGripper),
      // Position and rotation tolerances.  These were hand-tuned by
      // adjusting to tighter bounds until IK stopped reliably giving
      // results.
      tight_pos_tol_(0.001, 0.001, 0.001),
      tight_rot_tol_(0.05),
      loose_pos_tol_(0.1, 0.1, 0.1),
      loose_rot_tol_(30 * M_PI / 180),
      configuration_(configuration) {}

PickAndPlaceStateMachine::~PickAndPlaceStateMachine() {}

bool PickAndPlaceStateMachine::ComputeNominalConfigurations(
    const RigidBodyTree<double>& iiwa, const WorldState& env_state) {
  bool success = false;
  nominal_q_map_.clear();
  //  Create vectors to hold the constraint objects
  std::vector<std::unique_ptr<WorldPositionConstraint>> position_constraints;
  std::vector<std::unique_ptr<WorldQuatConstraint>> orientation_constraints;
  std::vector<std::unique_ptr<PostureChangeConstraint>>
    posture_change_constraints;
  std::vector<std::vector<RigidBodyConstraint*>> constraint_array;
  std::vector<double> yaw_offsets{M_PI, 0.0};
  std::unique_ptr<RigidBodyTree<double>> robot{iiwa.Clone()};
  std::vector<double> pitch_offsets{M_PI/8};
  int kNumJoints = iiwa.get_num_positions();

  int end_effector_body_idx = robot->FindBodyIndex("iiwa_link_ee");
  Vector3<double> end_effector_points{kEndEffectorToMidFingerDepth, 0, 0};

  std::vector<PickAndPlaceState> states{
      PickAndPlaceState::kApproachPickPregrasp,
      PickAndPlaceState::kApproachPick,
      PickAndPlaceState::kLiftFromPick,
      PickAndPlaceState::kApproachPlacePregrasp,
      PickAndPlaceState::kApproachPlace,
      PickAndPlaceState::kLiftFromPlace};
  int kNumKnots = states.size() + 1;

  VectorX<double> q_initial{env_state.get_iiwa_q()};
  VectorX<double> q_seed_local{q_initial};
  VectorX<double> t{VectorX<double>::LinSpaced(kNumKnots, 0, kNumKnots - 1)};
  // Set up an inverse kinematics trajectory problem with one knot for each
  // state
  MatrixX<double> q_nom =
      MatrixX<double>::Zero(robot->get_num_positions(), kNumKnots);

  for (double pitch_offset : pitch_offsets) {
    for (double yaw_offset : yaw_offsets) {
      if (ComputeDesiredPoses(env_state, yaw_offset, pitch_offset)) {
        constraint_array.emplace_back();

        for (int i = 1; i < kNumKnots; ++i) {
          const PickAndPlaceState state{states[i-1]};
          const Vector2<double> knot_tspan{t(i), t(i)};

          // Extract desired position and orientation of end effector at the
          // given
          // state.
          const Isometry3<double>& X_WE = X_WE_desired_.at(state);
          const Vector3<double>& r_WE = X_WE.translation();
          const Quaternion<double>& quat_WE{X_WE.rotation()};

          // Constrain the end-effector position for all knots.
          position_constraints.emplace_back(new WorldPositionConstraint(
              robot.get(), end_effector_body_idx, end_effector_points,
              r_WE - tight_pos_tol_, r_WE + tight_pos_tol_, knot_tspan));
          constraint_array.back().push_back(position_constraints.back().get());

          // Constrain the end-effector orientation for all knots
          orientation_constraints.emplace_back(
              new WorldQuatConstraint(robot.get(), end_effector_body_idx,
                                      Eigen::Vector4d(quat_WE.w(), quat_WE.x(),
                                                      quat_WE.y(), quat_WE.z()),
                                      tight_rot_tol_, knot_tspan));
          constraint_array.back().push_back(
              orientation_constraints.back().get());

          // For each pair of adjacent knots, add a constraint on the change in
          // joint
          // positions.
          if (i > 1) {
            const VectorX<int> joint_indices =
                VectorX<int>::LinSpaced(kNumJoints, 0, kNumJoints - 1);
            const Vector2<double> segment_tspan{t(i - 1), t(i)};
            // The move to ApproachPlacePregrasp can require large joint
            // motions.
            const double max_joint_position_change =
                (state == PickAndPlaceState::kApproachPlacePregrasp)
                    ? 0.75 * M_PI
                    : M_PI_4;
            const VectorX<double> ub_change{max_joint_position_change *
                                            VectorX<double>::Ones(kNumJoints)};
            const VectorX<double> lb_change{-ub_change};
            posture_change_constraints.emplace_back(new PostureChangeConstraint(
                robot.get(), joint_indices, lb_change, ub_change, segment_tspan));
            constraint_array.back().push_back(
                posture_change_constraints.back().get());
          }
        }
      }
    }
  }

  if (constraint_array.empty()) return false;

  // Solve the IK problem. Re-seed with random values if the initial seed is
  // unsuccessful.
  IKResults ik_res;
  IKoptions ikoptions(robot.get());
  ikoptions.setFixInitialState(true);
  const int kNumRestarts = 50;
  for (int i = 0; i < kNumRestarts; ++i) {
    MatrixX<double> q_knots_seed{robot->get_num_positions(), kNumKnots};
    for (int j = 0; j < kNumKnots; ++j) {
      double s = static_cast<double>(j)/static_cast<double>(kNumKnots-1);
      q_knots_seed.col(j) = (1 - s) * q_initial + s * q_seed_local;
    }
    ik_res =
        inverseKinTrajSimple(robot.get(), t, q_knots_seed, q_nom,
                             constraint_array[i % constraint_array.size()], ikoptions);
    success = ik_res.info[0] == 1;
    if (success) {
      q_seed_local = ik_res.q_sol.back();
      break;
    } else {
      q_seed_local = robot->getRandomConfiguration(rand_generator_);
      drake::log()->warn("Attempt {} failed with info {}", i, ik_res.info[0]);
    }
  }
  if (success) {
    for (int i = 1; i < kNumKnots; ++i) {
      drake::log()->debug("State {}: q = ({})", states[i-1],
                          ik_res.q_sol[i].transpose());
      nominal_q_map_.emplace(states[i-1], ik_res.q_sol[i]);
    }
  }
  return success;
}

bool PickAndPlaceStateMachine::ComputeTrajectories(const RigidBodyTree<double>& iiwa, const WorldState& env_state) {
  bool success = ComputeNominalConfigurations(iiwa, env_state);
  if (!success) return false;
  std::vector<PickAndPlaceState> states{
      PickAndPlaceState::kApproachPickPregrasp,
      PickAndPlaceState::kApproachPick,
      PickAndPlaceState::kLiftFromPick,
      PickAndPlaceState::kApproachPlacePregrasp,
      PickAndPlaceState::kApproachPlace,
      PickAndPlaceState::kLiftFromPlace};
  VectorX<double> q_0{iiwa.get_num_positions()};
  if (interpolation_result_map_.empty()) {
    q_0 << env_state.get_iiwa_q();
    drake::log()->debug("Using current configuration as q_0.");
  } else {
    drake::log()->debug("Using end of the last plan as q_0.");
    const PiecewisePolynomial<double>& q_traj_last =
        interpolation_result_map_.at(states.back()).q_traj;
    q_0 << q_traj_last.value(q_traj_last.getEndTime());
  }
  drake::log()->debug("\tq_0 = [{}]", q_0.transpose());
  drake::log()->debug("Clearing interpolation_result_map_.");
  interpolation_result_map_.clear();
  const double kExtraShortDuration = 0.5;
  const double kShortDuration = 1;
  const double kLongDuration = 1.5;
  const double kExtraLongDuration = 1.5;
  int kNumJoints = iiwa.get_num_positions();
  for (PickAndPlaceState state : states) {
    drake::log()->info("Planning trajectory for {}.", state);
    const VectorX<double> q_f = nominal_q_map_.at(state);
    PostureInterpolationResult result;
    if ((q_f - q_0).array().abs().maxCoeff() < 10*M_PI/180) {
      // If very close, just interpolate in joint space.
      VectorX<double> q_dot0{VectorX<double>::Zero(kNumJoints)};
      VectorX<double> q_dotf{VectorX<double>::Zero(kNumJoints)};

      result.success = true;
      result.q_traj = PiecewisePolynomial<double>::Cubic(
          {0, kExtraShortDuration}, {q_0, q_f}, q_dot0, q_dotf);
    } else {
      double duration{kShortDuration};
      double position_tolerance{tight_pos_tol_(0)};
      double orientation_tolerance{tight_rot_tol_};
      bool fall_back_to_joint_space_interpolation{false};
      switch (state) {
        case PickAndPlaceState::kApproachPickPregrasp:
        case PickAndPlaceState::kApproachPlacePregrasp: {
          position_tolerance = loose_pos_tol_(0);
          orientation_tolerance = loose_rot_tol_;
          fall_back_to_joint_space_interpolation = true;
          duration = kLongDuration;
        } break;

        case PickAndPlaceState::kLiftFromPlace: {
          position_tolerance = loose_pos_tol_(0);
          orientation_tolerance = loose_rot_tol_;
          fall_back_to_joint_space_interpolation = true;
          duration = kExtraLongDuration;
        } break;

        default:  // No action needed for other cases
          break;
      }
      PostureInterpolationRequest request;

      request.max_joint_position_change = 0.5 * M_PI_4;
      request.q_initial = q_0;
      request.q_final = q_f;
      double max_delta_q{
          (request.q_final - request.q_initial).cwiseAbs().maxCoeff()};
      int num_via_points = std::min<int>(
          3, std::ceil(2 * max_delta_q / request.max_joint_position_change));
      double dt{duration / static_cast<double>(num_via_points)};
      request.times.resize(num_via_points + 1);
      request.times.front() = 0.0;
      for (int i = 1; i < static_cast<int>(request.times.size()) - 1; ++i) {
        request.times[i] = i * dt;
      }
      request.times.back() = duration;
      request.position_tolerance = position_tolerance;
      request.orientation_tolerance = orientation_tolerance;
      request.fall_back_to_joint_space_interpolation =
          fall_back_to_joint_space_interpolation;

      result = PlanStraightLineMotion(request, iiwa);
    }

    interpolation_result_map_.emplace(state, result);
    success = success && result.success;
    q_0 = q_f;
  }
  return success;
}

void PickAndPlaceStateMachine::Update(
    const WorldState& env_state,
    const IiwaPublishCallback& iiwa_callback,
    const WsgPublishCallback& wsg_callback) {
  IKResults ik_res;
  robotlocomotion::robot_plan_t stopped_plan{};
  stopped_plan.num_states = 0;

  PickAndPlaceState next_state{state_};
  auto schunk_action = OpenGripper;
  switch (state_) {
    case PickAndPlaceState::kOpenGripper: {
      next_state = PickAndPlaceState::kPlan;
    } break;

    case PickAndPlaceState::kApproachPickPregrasp: {
      next_state = PickAndPlaceState::kApproachPick;
    } break;

    case PickAndPlaceState::kApproachPick: {
      next_state = PickAndPlaceState::kGrasp;
    } break;

    case PickAndPlaceState::kGrasp: {
      schunk_action = CloseGripper;
      next_state = PickAndPlaceState::kLiftFromPick;
    } break;

    case PickAndPlaceState::kLiftFromPick: {
      next_state = PickAndPlaceState::kApproachPlacePregrasp;
    } break;

    case PickAndPlaceState::kApproachPlacePregrasp: {
      next_state = PickAndPlaceState::kApproachPlace;
    } break;

    case PickAndPlaceState::kApproachPlace: {
      next_state = PickAndPlaceState::kPlace;
    } break;

    case PickAndPlaceState::kPlace: {
        next_state = PickAndPlaceState::kLiftFromPlace;
    } break;

    case PickAndPlaceState::kLiftFromPlace: {
      next_state = PickAndPlaceState::kReset;
    } break;

    default: // No action needed for other cases
      break;
  }

  switch (state_) {

    // IIWA arm movements
    case PickAndPlaceState::kApproachPick:
    case PickAndPlaceState::kLiftFromPick:
    case PickAndPlaceState::kApproachPlace:
    case PickAndPlaceState::kLiftFromPlace:
    case PickAndPlaceState::kApproachPickPregrasp:
    case PickAndPlaceState::kApproachPlacePregrasp: {
      if (!iiwa_move_.ActionStarted()) {
        robotlocomotion::robot_plan_t plan{};
        std::vector<VectorX<double>> q;
        PostureInterpolationResult& result = interpolation_result_map_.at(state_); 
        DRAKE_THROW_UNLESS(result.success);
        const std::vector<double>& times{result.q_traj.getSegmentTimes()};
        q.reserve(times.size());
        for (double t : times) {
          q.push_back(result.q_traj.value(t));
        }
        WorldSimTreeBuilder<double> tree_builder;
        tree_builder.StoreModel("iiwa", configuration_.model_path);
        tree_builder.AddFixedModelInstance("iiwa", Vector3<double>::Zero());
        iiwa_move_.MoveJoints(env_state, *tree_builder.Build(), times, q, &plan);
        iiwa_callback(&plan);

        drake::log()->info("{} at {}", state_, env_state.get_iiwa_time());
        drake::log()->debug("\tq_0 = [{}]", q.front().transpose());
        drake::log()->debug("\tq_f = [{}]", q.back().transpose());
      }
      if (iiwa_move_.ActionFinished(env_state)) {
        // If the object has moved since kPlan, we need to replan.
        state_ = next_state;
        switch (state_) {
          case PickAndPlaceState::kApproachPick:
          case PickAndPlaceState::kApproachPickPregrasp: {
            if (!env_state.get_object_pose().translation().isApprox(expected_object_pose_.translation(), 0.05)) {
              drake::log()->info("Target moved! Re-planning ...");
              interpolation_result_map_.clear();
              state_ = PickAndPlaceState::kPlan;
            }
          }

          default: // No action needed for other cases
            break;
        }
        iiwa_move_.Reset();
      }
      break;
    }

    // Schunk gripper actions
    case PickAndPlaceState::kOpenGripper: {
      if (!wsg_act_.ActionStarted()) {
        const Isometry3<double>& obj_pose = env_state.get_object_pose();
        drake::log()->info("Object at: {} {}", obj_pose.translation().transpose(),
                           math::rotmat2rpy(obj_pose.rotation()).transpose());
        const Isometry3<double>& iiwa_pose = env_state.get_iiwa_base();
        drake::log()->info("Base at: {} {}", iiwa_pose.translation().transpose(),
                           math::rotmat2rpy(iiwa_pose.rotation()).transpose());
      }
    } // Intentionally fall through
    case PickAndPlaceState::kGrasp:
    case PickAndPlaceState::kPlace: {
      if (!wsg_act_.ActionStarted()) {
        lcmt_schunk_wsg_command msg;
        schunk_action(env_state, &wsg_act_, &msg);
        wsg_callback(&msg);

        drake::log()->info("{} at {}", state_, env_state.get_iiwa_time());
      }

      if (wsg_act_.ActionFinished(env_state)) {
        state_ = next_state;
        wsg_act_.Reset();
      }
      break;
    }

    case PickAndPlaceState::kPlan: {
      // Compute all the desired configurations
      expected_object_pose_ = env_state.get_object_pose();
      WorldSimTreeBuilder<double> tree_builder;
      tree_builder.StoreModel("iiwa", configuration_.model_path);
      tree_builder.AddFixedModelInstance("iiwa", Vector3<double>::Zero());
      std::unique_ptr<RigidBodyTree<double>> iiwa = tree_builder.Build();
      drake::log()->info("Building model in kPlan.");
      if (ComputeTrajectories(*iiwa, env_state)) {
        // Proceed to execution
        state_ = PickAndPlaceState::kApproachPickPregrasp;
      } // otherwise re-plan on next call to Update.
      break;
    }

    case PickAndPlaceState::kReset: {
      if (single_move_) {
        state_ = PickAndPlaceState::kDone;
        iiwa_callback(&stopped_plan);
        drake::log()->info("{} at {}", state_, env_state.get_iiwa_time());
      } else {
        state_ = PickAndPlaceState::kOpenGripper;
      }
      break;
    }

    case PickAndPlaceState::kDone: {
      break;
    }
  }
}

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
