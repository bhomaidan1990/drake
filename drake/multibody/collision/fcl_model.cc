#include "drake/multibody/collision/fcl_model.h"

#include <fcl/fcl.h>
#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/common/unused.h"

using Eigen::Isometry3d;
using Eigen::Vector3d;

namespace DrakeCollision {

// TODO(avalenzu): Implement the model.

// TODO(avalenzu): Tailor DistanceData and distanceFunction to our use-case

/// @brief Distance data stores the distance request and the result given by distance algorithm.
struct DistanceData
{
  /// @brief Distance request
  fcl::DistanceRequestd request;

  /// @brief Vector of distance results
  std::vector<PointPair>* closest_points{nullptr};
};

/// @brief Default distance callback for two objects o1 and o2 in broad phase. return value means whether the broad phase can stop now. also return dist, i.e. the bmin distance till now
bool allToAllDistanceFunction(fcl::CollisionObjectd* fcl_object_A,
    fcl::CollisionObjectd* fcl_object_B, void* callback_data, double& dist)
{
  auto element_A = static_cast<Element*>(fcl_object_A->getUserData());
  auto element_B = static_cast<Element*>(fcl_object_B->getUserData());
  if (element_A && element_B && element_A->CanCollideWith(element_B)) {
      // Unpack the callback data
      auto* distance_data = static_cast<DistanceData*>(callback_data);
      const fcl::DistanceRequestd& request = distance_data->request;
      fcl::DistanceResultd result;
      fcl::detail::GJKSolver_indepd gjk_solver;
      //TODO(avalenzu): Make this tolerance adjustable
      gjk_solver.gjk_tolerance = std::pow(std::numeric_limits<double>::epsilon(), 0.7);

      // Perform nearphase distance computation
      fcl::distance(fcl_object_A, fcl_object_B, &gjk_solver, request, result);

      // Let the closest points on elements A and B be denoted by P and Q
      // respectively
      //const Vector3d& p_WP = result.nearest_points[0];
      //const Vector3d& p_WQ = result.nearest_points[1];
      const Vector3d& p_AP = result.nearest_points[0];
      const Vector3d& p_BQ = result.nearest_points[1];

      double d_QP = result.min_distance;

      // Transform the closest points to their respective body frames.
      // Let element A be on body C and element B
      const Isometry3d X_CA = element_A->getLocalTransform();
      const Isometry3d X_DB = element_B->getLocalTransform();
      //const Isometry3d X_AW = element_A->getWorldTransform().inverse();
      //const Isometry3d X_BW = element_B->getWorldTransform().inverse();
      const Isometry3d X_WA = element_A->getWorldTransform();
      const Isometry3d X_WB = element_B->getWorldTransform();
      //const Vector3d p_CP = X_CA * X_AW * p_WP;
      //const Vector3d p_DQ = X_DB * X_BW * p_WQ;
      const Vector3d p_CP = X_CA * p_AP;
      const Vector3d p_DQ = X_DB * p_BQ;

      // Define the normal as the unit vector from Q to P
      auto n_QP = (X_WA*p_AP - X_WB*p_BQ)/d_QP;

      distance_data->closest_points->emplace_back(element_A, element_B, 
          p_CP, p_DQ, n_QP, d_QP);
  }
  return false; // Check all N^2 pairs
}

void FCLModel::DoAddElement(const Element& element) {

  ElementId id = element.getId();

  if (id != 0) {
    // Create the fcl::CollisionObject
    std::shared_ptr<fcl::CollisionGeometryd> fcl_geometry;

    switch (element.getShape()) {
      case DrakeShapes::BOX: {
        const auto box =
            static_cast<const DrakeShapes::Box&>(element.getGeometry());
        fcl_geometry = std::shared_ptr<fcl::CollisionGeometryd>(new fcl::Boxd(box.size));
      } break;
      case DrakeShapes::SPHERE: {
        const auto sphere =
            static_cast<const DrakeShapes::Sphere&>(element.getGeometry());
        fcl_geometry = std::shared_ptr<fcl::CollisionGeometryd>(new fcl::Sphered(sphere.radius));
      } break;
      case DrakeShapes::CYLINDER: {
        const auto cylinder =
            static_cast<const DrakeShapes::Cylinder&>(element.getGeometry());
        fcl_geometry = std::shared_ptr<fcl::CollisionGeometryd>(new fcl::Cylinderd(cylinder.radius, cylinder.length));
      } break;
      case DrakeShapes::MESH: {
        DRAKE_ABORT_MSG("Not implemented.");
      } break;
      case DrakeShapes::MESH_POINTS: {
        DRAKE_ABORT_MSG("Not implemented.");
      } break;
      case DrakeShapes::CAPSULE: {
        const auto capsule =
            static_cast<const DrakeShapes::Capsule&>(element.getGeometry());
        fcl_geometry = std::shared_ptr<fcl::CollisionGeometryd>(new fcl::Capsuled(capsule.radius, capsule.length));
      } break;
      default:
        DRAKE_ABORT_MSG("Not implemented.");
        //std::cerr << "Warning: Collision elements[id] has an unknown type "
                  //<< element.getShape() << std::endl;
        //throw UnknownShapeException(element.getShape());
        break;
    }
    
    if (fcl_geometry != nullptr) {
      std::unique_ptr<fcl::CollisionObjectd> fcl_object{new fcl::CollisionObjectd(fcl_geometry)};

      // Store a pointer to the Element in the fcl collision object
      fcl_object->setUserData(FindMutableElement(id));
      // Register the object with the collision manager
      broadphase_manager_.registerObject(fcl_object.get());
      // Take ownership of FCL collision object
      fcl_collision_objects_.insert(std::make_pair(id, move(fcl_object)));
    }
  }
}

bool FCLModel::updateElementWorldTransform(
    ElementId id, const Isometry3d& T_local_to_world) {
  const bool element_exists(
      Model::updateElementWorldTransform(id, T_local_to_world));
  if (element_exists) {
    fcl_collision_objects_[id]->setTransform(FindElement(id)->getWorldTransform());
  }
  return element_exists;
}
void FCLModel::updateModel() {
  DRAKE_ABORT_MSG("Not implemented.");
}

bool FCLModel::closestPointsAllToAll(const std::vector<ElementId>& ids_to_check,
                                     bool use_margins,
                                     std::vector<PointPair>& closest_points) {
  DistanceData distance_data;
  distance_data.closest_points = &closest_points;
  distance_data.request.enable_nearest_points = true;
  distance_data.request.enable_signed_distance = true;
  distance_data.request.gjk_solver_type = fcl::GJKSolverType::GST_INDEP;
  broadphase_manager_.distance(static_cast<void*>(&distance_data), allToAllDistanceFunction);
  return true;
}

bool FCLModel::ComputeMaximumDepthCollisionPoints(
    bool use_margins, std::vector<PointPair>& points) {
  drake::unused(use_margins, points);
  DRAKE_ABORT_MSG("Not implemented.");
  return false;
}

void FCLModel::collisionDetectFromPoints(
    const Eigen::Matrix3Xd& points, bool use_margins,
    std::vector<PointPair>& closest_points) {
  drake::unused(points, use_margins, closest_points);
  DRAKE_ABORT_MSG("Not implemented.");
}

void FCLModel::ClearCachedResults(bool use_margins) {
  drake::unused(use_margins);
  DRAKE_ABORT_MSG("Not implemented.");
}

bool FCLModel::collisionRaycast(const Eigen::Matrix3Xd& origins,
                                const Eigen::Matrix3Xd& ray_endpoints,
                                bool use_margins, Eigen::VectorXd& distances,
                                Eigen::Matrix3Xd& normals) {
  drake::unused(origins, ray_endpoints, use_margins, distances, normals);
  DRAKE_ABORT_MSG("Not implemented.");
  return false;
}

std::vector<PointPair> FCLModel::potentialCollisionPoints(bool use_margins) {
  drake::unused(use_margins);
  DRAKE_ABORT_MSG("Not implemented.");
  return std::vector<PointPair>();
}

bool FCLModel::collidingPointsCheckOnly(
    const std::vector<Eigen::Vector3d>& input_points,
    double collision_threshold) {
  drake::unused(input_points, collision_threshold);
  DRAKE_ABORT_MSG("Not implemented.");
  return false;
}

std::vector<size_t> FCLModel::collidingPoints(
    const std::vector<Eigen::Vector3d>& input_points,
    double collision_threshold) {
  drake::unused(input_points, collision_threshold);
  DRAKE_ABORT_MSG("Not implemented.");
  return std::vector<size_t>();
}

}  // namespace DrakeCollision
