#include "drake/multibody/collision/fcl_model.h"

#include <fcl/fcl.h>
#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/common/unused.h"

using Eigen::Isometry3d;

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
bool allToAllDistanceFunction(fcl::CollisionObjectd* o1,
    fcl::CollisionObjectd* o2, void* cdata_, double& dist)
{
  auto element1 = static_cast<Element*>(o1->getUserData());
  auto element2 = static_cast<Element*>(o1->getUserData());
  if (element1 && element2 && element1->CanCollideWith(element2)) {
      auto* cdata = static_cast<DistanceData*>(cdata_);
      const fcl::DistanceRequestd& request = cdata->request;
      fcl::DistanceResultd result;

      distance(o1, o2, request, result);

      double distance = result.min_distance;
      //auto normal = result.nearest_points[0]
      //cdata->closest_points->emplace_back(element1, element2, 
          //result.nearest_points[0], result.nearest_points[1], 

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
    fcl_collision_objects_[id]->setTransform(T_local_to_world);
  }
  return element_exists;
}
void FCLModel::updateModel() {
  DRAKE_ABORT_MSG("Not implemented.");
}

bool FCLModel::closestPointsAllToAll(const std::vector<ElementId>& ids_to_check,
                                     bool use_margins,
                                     std::vector<PointPair>& closest_points) {
  return false;
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
