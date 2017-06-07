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

      // Perform nearphase distance computation
      fcl::distance(fcl_object_A, fcl_object_B, request, result);

      // Let the closest points on elements A and B be denoted by P and Q
      // respectively
      const Vector3d& p_AP = result.nearest_points[0];
      const Vector3d& p_BQ = result.nearest_points[1];

      double d_QP = result.min_distance;

      // Transform the closest points to their respective body frames.
      // Let element A be on body C and element B
      const Isometry3d X_CA = element_A->getLocalTransform();
      const Isometry3d X_DB = element_B->getLocalTransform();
      const Isometry3d X_WA = element_A->getWorldTransform();
      const Isometry3d X_WB = element_B->getWorldTransform();
      const Vector3d p_CP = X_CA * p_AP;
      const Vector3d p_DQ = X_DB * p_BQ;

      // Define the normal as the unit vector from Q to P
      auto n_QP = (X_WA*p_AP - X_WB*p_BQ)/d_QP;

      distance_data->closest_points->emplace_back(element_A, element_B, 
          p_CP, p_DQ, n_QP, d_QP);
  }
  return false; // Check all N^2 pairs
}

/// @brief Distance data stores the distance request and the result given by distance algorithm.
struct CollisionData
{
  /// @brief Collision request
  fcl::CollisionRequestd request;

  /// @brief Vector of distance results
  std::vector<PointPair>* closest_points{nullptr};
};

bool collisionPointsFunction(fcl::CollisionObjectd* fcl_object_A,
    fcl::CollisionObjectd* fcl_object_B, void* callback_data)
{
  auto element_A = static_cast<Element*>(fcl_object_A->getUserData());
  auto element_B = static_cast<Element*>(fcl_object_B->getUserData());
  if (element_A && element_B && element_A->CanCollideWith(element_B)) {
      // Unpack the callback data
      auto* collision_data = static_cast<CollisionData*>(callback_data);
      const fcl::CollisionRequestd& request = collision_data->request;
      fcl::CollisionResultd result;

      // Perform nearphase collision detection
      fcl::collide(fcl_object_A, fcl_object_B, request, result);

      // Process the contact points
      std::vector<fcl::Contactd> contacts;
      result.getContacts(contacts);

      for (auto contact : contacts) {
        // Signed distance is negative when penetration depth is positive
        double d_QP = -contact.penetration_depth;
        // Define the normal as the unit vector from Q to P (opposite
        // convention from FCL)
        Vector3d n_QP = -contact.normal;
        if (element_B->getShape() == DrakeShapes::MESH) {
          if (element_A->getShape() == DrakeShapes::SPHERE) {
            // Penetration depth sign convention is reversed for sphere-mesh contact???
            d_QP = -d_QP;
          }
          n_QP *= -1;
        }

        // FCL returns a single contact point, but PointPair expects two
        const Vector3d p_WP{contact.pos + 0.5*d_QP*n_QP};
        const Vector3d p_WQ{contact.pos - 0.5*d_QP*n_QP};

        // Transform the closest points to their respective body frames.
        // Let element A be on body C and element B
        //const Isometry3d X_CA = element_A->getLocalTransform();
        //const Isometry3d X_DB = element_B->getLocalTransform();
        //const Isometry3d X_AW = element_A->getWorldTransform().inverse();
        //const Isometry3d X_BW = element_B->getWorldTransform().inverse();
        //const Vector3d p_CP = X_CA * X_AW * p_WP;
        //const Vector3d p_DQ = X_DB * X_BW * p_WQ;

        collision_data->closest_points->emplace_back(element_A, element_B, 
            p_WP, p_WQ, n_QP, d_QP);
      }
  }
  return false; // Check all pairs provided by the broadphase
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
        const auto mesh =
            static_cast<const DrakeShapes::Mesh&>(element.getGeometry());
        DrakeShapes::PointsVector vertices;
        DrakeShapes::TrianglesVector triangles;
        mesh.LoadObjFile(&vertices, &triangles, DrakeShapes::Mesh::TriangulatePolicy::kTry);
        typedef fcl::OBBRSSd bvh_type;
        fcl_geometry = std::shared_ptr<fcl::CollisionGeometryd>(new fcl::BVHModel<bvh_type>());
        auto fcl_bvh{static_cast<fcl::BVHModel<bvh_type>*>(fcl_geometry.get())};
        int status;
        status = fcl_bvh->beginModel(triangles.size(), vertices.size());
        DRAKE_ASSERT(status == fcl::BVH_OK);
        for (auto triangle : triangles) {
          fcl_bvh->addTriangle(vertices[triangle(2)], vertices[triangle(1)], vertices[triangle(0)]);
        }
        status = fcl_bvh->endModel();
        DRAKE_ASSERT(status == fcl::BVH_OK);
      } break;
      case DrakeShapes::MESH_POINTS: {
        DRAKE_ABORT_MSG("Not implemented.");
      } break;
      case DrakeShapes::CAPSULE: {
        const auto capsule =
            static_cast<const DrakeShapes::Capsule&>(element.getGeometry());
        fcl_geometry = std::shared_ptr<fcl::CollisionGeometryd>(new fcl::Capsuled(capsule.radius, capsule.length));
      } break;
      case DrakeShapes::HALFSPACE: {
        fcl_geometry = std::shared_ptr<fcl::CollisionGeometryd>(new fcl::Halfspaced(0, 0, 1, 0));
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

      // NOTE: The FCL collision object is assigned the Drake element's
      // world transform.  This will be the *only* time that anchored collision
      // objects will have their world transform set.  This code assumes that
      // the world transform on the corresponding input Drake element has
      // already been properly set. (See RigidBodyTree::CompileCollisionState.)
      fcl_object->setTransform(element.getWorldTransform());

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
  broadphase_manager_.update();
}

Eigen::Transform<double, 3, Eigen::AffineCompact> FCLModel::getFclObjectTransform(ElementId id) {
  return fcl_collision_objects_[id]->getTransform();
}

bool FCLModel::closestPointsAllToAll(const std::vector<ElementId>& ids_to_check,
                                     bool use_margins,
                                     std::vector<PointPair>& closest_points) {
  DistanceData distance_data;
  distance_data.closest_points = &closest_points;
  distance_data.request.enable_nearest_points = true;
  distance_data.request.enable_signed_distance = true;
  distance_data.request.gjk_solver_type = narrowphase_solver_type_;
  distance_data.request.distance_tolerance = std::pow(std::numeric_limits<double>::epsilon(), 0.7);
  broadphase_manager_.distance(static_cast<void*>(&distance_data), allToAllDistanceFunction);
  return true;
}

bool FCLModel::ComputeMaximumDepthCollisionPoints(
    bool use_margins, std::vector<PointPair>& points) {
  CollisionData collision_data;
  collision_data.closest_points = &points;
  collision_data.request.gjk_solver_type = narrowphase_solver_type_;
  collision_data.request.enable_contact = true;
  collision_data.request.num_max_contacts = 1e3;
  collision_data.request.collision_tolerance = 1e-12;
  broadphase_manager_.collide(static_cast<void*>(&collision_data), collisionPointsFunction);
  return true;
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

fcl::GJKSolverType FCLModel::get_narrowphase_solver_type() const {
  return narrowphase_solver_type_;
};

void FCLModel::set_narrowphase_solver_type(fcl::GJKSolverType new_type) {
  narrowphase_solver_type_ = new_type;
}
}  // namespace DrakeCollision
