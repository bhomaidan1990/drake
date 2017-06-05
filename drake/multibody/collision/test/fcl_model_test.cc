#include "drake/multibody/collision/fcl_model.h"

#include <fcl/fcl.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"

using Eigen::Isometry3d;
using Eigen::Vector3d;
using Eigen::Matrix3Xd;
using std::make_unique;
using std::unique_ptr;
using drake::CompareMatrices;

namespace DrakeCollision {
namespace {

// Structure used to hold the analytical solution of the tests.
// It stores the collision point on the surface of a collision body in both
// world and body frames.
struct SurfacePoint {
  SurfacePoint() {}
  SurfacePoint(Vector3d wf, Vector3d bf) : world_frame(wf), body_frame(bf) {}
  // Eigen variables are left uninitalized by default.
  Vector3d world_frame;
  Vector3d body_frame;
};

// Solutions are accessed by collision element id using an std::unordered_set.
// DrakeCollision::Model returns the collision detection results as a vector of
// DrakeCollision::PointPair entries. Each entry holds a reference to the pair
// of collision elements taking part in the collision. Collision elements are
// referenced by their id.
// The order in which the pair of elements is stored in a PointPair cannot
// be guaranteed, and therefore we cannot guarantee the return of
// PointPair::idA and PointPair::idB in our tests.
// This means we cannot guarantee that future versions of the underlying
// implementation (say Bullet, FCL) won't change this order (since unfortunately
// id's are merely a memory address cast to an integer).
// The user only has access to collision elements by id.
// To provide a unique mapping between id's and the analytical solution to the
// contact point on a specific element here we use an `std::unordered_set` to
// map id's to a `SurfacePoint` structure holding the analytical solution on
// both body and world frames.
typedef std::unordered_map<const DrakeCollision::Element*, SurfacePoint>
    ElementToSurfacePointMap;

// TODO(jamiesnape): Test the model.

// TODO(jamiesnape): Remove this test as redundant once model is implemented.
//
// Two spheres of diameter 1.0 are placed 0.75 apart.  The spheres overlap by
// 0.25. Only one contact point is expected for the collision of two spheres.
class SphereVsSphereTest : public ::testing::Test {
 public:
  void SetUp() override {
    DrakeShapes::Sphere sphere_A(0.5);
    DrakeShapes::Sphere sphere_B(0.5);

    // Populate the model.
    model_ = unique_ptr<Model>(new FCLModel());
    sphere_A_ = model_->AddElement(make_unique<Element>(sphere_A));
    sphere_B_ = model_->AddElement(make_unique<Element>(sphere_B));

    // Access the analytical solution to the contact point on the surface of
    // each collision element by element id.
    // Solutions are expressed in world and body frames.
    solution_ = {
        /*           world frame     , body frame  */
        {sphere_A_,    {{0.0,  0.5, 0.0}, {0.0,  0.0, 0.5}}},
        {sphere_B_, {{0.0, 0.25, 0.0}, {0.0, -0.5, 0.0}}}};

    // Body 1 pose
    Isometry3d sphere_A_pose;
    sphere_A_pose.setIdentity();
    sphere_A_pose.rotate(Eigen::AngleAxisd(M_PI_2, Vector3d(-1.0, 0.0, 0.0)));
    model_->updateElementWorldTransform(sphere_A_->getId(), sphere_A_pose);

    // Body 2 pose
    Isometry3d sphere_B_pose;
    sphere_B_pose.setIdentity();
    sphere_B_pose.translation() = Vector3d(0.0, 0.75, 0.0);
    model_->updateElementWorldTransform(sphere_B_->getId(), sphere_B_pose);
  }

 protected:
  double tolerance_;
  std::unique_ptr<Model> model_;
  Element* sphere_A_, * sphere_B_;
  ElementToSurfacePointMap solution_;
};

TEST_F(SphereVsSphereTest, SingleContact) {
  // Numerical precision tolerance to perform floating point comparisons.
  // Its magnitude was chosen to be the minimum value for which these tests can
  // successfully pass.
  tolerance_ = 1.0e-9;

  // List of collision points.
  std::vector<PointPair> points;

  // Collision test performed with Model::ComputeMaximumDepthCollisionPoints.
  // Not using margins.
  points.clear();
  model_->ComputeMaximumDepthCollisionPoints(false, points);
  ASSERT_EQ(1u, points.size());
  EXPECT_NEAR(-0.25, points[0].distance, tolerance_);
  // Points are in the world frame on the surface of the corresponding body.
  // That is why ptA is generally different from ptB, unless there is
  // an exact non-penetrating collision.
  // WARNING:
  // This convention is different from the one used by closestPointsAllToAll
  // which computes points in the local frame of the body.
  // TODO(amcastro-tri): make these two conventions match? does this interfere
  // with any Matlab functionality?
  EXPECT_TRUE(CompareMatrices(points[0].normal, Vector3d(0.0, -1.0, 0.0),
                              tolerance_, drake::MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(points[0].ptA,
                              solution_[points[0].elementA].world_frame,
                              tolerance_, drake::MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(points[0].ptB,
                              solution_[points[0].elementB].world_frame,
                              tolerance_, drake::MatrixCompareType::absolute));

  points.clear();
  // Body 1 pose
  Isometry3d sphere_A_pose{sphere_A_->getWorldTransform()};
  sphere_A_pose.translation() += Vector3d(0, -1, 0);
  model_->updateElementWorldTransform(sphere_A_->getId(), sphere_A_pose);
  model_->ComputeMaximumDepthCollisionPoints(false, points);
  ASSERT_EQ(0u, points.size());
}

// A sphere of diameter 1.0 is placed on top of a halfspace.  The sphere
// overlaps with the halfspace with its deepest penetration point (the bottom)
// 0.25 units into the halfspace (negative distance). Only one contact point is
// expected when colliding with a sphere.
class HalfspaceVsSphereTest : public ::testing::Test {
 public:
  void SetUp() override {
    DrakeShapes::Halfspace halfspace;
    DrakeShapes::Sphere sphere(0.5);

    // Populate the model.
    model_ = unique_ptr<Model>(new FCLModel());
    halfspace_ = model_->AddElement(make_unique<Element>(halfspace));
    sphere_ = model_->AddElement(make_unique<Element>(sphere));

    // Access the analytical solution to the contact point on the surface of
    // each collision element by element id.
    // Solutions are expressed in world and body frames.
    solution_ = {
        /*           world frame     , body frame  */
        {halfspace_,    {{0.0,  0.0, 0.0}, {0.0,  0.0, 0.0}}},
        {sphere_, {{0.0, -0.25, 0.0}, {0.0, -0.5, 0.0}}}};

    // Body 1 pose
    Isometry3d halfspace_pose;
    halfspace_pose.setIdentity();
    halfspace_pose.rotate(Eigen::AngleAxisd(M_PI_2, Vector3d(-1.0, 0.0, 0.0)));
    model_->updateElementWorldTransform(halfspace_->getId(), halfspace_pose);

    // Body 2 pose
    Isometry3d sphere_pose;
    sphere_pose.setIdentity();
    sphere_pose.translation() = Vector3d(0.0, 0.25, 0.0);
    model_->updateElementWorldTransform(sphere_->getId(), sphere_pose);
  }

 protected:
  double tolerance_;
  std::unique_ptr<Model> model_;
  Element* halfspace_, * sphere_;
  ElementToSurfacePointMap solution_;
};

TEST_F(HalfspaceVsSphereTest, SingleContact) {
  // Numerical precision tolerance to perform floating point comparisons.
  // Its magnitude was chosen to be the minimum value for which these tests can
  // successfully pass.
  tolerance_ = 1.0e-9;

  // List of collision points.
  std::vector<PointPair> points;

  // Collision test performed with Model::ComputeMaximumDepthCollisionPoints.
  // Not using margins.
  points.clear();
  model_->ComputeMaximumDepthCollisionPoints(false, points);
  ASSERT_EQ(1u, points.size());
  EXPECT_NEAR(-0.25, points[0].distance, tolerance_);
  // Points are in the world frame on the surface of the corresponding body.
  // That is why ptA is generally different from ptB, unless there is
  // an exact non-penetrating collision.
  // WARNING:
  // This convention is different from the one used by closestPointsAllToAll
  // which computes points in the local frame of the body.
  // TODO(amcastro-tri): make these two conventions match? does this interfere
  // with any Matlab functionality?
  EXPECT_TRUE(CompareMatrices(points[0].normal, Vector3d(0.0, -1.0, 0.0),
                              tolerance_, drake::MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(points[0].ptA,
                              solution_[points[0].elementA].world_frame,
                              tolerance_, drake::MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(points[0].ptB,
                              solution_[points[0].elementB].world_frame,
                              tolerance_, drake::MatrixCompareType::absolute));

  points.clear();
  // Body 1 pose
  Isometry3d halfspace_pose{halfspace_->getWorldTransform()};
  halfspace_pose.translation() = Vector3d(0, -1, 0);
  model_->updateElementWorldTransform(halfspace_->getId(), halfspace_pose);
  model_->ComputeMaximumDepthCollisionPoints(false, points);
  ASSERT_EQ(0u, points.size());
}

}  // namespace
}  // namespace DrakeCollision
