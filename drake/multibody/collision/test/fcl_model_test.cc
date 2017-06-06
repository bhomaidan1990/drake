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
using std::vector;

namespace DrakeCollision {
namespace {

// Structure used to hold the analytical solution of the tests.
// It stores the collision point on the surface of a collision body in both
// world and body frames.
struct SurfacePoint {
  SurfacePoint() {}
  SurfacePoint(Vector3d wf, Vector3d bf, Vector3d n = Vector3d::Zero()) : world_frame(wf), body_frame(bf), normal(n) {}
  // Eigen variables are left uninitalized by default.
  Vector3d world_frame;
  Vector3d body_frame;
  Vector3d normal;
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

struct ShapeVsShapeTestParam {
  ShapeVsShapeTestParam(const DrakeShapes::Geometry& shape_A,
                        const DrakeShapes::Geometry& shape_B,
                        Isometry3d X_WA, Isometry3d X_WB,
                        SurfacePoint surface_point_A,
                        SurfacePoint surface_point_B,
                        fcl::GJKSolverType gjk_solver_type)
  : elements_(std::piecewise_construct, std::forward_as_tuple(shape_A),
              std::forward_as_tuple(shape_B)), 
    surface_points_(surface_point_A, surface_point_B),
    gjk_solver_type_(gjk_solver_type) {
    elements_.first.updateWorldTransform(X_WA);
    elements_.second.updateWorldTransform(X_WB);
  }

  ShapeVsShapeTestParam(const ShapeVsShapeTestParam& other)
    : ShapeVsShapeTestParam(other.elements_.first.getGeometry(),
                            other.elements_.second.getGeometry(),
                            other.elements_.first.getWorldTransform(),
                            other.elements_.second.getWorldTransform(),
                            other.surface_points_.first, 
                            other.surface_points_.second, 
                            other.gjk_solver_type_) {}

  std::pair<DrakeShapes::Element, DrakeShapes::Element> elements_;
  std::pair<SurfacePoint, SurfacePoint> surface_points_;
  fcl::GJKSolverType gjk_solver_type_;
};

void PrintTo(const ShapeVsShapeTestParam& param, ::std::ostream* os) {
  *os << DrakeShapes::ShapeToString(param.elements_.first.getShape());
  *os << "_"; 
  *os << DrakeShapes::ShapeToString(param.elements_.second.getShape());
  if (param.gjk_solver_type_ == fcl::GST_INDEP) {
    *os << "_GST:INDEP";
  } else {
    *os << "_GST:LIBCCD";
  }
}

class ShapeVsShapeTest : public ::testing::TestWithParam<ShapeVsShapeTestParam> {
 public:
  void SetUp() override {
    // Populate the model.
    ShapeVsShapeTestParam param = GetParam();
    model_ = unique_ptr<FCLModel>(new FCLModel());
    model_->set_narrowphase_solver_type(param.gjk_solver_type_);
    element_A_ = model_->AddElement(make_unique<Element>(param.elements_.first.getGeometry()));
    element_B_ = model_->AddElement(make_unique<Element>(param.elements_.second.getGeometry()));
    model_->updateElementWorldTransform(element_A_->getId(), param.elements_.first.getWorldTransform());
    model_->updateElementWorldTransform(element_B_->getId(), param.elements_.second.getWorldTransform());
    solution_ = {
        {element_A_, param.surface_points_.first},
        {element_B_, param.surface_points_.second}};
  }

 protected:
  double tolerance_;
  std::unique_ptr<FCLModel> model_;
  ElementToSurfacePointMap solution_;
  Element* element_A_;
  Element* element_B_;
};

TEST_P(ShapeVsShapeTest, ComputeMaximumDepthCollisionPoints) {
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

  auto point = points[0];
  Vector3d p_WP_expected = solution_[point.elementA].world_frame;
  Vector3d p_WQ_expected = solution_[point.elementB].world_frame;
  Vector3d n_QP_W_expected = solution_[point.elementB].normal;
  // Remainder of test assumes unit normal
  ASSERT_DOUBLE_EQ(n_QP_W_expected.norm(), 1);
  Vector3d p_QP_W_expected = p_WP_expected - p_WQ_expected;
  double distance_expected{p_QP_W_expected.dot(n_QP_W_expected)};

  EXPECT_NEAR(point.distance, distance_expected, tolerance_);
  // Points are in the world frame on the surface of the corresponding body.
  // That is why ptA is generally different from ptB, unless there is
  // an exact non-penetrating collision.
  // WARNING:
  // This convention is different from the one used by closestPointsAllToAll
  // which computes points in the local frame of the body.
  // TODO(amcastro-tri): make these two conventions match? does this interfere
  // with any Matlab functionality?
  EXPECT_TRUE(CompareMatrices(point.normal, n_QP_W_expected, tolerance_, 
        drake::MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(point.ptA, p_WP_expected, tolerance_, 
        drake::MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(point.ptB, p_WQ_expected, tolerance_,
        drake::MatrixCompareType::absolute));
}

vector<ShapeVsShapeTestParam> generateSphereVsSphereParam() {
  //First sphere
  DrakeShapes::Sphere sphere_A{0.5};
  Isometry3d X_WA;
  X_WA.setIdentity();
  X_WA.rotate(Eigen::AngleAxisd(M_PI_2, Vector3d(-1.0, 0.0, 0.0)));
  Vector3d p_WP{0.0,  0.5, 0.0};
  Vector3d p_AP{0.0,  0.0, 0.5};
  Vector3d n_PQ_W{0.0, 1.0, 0.0};
  SurfacePoint surface_point_A = {p_WP, p_AP, n_PQ_W};

  // Second sphere
  DrakeShapes::Sphere sphere_B{0.5};
  Isometry3d X_WB;
  X_WB.setIdentity();
  X_WB.translation() = Vector3d(0.0, 0.75, 0.0);
  Vector3d p_WQ{0.0,  0.25, 0.0};
  Vector3d p_BQ{0.0,  -0.5, 0.0};
  Vector3d n_QP_W{0.0, -1.0, 0.0};
  SurfacePoint surface_point_B = {p_WQ, p_BQ, n_QP_W};

  vector<ShapeVsShapeTestParam> params;
  params.push_back(ShapeVsShapeTestParam(sphere_A, sphere_B, X_WA, X_WB,
        surface_point_A, surface_point_B, fcl::GST_INDEP));
  params.push_back(ShapeVsShapeTestParam(sphere_A, sphere_B, X_WA, X_WB,
        surface_point_A, surface_point_B, fcl::GST_LIBCCD));
  params.push_back(ShapeVsShapeTestParam(sphere_B, sphere_A, X_WB, X_WA,
        surface_point_B, surface_point_A, fcl::GST_INDEP));
  params.push_back(ShapeVsShapeTestParam(sphere_B, sphere_A, X_WB, X_WA,
        surface_point_B, surface_point_A, fcl::GST_LIBCCD));
  
  return params;
}

INSTANTIATE_TEST_CASE_P(SphereVsSphere, ShapeVsShapeTest, ::testing::ValuesIn(generateSphereVsSphereParam()));

// A sphere of diameter 1.0 is placed on top of a halfspace.  The sphere
// overlaps with the halfspace with its deepest penetration point (the bottom)
// 0.25 units into the halfspace (negative distance). Only one contact point is
// expected when colliding with a sphere.
vector<ShapeVsShapeTestParam> generateHalfspaceVsSphereParam() {
  //Halfspace
  DrakeShapes::Halfspace halfspace;
  Isometry3d X_WA;
  X_WA.setIdentity();
  X_WA.rotate(Eigen::AngleAxisd(M_PI_2, Vector3d(-1.0, 0.0, 0.0)));
  Vector3d p_WP{0.0,  0.0, 0.0};
  Vector3d p_AP{0.0,  0.0, 0.0};
  Vector3d n_PQ_W{0.0, 1.0, 0.0};
  SurfacePoint surface_point_A = {p_WP, p_AP, n_PQ_W};

  // Sphere
  DrakeShapes::Sphere sphere{0.5};
  Isometry3d X_WB;
  X_WB.setIdentity();
  X_WB.translation() = Vector3d(0.0, 0.25, 0.0);
  Vector3d p_WQ{0.0,  -0.25, 0.0};
  Vector3d p_BQ{0.0,  -0.5, 0.0};
  Vector3d n_QP_W{0.0, -1.0, 0.0};
  SurfacePoint surface_point_B = {p_WQ, p_BQ, n_QP_W};

  vector<ShapeVsShapeTestParam> params;
  params.push_back(ShapeVsShapeTestParam(halfspace, sphere, X_WA, X_WB,
        surface_point_A, surface_point_B, fcl::GST_INDEP));
  params.push_back(ShapeVsShapeTestParam(halfspace, sphere, X_WA, X_WB,
        surface_point_A, surface_point_B, fcl::GST_LIBCCD));
  params.push_back(ShapeVsShapeTestParam(sphere, halfspace, X_WB, X_WA,
        surface_point_B, surface_point_A, fcl::GST_INDEP));
  params.push_back(ShapeVsShapeTestParam(sphere, halfspace, X_WB, X_WA,
        surface_point_B, surface_point_A, fcl::GST_LIBCCD));
  
  return params;
}

INSTANTIATE_TEST_CASE_P(HalfspaceVsSphere, ShapeVsShapeTest, ::testing::ValuesIn(generateHalfspaceVsSphereParam()));

// A sphere of diameter 1.0 is placed  above a box.  The sphere overlaps with
// the box with its deepest penetration point (the bottom) 0.25 units into the
// box (negative distance). Only one contact point is expected when colliding
// with a sphere.
vector<ShapeVsShapeTestParam> generateBoxVsSphereParam() {
  //Box
  DrakeShapes::Box box{Vector3d(1, 1, 1)};
  Isometry3d X_WA;
  X_WA.setIdentity();
  X_WA.translation() = Vector3d(0, 0.5, 0);
  Vector3d p_WP{0.0,  1.0, 0.0};
  Vector3d p_AP{0.0,  0.5, 0.0};
  Vector3d n_PQ_W{0.0, 1.0, 0.0};
  SurfacePoint surface_point_A = {p_WP, p_AP, n_PQ_W};

  // Sphere
  DrakeShapes::Sphere sphere{0.5};
  Isometry3d X_WB;
  X_WB.setIdentity();
  X_WB.translation() = Vector3d(0.0, 1.25, 0.0);
  Vector3d p_WQ{0.0,  0.75, 0.0};
  Vector3d p_BQ{0.0,  -0.5, 0.0};
  Vector3d n_QP_W{0.0, -1.0, 0.0};
  SurfacePoint surface_point_B = {p_WQ, p_BQ, n_QP_W};

  vector<ShapeVsShapeTestParam> params;
  params.push_back(ShapeVsShapeTestParam(box, sphere, X_WA, X_WB,
        surface_point_A, surface_point_B, fcl::GST_INDEP));
  params.push_back(ShapeVsShapeTestParam(box, sphere, X_WA, X_WB,
        surface_point_A, surface_point_B, fcl::GST_LIBCCD));
  params.push_back(ShapeVsShapeTestParam(sphere, box, X_WB, X_WA,
        surface_point_B, surface_point_A, fcl::GST_INDEP));
  params.push_back(ShapeVsShapeTestParam(sphere, box, X_WB, X_WA,
        surface_point_B, surface_point_A, fcl::GST_LIBCCD));
  
  return params;
}

INSTANTIATE_TEST_CASE_P(BoxVsSphere, ShapeVsShapeTest, ::testing::ValuesIn(generateBoxVsSphereParam()));

// A sphere of diameter 1.0 is placed  above a box.  The sphere overlaps with
// the box with its deepest penetration point (the bottom) 0.25 units into the
// box (negative distance). Only one contact point is expected when colliding
// with a sphere.
vector<ShapeVsShapeTestParam> generateBoxVsBoxParam() {
  // First box
  DrakeShapes::Box box_A{Vector3d(1, 1, 1)};
  Isometry3d X_WA;
  X_WA.setIdentity();
  X_WA.translation() = Vector3d(0, 0.5, 0);
  Vector3d p_WP{0.0,  1.0, 0.0};
  Vector3d p_AP{0.0,  0.5, 0.0};
  Vector3d n_PQ_W{0.0, 1.0, 0.0};
  SurfacePoint surface_point_A = {p_WP, p_AP, n_PQ_W};

  // Second box
  DrakeShapes::Box box_B{1.0/std::sqrt(3)*Vector3d(1, 1, 1)};
  Isometry3d X_WB;
  X_WB.setIdentity();
  X_WB.rotate(Eigen::AngleAxisd(M_PI_4, Vector3d(M_SQRT1_2,0,M_SQRT1_2)));
  X_WB.translation() = Vector3d(0.0, 1.25, 0.0);
  Vector3d p_WQ{0.0,  0.75, 0.0};
  Vector3d p_BQ{0.0,  -0.5, 0.0};
  Vector3d n_QP_W{0.0, -1.0, 0.0};
  SurfacePoint surface_point_B = {p_WQ, p_BQ, n_QP_W};

  vector<ShapeVsShapeTestParam> params;
  params.push_back(ShapeVsShapeTestParam(box_A, box_B, X_WA, X_WB,
        surface_point_A, surface_point_B, fcl::GST_INDEP));
  params.push_back(ShapeVsShapeTestParam(box_A, box_B, X_WA, X_WB,
        surface_point_A, surface_point_B, fcl::GST_LIBCCD));
  params.push_back(ShapeVsShapeTestParam(box_B, box_A, X_WB, X_WA,
        surface_point_B, surface_point_A, fcl::GST_INDEP));
  params.push_back(ShapeVsShapeTestParam(box_B, box_A, X_WB, X_WA,
        surface_point_B, surface_point_A, fcl::GST_LIBCCD));
  
  return params;
}

INSTANTIATE_TEST_CASE_P(BoxVsBox, ShapeVsShapeTest, ::testing::ValuesIn(generateBoxVsBoxParam()));

}  // namespace
}  // namespace DrakeCollision
