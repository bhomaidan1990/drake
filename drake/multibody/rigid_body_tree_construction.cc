#include "drake/multibody/rigid_body_tree_construction.h"

namespace drake {
namespace multibody {

void AddFlatTerrainToWorld(RigidBodyTreed* tree, double box_size,
                           double box_depth) {
  Eigen::Vector3d sides(box_size, box_size, box_depth);
  Eigen::Isometry3d T_element_to_link = Eigen::Isometry3d::Identity();
  T_element_to_link.translation() << 0, 0,
      -box_depth / 2;  // Top of the box is at z = 0.
  AddBoxToWorld(T_element_to_link, sides, "terrain", tree);
}

void AddBoxToWorld(const Isometry3<double>& X_WO, const Vector3<double>& sides,
                   const std::string& name, RigidBodyTree<double>* tree) {
  DrakeShapes::Box geom(sides);
  RigidBody<double>& world = tree->world();

  // Defines a color called "desert sand" according to htmlcsscolor.com.
  Eigen::Vector4d color;
  color << 0.9297, 0.7930, 0.6758, 1;

  world.AddVisualElement(
      DrakeShapes::VisualElement(geom, X_WO, color));
  tree->addCollisionElement(
      DrakeCollision::Element(geom, X_WO, &world), world, name);
  tree->compile();
}

}  // namespace multibody
}  // namespace drake
