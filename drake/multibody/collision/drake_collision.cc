#include "drake/multibody/collision/drake_collision.h"

#include "drake/multibody/collision/collision_filter.h"

#include "drake/multibody/collision/fcl_model.h"

using std::unique_ptr;

namespace DrakeCollision {

unique_ptr<Model> newModel() {
  return unique_ptr<Model>(new FCLModel());
}

}  // namespace DrakeCollision
