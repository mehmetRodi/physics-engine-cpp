#pragma once
#include "math/Vec3.hpp"
#include <cstddef>
#include <vector>

struct SphereProxy {
  std::size_t id;
  Vec3 position;
  float radius;
};

struct CollisionPair {
  std::size_t a;
  std::size_t b;
};

std::vector<CollisionPair>
findSpherePairs(const std::vector<SphereProxy> &spheres);
