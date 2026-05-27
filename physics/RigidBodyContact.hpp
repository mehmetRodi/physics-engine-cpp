#pragma once

#include "math/Vec3.hpp"

#include <cstddef>

struct RigidBodyContact {
  std::size_t a;
  std::size_t b;
  Vec3 normal;
  float penetration;
  float restitution;
};
