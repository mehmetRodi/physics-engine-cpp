#pragma once

#include "math/Vec3.hpp"

struct AABB {
  Vec3 min;
  Vec3 max;

  bool overlaps(const AABB& other) const;
};
