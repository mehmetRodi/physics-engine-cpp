#include "collision/SpherePair.hpp"

std::vector<CollisionPair>
findSpherePairs(const std::vector<SphereProxy> &spheres) {
  std::vector<CollisionPair> pairs;

  for (std::size_t i = 0; i < spheres.size(); ++i) {
    const SphereProxy &a = spheres[i];
    for (std::size_t j = i + 1; j < spheres.size(); ++j) {
      const SphereProxy &b = spheres[j];

      const float radiusSum = a.radius + b.radius;
      const Vec3 offset = a.position - b.position;

      if (offset.lengthSq() < radiusSum * radiusSum) {
        pairs.push_back({a.id, b.id});
      }
    }
  }

  return pairs;
}
