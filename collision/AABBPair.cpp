#include "collision/AABBPair.hpp"

void findAABBPairs(const std::vector<AABBProxy>& proxies,
                   std::vector<AABBPair>& outPairs) {
  outPairs.clear();

  for (std::size_t i = 0; i < proxies.size(); ++i) {
    const AABBProxy& a = proxies[i];

    for (std::size_t j = i + 1; j < proxies.size(); ++j) {
      const AABBProxy& b = proxies[j];

      if (a.bounds.overlaps(b.bounds)) {
        outPairs.push_back({a.id, b.id});
      }
    }
  }
}

std::vector<AABBPair> findAABBPairs(const std::vector<AABBProxy>& proxies) {
  std::vector<AABBPair> pairs;
  findAABBPairs(proxies, pairs);
  return pairs;
}
