#include "collision/AABBPair.hpp"

#include <algorithm>
#include <cstddef>
#include <numeric>
#include <vector>

namespace {
struct IndexedAABBPair {
  std::size_t firstIndex;
  std::size_t secondIndex;
  AABBPair pair;
};
} // namespace

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

void findAABBPairsSweepAndPrune(const std::vector<AABBProxy>& proxies,
                                std::vector<AABBPair>& outPairs) {
  outPairs.clear();

  if (proxies.size() < 2) {
    return;
  }

  std::vector<std::size_t> sortedIndices(proxies.size());
  std::iota(sortedIndices.begin(), sortedIndices.end(), std::size_t{0});

  std::sort(sortedIndices.begin(), sortedIndices.end(),
            [&proxies](std::size_t lhs, std::size_t rhs) {
              const float lhsMinX = proxies[lhs].bounds.min.x;
              const float rhsMinX = proxies[rhs].bounds.min.x;

              if (lhsMinX != rhsMinX) {
                return lhsMinX < rhsMinX;
              }

              return lhs < rhs;
            });

  std::vector<IndexedAABBPair> candidates;

  for (std::size_t sortedI = 0; sortedI < sortedIndices.size(); ++sortedI) {
    const std::size_t proxyI = sortedIndices[sortedI];
    const AABBProxy& a = proxies[proxyI];

    for (std::size_t sortedJ = sortedI + 1; sortedJ < sortedIndices.size(); ++sortedJ) {
      const std::size_t proxyJ = sortedIndices[sortedJ];
      const AABBProxy& b = proxies[proxyJ];

      if (b.bounds.min.x > a.bounds.max.x) {
        break;
      }

      if (a.bounds.overlaps(b.bounds)) {
        const std::size_t firstIndex = std::min(proxyI, proxyJ);
        const std::size_t secondIndex = std::max(proxyI, proxyJ);
        candidates.push_back(
            {firstIndex, secondIndex, {proxies[firstIndex].id, proxies[secondIndex].id}});
      }
    }
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const IndexedAABBPair& lhs, const IndexedAABBPair& rhs) {
              if (lhs.firstIndex != rhs.firstIndex) {
                return lhs.firstIndex < rhs.firstIndex;
              }

              return lhs.secondIndex < rhs.secondIndex;
            });

  outPairs.reserve(candidates.size());
  for (const IndexedAABBPair& candidate : candidates) {
    outPairs.push_back(candidate.pair);
  }
}

std::vector<AABBPair>
findAABBPairsSweepAndPrune(const std::vector<AABBProxy>& proxies) {
  std::vector<AABBPair> pairs;
  findAABBPairsSweepAndPrune(proxies, pairs);
  return pairs;
}
