#include "collision/AABBPair.hpp"

#include <algorithm>
#include <cstddef>
#include <numeric>
#include <vector>

namespace {
std::size_t pairCapacityForProxyCount(std::size_t proxyCount) {
  return proxyCount * (proxyCount - 1) / 2;
}

AABB mergeAABBs(const AABB& a, const AABB& b) {
  return {
      Vec3(std::min(a.min.x, b.min.x), std::min(a.min.y, b.min.y), std::min(a.min.z, b.min.z)),
      Vec3(std::max(a.max.x, b.max.x), std::max(a.max.y, b.max.y), std::max(a.max.z, b.max.z)),
  };
}

Vec3 aabbCentroid(const AABB& bounds) {
  return (bounds.min + bounds.max) * 0.5f;
}

float centroidComponent(const AABBProxy& proxy, int axis) {
  const Vec3 centroid = aabbCentroid(proxy.bounds);

  if (axis == 0) {
    return centroid.x;
  }

  if (axis == 1) {
    return centroid.y;
  }

  return centroid.z;
}

int longestCentroidAxis(const std::vector<AABBProxy>& proxies,
                        const std::vector<std::size_t>& proxyIndices,
                        std::size_t begin,
                        std::size_t end) {
  Vec3 minCentroid = aabbCentroid(proxies[proxyIndices[begin]].bounds);
  Vec3 maxCentroid = minCentroid;

  for (std::size_t i = begin + 1; i < end; ++i) {
    const Vec3 centroid = aabbCentroid(proxies[proxyIndices[i]].bounds);

    minCentroid.x = std::min(minCentroid.x, centroid.x);
    minCentroid.y = std::min(minCentroid.y, centroid.y);
    minCentroid.z = std::min(minCentroid.z, centroid.z);

    maxCentroid.x = std::max(maxCentroid.x, centroid.x);
    maxCentroid.y = std::max(maxCentroid.y, centroid.y);
    maxCentroid.z = std::max(maxCentroid.z, centroid.z);
  }

  const Vec3 extent = maxCentroid - minCentroid;

  if (extent.x >= extent.y && extent.x >= extent.z) {
    return 0;
  }

  if (extent.y >= extent.z) {
    return 1;
  }

  return 2;
}
} // namespace

void AABBSweepAndPruneScratch::reserve(std::size_t proxyCapacity) {
  sortedProxyIndices.reserve(proxyCapacity);
  candidateProxyIndexPairs.reserve(pairCapacityForProxyCount(proxyCapacity));
}

void AABBBVHScratch::reserve(std::size_t proxyCapacity) {
  proxyIndices.reserve(proxyCapacity);
  nodes.reserve(proxyCapacity == 0 ? 0 : proxyCapacity * 2 - 1);
  candidateProxyIndexPairs.reserve(pairCapacityForProxyCount(proxyCapacity));
}

void findAABBPairs(const std::vector<AABBProxy>& proxies, std::vector<AABBPair>& outPairs) {
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
  AABBSweepAndPruneScratch scratch;
  findAABBPairsSweepAndPrune(proxies, scratch, outPairs);
}

void findAABBPairsSweepAndPrune(const std::vector<AABBProxy>& proxies,
                                AABBSweepAndPruneScratch& scratch,
                                std::vector<AABBPair>& outPairs) {
  outPairs.clear();
  scratch.sortedProxyIndices.clear();
  scratch.candidateProxyIndexPairs.clear();

  if (proxies.size() < 2) {
    return;
  }

  scratch.sortedProxyIndices.resize(proxies.size());
  std::iota(scratch.sortedProxyIndices.begin(), scratch.sortedProxyIndices.end(), std::size_t{0});

  std::sort(scratch.sortedProxyIndices.begin(), scratch.sortedProxyIndices.end(),
            [&proxies](std::size_t lhs, std::size_t rhs) {
              const float lhsMinX = proxies[lhs].bounds.min.x;
              const float rhsMinX = proxies[rhs].bounds.min.x;

              if (lhsMinX != rhsMinX) {
                return lhsMinX < rhsMinX;
              }

              return lhs < rhs;
            });

  for (std::size_t sortedI = 0; sortedI < scratch.sortedProxyIndices.size(); ++sortedI) {
    const std::size_t proxyI = scratch.sortedProxyIndices[sortedI];
    const AABBProxy& a = proxies[proxyI];

    for (std::size_t sortedJ = sortedI + 1; sortedJ < scratch.sortedProxyIndices.size();
         ++sortedJ) {
      const std::size_t proxyJ = scratch.sortedProxyIndices[sortedJ];
      const AABBProxy& b = proxies[proxyJ];

      if (b.bounds.min.x > a.bounds.max.x) {
        break;
      }

      if (a.bounds.overlaps(b.bounds)) {
        const std::size_t firstIndex = std::min(proxyI, proxyJ);
        const std::size_t secondIndex = std::max(proxyI, proxyJ);
        scratch.candidateProxyIndexPairs.push_back({firstIndex, secondIndex});
      }
    }
  }

  std::sort(scratch.candidateProxyIndexPairs.begin(), scratch.candidateProxyIndexPairs.end(),
            [](const AABBPair& lhs, const AABBPair& rhs) {
              if (lhs.a != rhs.a) {
                return lhs.a < rhs.a;
              }

              return lhs.b < rhs.b;
            });

  outPairs.reserve(scratch.candidateProxyIndexPairs.size());
  for (const AABBPair& candidate : scratch.candidateProxyIndexPairs) {
    outPairs.push_back({proxies[candidate.a].id, proxies[candidate.b].id});
  }
}

std::vector<AABBPair> findAABBPairsSweepAndPrune(const std::vector<AABBProxy>& proxies) {
  std::vector<AABBPair> pairs;
  findAABBPairsSweepAndPrune(proxies, pairs);
  return pairs;
}

void findAABBPairsBVH(const std::vector<AABBProxy>& proxies, AABBBVHScratch& scratch,
                      std::vector<AABBPair>& outPairs) {
  outPairs.clear();
  scratch.proxyIndices.clear();
  scratch.nodes.clear();
  scratch.candidateProxyIndexPairs.clear();

  if (proxies.size() < 2) {
    return;
  }

  if (proxies.size() == 2) {
    scratch.nodes.push_back({proxies[0].bounds, 0, 0, 0, true});
    scratch.nodes.push_back({proxies[1].bounds, 0, 0, 1, true});
    scratch.nodes.push_back({mergeAABBs(proxies[0].bounds, proxies[1].bounds), 0, 1, 0, false});

    if (scratch.nodes[0].bounds.overlaps(scratch.nodes[1].bounds)) {
      outPairs.push_back(
          {proxies[scratch.nodes[0].proxyIndex].id, proxies[scratch.nodes[1].proxyIndex].id});
    }
    return;
  }
}

std::vector<AABBPair> findAABBPairsBVH(const std::vector<AABBProxy>& proxies) {
  std::vector<AABBPair> pairs;
  AABBBVHScratch scratch;
  findAABBPairsBVH(proxies, scratch, pairs);
  return pairs;
}
