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
                        const std::vector<std::size_t>& proxyIndices, std::size_t begin,
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

std::size_t buildBVHNode(const std::vector<AABBProxy>& proxies, AABBBVHScratch& scratch,
                         std::size_t begin, std::size_t end) {
  if (end - begin == 1) {
    const std::size_t proxyIndex = scratch.proxyIndices[begin];
    const std::size_t nodeIndex = scratch.nodes.size();
    scratch.nodes.push_back({proxies[proxyIndex].bounds, 0, 0, proxyIndex, true});
    return nodeIndex;
  }

  const int axis = longestCentroidAxis(proxies, scratch.proxyIndices, begin, end);
  const std::size_t mid = begin + (end - begin) / 2;

  std::nth_element(scratch.proxyIndices.begin() + static_cast<std::ptrdiff_t>(begin),
                   scratch.proxyIndices.begin() + static_cast<std::ptrdiff_t>(mid),
                   scratch.proxyIndices.begin() + static_cast<std::ptrdiff_t>(end),
                   [&proxies, axis](std::size_t lhs, std::size_t rhs) {
                     const float lhsCentroid = centroidComponent(proxies[lhs], axis);
                     const float rhsCentroid = centroidComponent(proxies[rhs], axis);

                     if (lhsCentroid != rhsCentroid) {
                       return lhsCentroid < rhsCentroid;
                     }

                     return lhs < rhs;
                   });

  const std::size_t leftChild = buildBVHNode(proxies, scratch, begin, mid);
  const std::size_t rightChild = buildBVHNode(proxies, scratch, mid, end);

  const std::size_t nodeIndex = scratch.nodes.size();
  scratch.nodes.push_back({
      mergeAABBs(scratch.nodes[leftChild].bounds, scratch.nodes[rightChild].bounds),
      leftChild,
      rightChild,
      0,
      false,
  });
  return nodeIndex;
}

void emitOverlappingLeafPairs(const AABBBVHScratch& scratch, std::size_t leftNodeIndex,
                              std::size_t rightNodeIndex,
                              std::vector<AABBPair>& outProxyIndexPairs) {
  const AABBBVHNode& left = scratch.nodes[leftNodeIndex];
  const AABBBVHNode& right = scratch.nodes[rightNodeIndex];

  if (!left.bounds.overlaps(right.bounds)) {
    return;
  }

  if (left.leaf && right.leaf) {
    const std::size_t firstIndex = std::min(left.proxyIndex, right.proxyIndex);
    const std::size_t secondIndex = std::max(left.proxyIndex, right.proxyIndex);
    outProxyIndexPairs.push_back({firstIndex, secondIndex});
    return;
  }

  if (left.leaf) {
    emitOverlappingLeafPairs(scratch, leftNodeIndex, right.leftChild, outProxyIndexPairs);
    emitOverlappingLeafPairs(scratch, leftNodeIndex, right.rightChild, outProxyIndexPairs);
    return;
  }

  if (right.leaf) {
    emitOverlappingLeafPairs(scratch, left.leftChild, rightNodeIndex, outProxyIndexPairs);
    emitOverlappingLeafPairs(scratch, left.rightChild, rightNodeIndex, outProxyIndexPairs);
    return;
  }

  emitOverlappingLeafPairs(scratch, left.leftChild, right.leftChild, outProxyIndexPairs);
  emitOverlappingLeafPairs(scratch, left.leftChild, right.rightChild, outProxyIndexPairs);
  emitOverlappingLeafPairs(scratch, left.rightChild, right.leftChild, outProxyIndexPairs);
  emitOverlappingLeafPairs(scratch, left.rightChild, right.rightChild, outProxyIndexPairs);
}

void collectBVHPairs(const AABBBVHScratch& scratch, std::size_t nodeIndex,
                     std::vector<AABBPair>& outProxyIndexPairs) {
  const AABBBVHNode& node = scratch.nodes[nodeIndex];

  if (node.leaf) {
    return;
  }

  collectBVHPairs(scratch, node.leftChild, outProxyIndexPairs);
  collectBVHPairs(scratch, node.rightChild, outProxyIndexPairs);
  emitOverlappingLeafPairs(scratch, node.leftChild, node.rightChild, outProxyIndexPairs);
}

void sortPairsByProxyIndex(std::vector<AABBPair>& pairs) {
  std::sort(pairs.begin(), pairs.end(), [](const AABBPair& lhs, const AABBPair& rhs) {
    if (lhs.a != rhs.a) {
      return lhs.a < rhs.a;
    }

    return lhs.b < rhs.b;
  });
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
  scratch.candidateProxyIndexPairs.clear();

  const AABBBVHBuildResult buildResult = buildAABBBVH(proxies, scratch);
  if (buildResult.empty) {
    return;
  }

  collectAABBBVHPairCandidates(scratch, buildResult.rootNode, scratch.candidateProxyIndexPairs);
  emitSortedAABBPairsFromProxyIndexPairs(proxies, scratch.candidateProxyIndexPairs, outPairs);
}

std::vector<AABBPair> findAABBPairsBVH(const std::vector<AABBProxy>& proxies) {
  std::vector<AABBPair> pairs;
  AABBBVHScratch scratch;
  findAABBPairsBVH(proxies, scratch, pairs);
  return pairs;
}

AABBBVHBuildResult buildAABBBVH(const std::vector<AABBProxy>& proxies, AABBBVHScratch& scratch) {
  scratch.proxyIndices.clear();
  scratch.nodes.clear();

  if (proxies.size() < 2) {
    return {};
  }

  scratch.proxyIndices.resize(proxies.size());
  std::iota(scratch.proxyIndices.begin(), scratch.proxyIndices.end(), std::size_t{0});

  return {buildBVHNode(proxies, scratch, 0, scratch.proxyIndices.size()), false};
}

void collectAABBBVHPairCandidates(const AABBBVHScratch& scratch, std::size_t rootNode,
                                  std::vector<AABBPair>& outProxyIndexPairs) {
  outProxyIndexPairs.clear();
  collectBVHPairs(scratch, rootNode, outProxyIndexPairs);
}

void emitSortedAABBPairsFromProxyIndexPairs(const std::vector<AABBProxy>& proxies,
                                            std::vector<AABBPair>& proxyIndexPairs,
                                            std::vector<AABBPair>& outPairs) {
  outPairs.clear();
  sortPairsByProxyIndex(proxyIndexPairs);
  outPairs.reserve(proxyIndexPairs.size());

  for (const AABBPair& candidate : proxyIndexPairs) {
    outPairs.push_back({proxies[candidate.a].id, proxies[candidate.b].id});
  }
}
