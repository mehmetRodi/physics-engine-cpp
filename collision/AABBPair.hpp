#pragma once

#include "collision/AABB.hpp"

#include <cstddef>
#include <vector>

struct AABBProxy {
  std::size_t id;
  AABB bounds;
};

struct AABBPair {
  std::size_t a;
  std::size_t b;
};

struct AABBSweepAndPruneScratch {
  std::vector<std::size_t> sortedProxyIndices;
  // Stores proxy indices so SAP can restore baseline input-order output.
  std::vector<AABBPair> candidateProxyIndexPairs;

  void reserve(std::size_t proxyCapacity);
};

void findAABBPairs(const std::vector<AABBProxy>& proxies,
                   std::vector<AABBPair>& outPairs);

std::vector<AABBPair> findAABBPairs(const std::vector<AABBProxy>& proxies);

void findAABBPairsSweepAndPrune(const std::vector<AABBProxy>& proxies,
                                std::vector<AABBPair>& outPairs);

void findAABBPairsSweepAndPrune(const std::vector<AABBProxy>& proxies,
                                AABBSweepAndPruneScratch& scratch,
                                std::vector<AABBPair>& outPairs);

std::vector<AABBPair>
findAABBPairsSweepAndPrune(const std::vector<AABBProxy>& proxies);
