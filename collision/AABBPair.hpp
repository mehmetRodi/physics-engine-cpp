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

void findAABBPairs(const std::vector<AABBProxy>& proxies,
                   std::vector<AABBPair>& outPairs);

std::vector<AABBPair> findAABBPairs(const std::vector<AABBProxy>& proxies);

void findAABBPairsSweepAndPrune(const std::vector<AABBProxy>& proxies,
                                std::vector<AABBPair>& outPairs);

std::vector<AABBPair>
findAABBPairsSweepAndPrune(const std::vector<AABBProxy>& proxies);
