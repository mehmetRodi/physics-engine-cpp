#include "collision/AABB.hpp"
#include "collision/AABBPair.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string_view>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;

enum class Distribution {
  SparseGrid,
  DenseGrid,
  AllOverlapping,
};

std::string_view distributionName(Distribution distribution) {
  switch (distribution) {
  case Distribution::SparseGrid:
    return "sparse_grid";
  case Distribution::DenseGrid:
    return "dense_grid";
  case Distribution::AllOverlapping:
    return "all_overlapping";
  }

  return "unknown";
}

std::vector<AABBProxy> createAABBProxies(std::size_t bodyCount,
                                         Distribution distribution) {
  std::vector<AABBProxy> proxies;
  proxies.reserve(bodyCount);

  for (std::size_t i = 0; i < bodyCount; ++i) {
    Vec3 center(0.0f, 0.0f, 0.0f);

    if (distribution != Distribution::AllOverlapping) {
      const float spacing = distribution == Distribution::SparseGrid ? 3.0f : 0.75f;
      center = Vec3(static_cast<float>(i % 64) * spacing,
                    static_cast<float>(i / 64) * spacing,
                    0.0f);
    }

    proxies.push_back({i, makeAABBForSphere(center, 0.5f)});
  }

  return proxies;
}

std::size_t pairChecksForBodyCount(std::size_t bodyCount) {
  return bodyCount * (bodyCount - 1) / 2;
}

int iterationsForBodyCount(std::size_t bodyCount) {
  constexpr std::size_t targetPairChecks = 20'000'000;
  const std::size_t pairChecks = pairChecksForBodyCount(bodyCount);
  return std::max(1, static_cast<int>(targetPairChecks / pairChecks));
}

void runCase(std::size_t bodyCount, Distribution distribution) {
  const std::vector<AABBProxy> proxies = createAABBProxies(bodyCount, distribution);
  const std::size_t pairChecksPerIteration = pairChecksForBodyCount(bodyCount);
  const int iterations = iterationsForBodyCount(bodyCount);

  std::vector<AABBPair> pairs;
  pairs.reserve(pairChecksPerIteration);

  std::size_t checksum = 0;

  const auto start = Clock::now();

  for (int i = 0; i < iterations; ++i) {
    findAABBPairs(proxies, pairs);
    checksum += pairs.size();
  }

  const auto end = Clock::now();

  const auto totalNanoseconds =
      std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

  const std::size_t totalPairChecks = pairChecksPerIteration * iterations;
  const double nanosecondsPerPairCheck = static_cast<double>(totalNanoseconds) /
                                         static_cast<double>(totalPairChecks);

  std::cout << "case: " << distributionName(distribution) << '\n';
  std::cout << "body_count: " << bodyCount << '\n';
  std::cout << "iterations: " << iterations << '\n';
  std::cout << "pair_checks_per_iteration: " << pairChecksPerIteration << '\n';
  std::cout << "total_pair_checks: " << totalPairChecks << '\n';
  std::cout << "last_candidate_pairs: " << pairs.size() << '\n';
  std::cout << "total_ns: " << totalNanoseconds << '\n';
  std::cout << "ns_per_pair_check: " << nanosecondsPerPairCheck << '\n';
  std::cout << "checksum: " << checksum << '\n';
  std::cout << '\n';
}
} // namespace

int main() {
  constexpr std::array<std::size_t, 3> bodyCounts = {128, 512, 1024};
  constexpr std::array<Distribution, 3> distributions = {
      Distribution::SparseGrid,
      Distribution::DenseGrid,
      Distribution::AllOverlapping,
  };

  std::cout << "AABB broadphase baseline benchmark\n";
  std::cout << "method: deterministic O(n^2) all-pairs AABB overlap scan\n\n";

  for (const std::size_t bodyCount : bodyCounts) {
    for (const Distribution distribution : distributions) {
      runCase(bodyCount, distribution);
    }
  }

  return 0;
}
