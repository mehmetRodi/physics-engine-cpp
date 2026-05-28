#include "physics/RigidBodySystem.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string_view>

class RigidBodySystemInstrumentationAccess {
public:
  static void buildAABBProxies(RigidBodySystem& system) {
    system.buildAABBProxies();
  }

  static void findAABBPairs(RigidBodySystem& system) {
    findAABBPairsSweepAndPrune(system.m_aabbProxies, system.m_aabbSweepScratch,
                               system.m_aabbPairs);
  }

  static void buildContacts(RigidBodySystem& system) {
    system.buildContacts();
  }

  static void resolveContacts(RigidBodySystem& system) {
    for (const RigidBodyContact& contact : system.m_contacts) {
      system.resolveContact(contact);
    }
  }
};

namespace {
using Clock = std::chrono::steady_clock;

enum class Distribution {
  SparseGrid,
  DenseGrid,
  AllOverlapping,
};

struct CollisionPipelineTimingTotals {
  std::uint64_t aabbProxyBuildNs = 0;
  std::uint64_t broadphaseNs = 0;
  std::uint64_t contactGenerationNs = 0;
  std::uint64_t contactResolutionNs = 0;

  std::uint64_t totalNs() const {
    return aabbProxyBuildNs + broadphaseNs + contactGenerationNs + contactResolutionNs;
  }
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

RigidBodySystem createRigidBodySystem(std::size_t bodyCount, Distribution distribution) {
  RigidBodySystem system;
  system.reserveRigidBodies(bodyCount);

  for (std::size_t i = 0; i < bodyCount; ++i) {
    const RigidBodySystem::RigidBodyId body = system.createRigidBody(0.0f, 0.25f);

    Vec3 position(0.f, 0.f, 0.f);

    if (distribution != Distribution::AllOverlapping) {
      const float spacing = distribution == Distribution::SparseGrid ? 2.0f : 0.4f;
      position = Vec3(static_cast<float>(i % 64) * spacing,
                      static_cast<float>(i / 64) * spacing,
                      0.f);
    }

    system.rigidBody(body).position = position;
    system.rigidBody(body).velocity = Vec3(0.f, 0.f, 0.f);
  }

  return system;
}

template <typename Function>
std::uint64_t measureNanoseconds(Function&& function) {
  const auto start = Clock::now();
  function();
  const auto end = Clock::now();

  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count());
}

void runCollisionPipeline(RigidBodySystem& system) {
  RigidBodySystemInstrumentationAccess::buildAABBProxies(system);
  RigidBodySystemInstrumentationAccess::findAABBPairs(system);
  RigidBodySystemInstrumentationAccess::buildContacts(system);
  RigidBodySystemInstrumentationAccess::resolveContacts(system);
}

void runCase(std::size_t bodyCount, Distribution distribution, int warmupSteps,
             int measuredSteps) {
  RigidBodySystem system = createRigidBodySystem(bodyCount, distribution);

  for (int i = 0; i < warmupSteps; ++i) {
    runCollisionPipeline(system);
  }

  CollisionPipelineTimingTotals timings;
  RigidBodyCollisionPipelineStats stats;

  for (int i = 0; i < measuredSteps; ++i) {
    timings.aabbProxyBuildNs += measureNanoseconds([&system]() {
      RigidBodySystemInstrumentationAccess::buildAABBProxies(system);
    });
    timings.broadphaseNs += measureNanoseconds([&system]() {
      RigidBodySystemInstrumentationAccess::findAABBPairs(system);
    });
    timings.contactGenerationNs += measureNanoseconds([&system]() {
      RigidBodySystemInstrumentationAccess::buildContacts(system);
    });
    timings.contactResolutionNs += measureNanoseconds([&system]() {
      RigidBodySystemInstrumentationAccess::resolveContacts(system);
    });

    stats = system.collisionPipelineStats();
  }

  const double iterations = static_cast<double>(measuredSteps);

  std::cout << "Collision pipeline benchmark\n";
  std::cout << "distribution: " << distributionName(distribution) << '\n';
  std::cout << "body_count: " << bodyCount << '\n';
  std::cout << "warmup_iterations: " << warmupSteps << '\n';
  std::cout << "measured_iterations: " << measuredSteps << '\n';
  std::cout << "last_stats_body_count: " << stats.bodyCount << '\n';
  std::cout << "last_aabb_candidate_pairs: " << stats.aabbCandidatePairCount << '\n';
  std::cout << "last_contacts: " << stats.contactCount << '\n';
  std::cout << "avg_aabb_proxy_build_ns: "
            << static_cast<double>(timings.aabbProxyBuildNs) / iterations << '\n';
  std::cout << "avg_broadphase_ns: "
            << static_cast<double>(timings.broadphaseNs) / iterations << '\n';
  std::cout << "avg_contact_generation_ns: "
            << static_cast<double>(timings.contactGenerationNs) / iterations << '\n';
  std::cout << "avg_contact_resolution_ns: "
            << static_cast<double>(timings.contactResolutionNs) / iterations << '\n';
  std::cout << "avg_total_collision_pipeline_ns: "
            << static_cast<double>(timings.totalNs()) / iterations << '\n';
  std::cout << '\n';
}
} // namespace

int main() {
  constexpr std::size_t bodyCount = 1024;
  constexpr int warmupSteps = 100;
  constexpr int measuredSteps = 1000;
  constexpr std::array<Distribution, 3> distributions = {
      Distribution::SparseGrid,
      Distribution::DenseGrid,
      Distribution::AllOverlapping,
  };

  for (const Distribution distribution : distributions) {
    runCase(bodyCount, distribution, warmupSteps, measuredSteps);
  }

  return 0;
}
