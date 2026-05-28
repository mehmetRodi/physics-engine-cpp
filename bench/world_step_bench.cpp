#include "physics/World.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string_view>
#include <vector>

namespace {
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

World createWorld(std::size_t bodyCount, Distribution distribution) {
  World world(Vec3(0.f, -9.8f, 0.f));
  world.reserveRigidBodies(bodyCount);

  for (std::size_t i = 0; i < bodyCount; ++i) {
    const World::RigidBodyId body = world.createRigidBody(0.0f, 0.25f);

    Vec3 position(0.f, 0.f, 0.f);

    if (distribution != Distribution::AllOverlapping) {
      const float spacing = distribution == Distribution::SparseGrid ? 2.0f : 0.4f;
      position = Vec3(static_cast<float>(i % 64) * spacing,
                      static_cast<float>(i / 64) * spacing,
                      0.f);
    }

    world.rigidBody(body).position = position;
    world.rigidBody(body).velocity = Vec3(0.f, 0.f, 0.f);
  }

  return world;
}

double averageNanoseconds(const std::vector<std::uint64_t>& samples) {
  std::uint64_t total = 0;

  for (const std::uint64_t sample : samples) {
    total += sample;
  }

  return static_cast<double>(total) / static_cast<double>(samples.size());
}

std::uint64_t percentileNanoseconds(const std::vector<std::uint64_t>& samples, double percentile) {
  const std::size_t index =
      static_cast<std::size_t>(percentile * static_cast<double>(samples.size() - 1));
  return samples[index];
}

void runCase(std::size_t bodyCount, Distribution distribution, int warmupSteps,
             int measuredSteps, float dt) {
  World world = createWorld(bodyCount, distribution);

  for (int i = 0; i < warmupSteps; ++i) {
    world.step(dt);
  }

  std::vector<std::uint64_t> samples;
  samples.reserve(measuredSteps);

  for (int i = 0; i < measuredSteps; ++i) {
    const auto start = std::chrono::steady_clock::now();

    world.step(dt);

    const auto end = std::chrono::steady_clock::now();

    const auto elapsedNanoseconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

    samples.push_back(static_cast<std::uint64_t>(elapsedNanoseconds));
  }

  std::sort(samples.begin(), samples.end());

  const double average = averageNanoseconds(samples);
  const std::uint64_t p95 = percentileNanoseconds(samples, 0.95);
  const std::uint64_t p99 = percentileNanoseconds(samples, 0.99);

  std::cout << "World::step benchmark\n";
  std::cout << "distribution: " << distributionName(distribution) << '\n';
  std::cout << "body_count: " << bodyCount << '\n';
  std::cout << "warmup_steps: " << warmupSteps << '\n';
  std::cout << "measured_steps: " << measuredSteps << '\n';
  std::cout << "avg_ns: " << average << '\n';
  std::cout << "p95_ns: " << p95 << '\n';
  std::cout << "p99_ns: " << p99 << '\n';
  std::cout << "max_ns: " << samples.back() << '\n';
  std::cout << '\n';
}
} // namespace

int main() {
  constexpr std::size_t bodyCount = 1024;
  constexpr float dt = 1.0f / 60.0f;
  constexpr int warmupSteps = 100;
  constexpr int measuredSteps = 1000;
  constexpr std::array<Distribution, 3> distributions = {
      Distribution::SparseGrid,
      Distribution::DenseGrid,
      Distribution::AllOverlapping,
  };

  for (const Distribution distribution : distributions) {
    runCase(bodyCount, distribution, warmupSteps, measuredSteps, dt);
  }

  return 0;
}
