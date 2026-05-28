#include "physics/World.hpp"

#include <array>
#include <cstddef>
#include <iostream>
#include <string_view>

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

void runCase(std::size_t bodyCount, Distribution distribution, int warmupSteps,
             int measuredSteps, float dt) {
  World world = createWorld(bodyCount, distribution);
  (void)world;

  std::cout << "Collision pipeline benchmark\n";
  std::cout << "distribution: " << distributionName(distribution) << '\n';
  std::cout << "body_count: " << bodyCount << '\n';
  std::cout << "warmup_steps: " << warmupSteps << '\n';
  std::cout << "measured_steps: " << measuredSteps << '\n';
  std::cout << "dt: " << dt << '\n';
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
