#include "physics/World.hpp"
#include <algorithm>
#include <cstdint>
#include <vector>

#include <chrono>
#include <cstddef>
#include <iostream>

int main() {
  constexpr std::size_t bodyCount = 1024;
  constexpr float dt = 1.0f / 60.0f;

  World world(Vec3(0.f, -9.8f, 0.f));
  world.reserveBodies(bodyCount);

  for (std::size_t i = 0; i < bodyCount; ++i) {
    const World::BodyId body = world.createBody(1.0f, 0.25f);

    const float x = static_cast<float>(i % 64) * 2.0f;
    const float y = static_cast<float>(i / 64) * 2.0f;

    world.body(body).position = Vec3(x, y, 0.f);
    world.body(body).velocity = Vec3(0.1f, -0.2f, 0.f);
  }

  constexpr int warmupSteps = 100;

  for (int i = 0; i < warmupSteps; ++i) {
    world.step(dt);
  }

  constexpr int measuredSteps = 1000;

  std::vector<std::uint64_t> samples;
  samples.reserve(measuredSteps);

  for (int i = 0; i < measuredSteps; ++i) {
    const auto start = std::chrono::steady_clock::now();

    world.step(dt);

    const auto end = std::chrono::steady_clock::now();

    const auto elapsedNanoseconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(end - start)
            .count();

    samples.push_back(static_cast<std::uint64_t>(elapsedNanoseconds));
  }

  std::sort(samples.begin(), samples.end());

  std::uint64_t total = 0;
  std::uint64_t max = 0;

  for (const std::uint64_t sample : samples) {
    total += sample;

    if (sample > max) {
      max = sample;
    }
  }

  const double average =
      static_cast<double>(total) / static_cast<double>(samples.size());

  const std::uint64_t p95 = samples[static_cast<std::size_t>(
      0.95 * static_cast<double>(samples.size() - 1))];

  const std::uint64_t p99 = samples[static_cast<std::size_t>(
      0.99 * static_cast<double>(samples.size() - 1))];

  std::cout << "World::step benchmark\n";
  std::cout << "body_count: " << bodyCount << '\n';
  std::cout << "warmup_steps: " << warmupSteps << '\n';
  std::cout << "measured_steps: " << measuredSteps << '\n';
  std::cout << "avg_ns: " << average << '\n';
  std::cout << "p95_ns: " << p95 << '\n';
  std::cout << "p99_ns: " << p99 << '\n';
  std::cout << "max_ns: " << samples.back() << '\n';

  return 0;
}
