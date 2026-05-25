#include "physics/World.hpp"

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

  const auto start = std::chrono::steady_clock::now();

  world.step(dt);

  const auto end = std::chrono::steady_clock::now();

  const auto elapsedNanoseconds =
      std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

  std::cout << "Created " << bodyCount << " bodies\n";
  std::cout << "one_step_ns: " << elapsedNanoseconds << '\n';
  std::cout << "Created " << bodyCount << " bodies\n";
  return 0;
}
