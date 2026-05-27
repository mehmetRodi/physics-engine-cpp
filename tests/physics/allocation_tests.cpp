#include <gtest/gtest.h>

#include "physics/World.hpp"

#include <cstddef>
#include <cstdlib>
#include <new>

namespace {
bool g_trackAllocations = false;
std::size_t g_allocationCount = 0;

void resetAllocationCount() {
  g_allocationCount = 0;
}

std::size_t allocationCount() {
  return g_allocationCount;
}

void setAllocationTracking(bool enabled) {
  g_trackAllocations = enabled;
}
} // namespace

void* operator new(std::size_t size) {
  if (g_trackAllocations) {
    ++g_allocationCount;
  }

  if (void* ptr = std::malloc(size)) {
    return ptr;
  }

  throw std::bad_alloc();
}

void operator delete(void* ptr) noexcept {
  std::free(ptr);
}

void operator delete(void* ptr, std::size_t) noexcept {
  std::free(ptr);
}

void* operator new[](std::size_t size) {
  if (g_trackAllocations) {
    ++g_allocationCount;
  }

  if (void* ptr = std::malloc(size)) {
    return ptr;
  }

  throw std::bad_alloc();
}

void operator delete[](void* ptr) noexcept {
  std::free(ptr);
}

void operator delete[](void* ptr, std::size_t) noexcept {
  std::free(ptr);
}

TEST(WorldAllocationTests, ReservedWorldStepDoesNotAllocate) {
  constexpr std::size_t bodyCount = 16;
  constexpr float dt = 1.0f / 60.0f;

  World world(Vec3(0.f, -9.8f, 0.f));
  world.reserveRigidBodies(bodyCount);

  for (std::size_t i = 0; i < bodyCount; ++i) {
    const World::RigidBodyId body = world.createRigidBody(1.0f, 0.75f);

    const float x = static_cast<float>(i % 4) * 0.5f;
    const float y = static_cast<float>(i / 4) * 0.5f;

    world.rigidBody(body).position = Vec3(x, y, 0.f);
    world.rigidBody(body).velocity = Vec3(0.1f, -0.2f, 0.f);
  }

  resetAllocationCount();
  setAllocationTracking(true);

  world.step(dt);

  setAllocationTracking(false);
  const std::size_t allocationsDuringStep = allocationCount();

  EXPECT_EQ(allocationsDuringStep, 0u);
}
