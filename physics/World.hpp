#pragma once
#include "math/Vec3.hpp"

#include "physics/RigidBodySystem.hpp"

#include <cstddef>
class World {
public:
  using RigidBodyId = RigidBodySystem::RigidBodyId;

  World(Vec3 gravity);
  World(World&&) = default;
  World(const World&) = delete;
  World& operator=(World&&) = default;
  World& operator=(const World&) = delete;
  ~World() = default;

  RigidBodyId createRigidBody(float mass, float radius);
  RigidBody& rigidBody(RigidBodyId id);
  const RigidBody& rigidBody(RigidBodyId id) const;
  void reserveRigidBodies(std::size_t capacity);

  // Advances one deterministic simulation tick. Callers own frame-time
  // accumulation and should pass a fixed dt for replayable simulation.
  void step(float dt);

private:
  Vec3 m_gravity;

  RigidBodySystem m_rigidBodySystem;
};
