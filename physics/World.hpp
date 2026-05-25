#pragma once
#include "math/Vec3.hpp"
#include "physics/RigidBody.hpp"
#include <cstddef>
#include <vector>
class World {
public:
  using BodyId = std::size_t;

  World(Vec3 gravity);
  World(World &&) = default;
  World(const World &) = delete;
  World &operator=(World &&) = default;
  World &operator=(const World &) = delete;
  ~World() = default;

  BodyId createBody(float mass, float radius);
  RigidBody &body(BodyId id);
  const RigidBody &body(BodyId id) const;
  void reserveBodies(std::size_t capacity);

  void step(float dt);

private:
  void resolveCollisions();
  void resolveContact(RigidBody &body1, RigidBody &body2);

  Vec3 m_gravity;

  // World owns bodies in deterministic vector order. BodyId values are stable
  // while bodies are only appended and never removed.
  std::vector<RigidBody> m_bodies;
};
