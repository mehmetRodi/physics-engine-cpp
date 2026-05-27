#pragma once
#include "collision/SpherePair.hpp"
#include "math/Vec3.hpp"
#include "physics/RigidBody.hpp"
#include <cstddef>
#include <vector>
class World {
public:
  using RigidBodyId = std::size_t;

  World(Vec3 gravity);
  World(World&&) = default;
  World(const World&) = delete;
  World& operator=(World&&) = default;
  World& operator=(const World&) = delete;
  ~World() = default;

  RigidBodyId createRigidBody(float mass, float radius);
  RigidBody& rigidBody(RigidBodyId id);
  const RigidBody& body(RigidBodyId id) const;
  void reserveRigidBodies(std::size_t capacity);

  void step(float dt);

private:
  void buildSphereProxies();
  void resolveCollisions();
  void resolveContact(RigidBody& body1, RigidBody& body2);

  Vec3 m_gravity;

  // World owns bodies in deterministic vector order. BodyId values are stable
  // while bodies are only appended and never removed.
  std::vector<RigidBody> m_bodies;
  std::vector<SphereProxy> m_sphereProxies;
  std::vector<CollisionPair> m_collisionPairs;
};
