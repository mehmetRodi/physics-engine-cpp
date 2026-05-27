#pragma once

#include "collision/SpherePair.hpp"
#include "math/Vec3.hpp"
#include "physics/RigidBody.hpp"
#include "physics/RigidBodyContact.hpp"

#include <cstddef>
#include <vector>

class RigidBodySystemTestAccess;

class RigidBodySystem {
public:
  using RigidBodyId = std::size_t;

  RigidBodyId createRigidBody(float mass, float radius);
  RigidBody& rigidBody(RigidBodyId id);
  const RigidBody& rigidBody(RigidBodyId id) const;
  void reserveRigidBodies(std::size_t capacity);

  void step(float dt, const Vec3& gravity);

private:
  friend class RigidBodySystemTestAccess;

  void buildSphereProxies();
  void resolveCollisions();
  void resolveContact(const RigidBodyContact& contact);
  void buildContacts();

  std::vector<RigidBody> m_bodies;
  std::vector<SphereProxy> m_sphereProxies;
  std::vector<CollisionPair> m_collisionPairs;
  std::vector<RigidBodyContact> m_contacts;
};
