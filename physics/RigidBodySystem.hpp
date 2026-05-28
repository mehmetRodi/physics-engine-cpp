#pragma once

#include "collision/AABBPair.hpp"
#include "math/Vec3.hpp"
#include "physics/RigidBody.hpp"
#include "physics/RigidBodyContact.hpp"

#include <cstddef>
#include <vector>

class RigidBodySystemTestAccess;
class RigidBodySystemInstrumentationAccess;

struct RigidBodyCollisionPipelineStats {
  std::size_t bodyCount = 0;
  std::size_t aabbCandidatePairCount = 0;
  std::size_t contactCount = 0;
};

class RigidBodySystem {
public:
  using RigidBodyId = std::size_t;

  RigidBodyId createRigidBody(float mass, float radius);
  RigidBody& rigidBody(RigidBodyId id);
  const RigidBody& rigidBody(RigidBodyId id) const;
  void reserveRigidBodies(std::size_t capacity);
  RigidBodyCollisionPipelineStats collisionPipelineStats() const;

  void step(float dt, const Vec3& gravity);

private:
  friend class RigidBodySystemTestAccess;
  friend class RigidBodySystemInstrumentationAccess;

  void buildAABBProxies();
  void resolveCollisions();
  void resolveContact(const RigidBodyContact& contact);
  void buildContacts();

  std::vector<RigidBody> m_bodies;
  std::vector<AABBProxy> m_aabbProxies;
  std::vector<AABBPair> m_aabbPairs;
  AABBSweepAndPruneScratch m_aabbSweepScratch;
  std::vector<RigidBodyContact> m_contacts;
};
