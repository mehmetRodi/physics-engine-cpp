#include "physics/RigidBodySystem.hpp"

#include "collision/AABB.hpp"
#include "math/Vec3.hpp"
#include "physics/RigidBody.hpp"

#include <algorithm>

namespace {
bool buildSphereSphereContact(RigidBodySystem::RigidBodyId aId, const RigidBody& a,
                              RigidBodySystem::RigidBodyId bId, const RigidBody& b,
                              RigidBodyContact& outContact) {
  const Vec3 offset = a.position - b.position;
  const float distanceSq = offset.lengthSq();

  if (distanceSq < 1e-12f) {
    return false;
  }

  const float distance = offset.length();
  const float penetration = a.shape.sphereRadius + b.shape.sphereRadius - distance;

  if (penetration <= 0.0f) {
    return false;
  }

  outContact = {
      aId,
      bId,
      offset / distance,
      penetration,
      std::max(a.material.restitution, b.material.restitution),
  };
  return true;
}

bool buildContactForShapes(RigidBodySystem::RigidBodyId aId, const RigidBody& a,
                           RigidBodySystem::RigidBodyId bId, const RigidBody& b,
                           RigidBodyContact& outContact) {
  if (a.shape.type == ShapeType::Sphere && b.shape.type == ShapeType::Sphere) {
    return buildSphereSphereContact(aId, a, bId, b, outContact);
  }

  return false;
}
} // namespace

RigidBodySystem::RigidBodyId RigidBodySystem::createRigidBody(float mass, float radius) {
  RigidBodyId id = m_bodies.size();
  m_bodies.emplace_back(mass, radius);
  return id;
}

RigidBody& RigidBodySystem::rigidBody(RigidBodyId id) {
  return m_bodies[id];
}

const RigidBody& RigidBodySystem::rigidBody(RigidBodyId id) const {
  return m_bodies[id];
}

void RigidBodySystem::reserveRigidBodies(std::size_t capacity) {
  m_bodies.reserve(capacity);
  m_aabbProxies.reserve(capacity);
  const std::size_t pairCapacity = capacity * (capacity - 1) / 2;
  m_aabbPairs.reserve(pairCapacity);
  m_aabbSweepScratch.reserve(capacity);
  m_contacts.reserve(pairCapacity);
}

RigidBodyCollisionPipelineStats RigidBodySystem::collisionPipelineStats() const {
  return {
      m_bodies.size(),
      m_aabbPairs.size(),
      m_contacts.size(),
  };
}

void RigidBodySystem::step(float dt, const Vec3& gravity) {
  for (RigidBody& body : m_bodies) {
    body.applyForce(gravity * body.mass);
    body.update(dt);
  }
  resolveCollisions();
}

void RigidBodySystem::buildAABBProxies() {
  m_aabbProxies.clear();
  m_aabbProxies.reserve(m_bodies.size());

  for (std::size_t i = 0; i < m_bodies.size(); ++i) {
    const RigidBody& body = m_bodies[i];
    m_aabbProxies.push_back({i, makeAABBForSphere(body.position, body.shape.sphereRadius)});
  }
}

void RigidBodySystem::resolveCollisions() {
  buildAABBProxies();
  findAABBPairsSweepAndPrune(m_aabbProxies, m_aabbSweepScratch, m_aabbPairs);
  buildContacts();

  for (const RigidBodyContact& contact : m_contacts) {
    resolveContact(contact);
  }
}

void RigidBodySystem::resolveContact(const RigidBodyContact& contact) {
  RigidBody& body1 = m_bodies[contact.a];
  RigidBody& body2 = m_bodies[contact.b];

  const float inverseMassSum = body1.invMass + body2.invMass;
  if (inverseMassSum <= 0.f) {
    return;
  }

  const Vec3 correction = contact.normal * (contact.penetration / inverseMassSum);
  body1.position += correction * body1.invMass;
  body2.position -= correction * body2.invMass;

  Vec3 relativeVelocity = body1.velocity - body2.velocity;
  float closingSpeed = relativeVelocity.dot(contact.normal);

  if (closingSpeed >= 0.f) {
    return;
  }

  float impulseMagnitude = -(1.f + contact.restitution) * closingSpeed / inverseMassSum;

  body1.velocity += contact.normal * impulseMagnitude * body1.invMass;
  body2.velocity -= contact.normal * impulseMagnitude * body2.invMass;
}

void RigidBodySystem::buildContacts() {
  m_contacts.clear();

  for (const AABBPair& pair : m_aabbPairs) {
    const RigidBody& body1 = m_bodies[pair.a];
    const RigidBody& body2 = m_bodies[pair.b];

    RigidBodyContact contact{};
    if (buildContactForShapes(pair.a, body1, pair.b, body2, contact)) {
      m_contacts.push_back(contact);
    }
  }
}
