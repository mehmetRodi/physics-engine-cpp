#include "physics/RigidBodySystem.hpp"

#include "math/Vec3.hpp"
#include "physics/RigidBody.hpp"

#include <algorithm>

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
  m_sphereProxies.reserve(capacity);
}

void RigidBodySystem::step(float dt, const Vec3& gravity) {
  for (RigidBody& body : m_bodies) {
    body.applyForce(gravity * body.mass);
    body.update(dt);
  }
  resolveCollisions();
}

void RigidBodySystem::buildSphereProxies() {
  m_sphereProxies.clear();
  m_sphereProxies.reserve(m_bodies.size());

  for (std::size_t i = 0; i < m_bodies.size(); ++i) {
    const RigidBody& body = m_bodies[i];
    m_sphereProxies.push_back({i, body.position, body.radius});
  }
}

void RigidBodySystem::resolveCollisions() {
  buildSphereProxies();
  findSpherePairs(m_sphereProxies, m_collisionPairs);

  for (const CollisionPair& pair : m_collisionPairs) {
    resolveContact(m_bodies[pair.a], m_bodies[pair.b]);
  }
}

void RigidBodySystem::resolveContact(RigidBody& body1, RigidBody& body2) {
  Vec3 offset = body1.position - body2.position;
  if (offset.lengthSq() < 1e-12f) {
    return;
  }

  Vec3 collisionNormal = offset.normalized();
  const float inverseMassSum = body1.invMass + body2.invMass;
  if (inverseMassSum <= 0.f) {
    return;
  }

  const float distance = offset.length();
  const float penetration = body1.radius + body2.radius - distance;
  if (penetration > 0.f) {
    const Vec3 correction = collisionNormal * (penetration / inverseMassSum);
    body1.position += correction * body1.invMass;
    body2.position -= correction * body2.invMass;
  }

  Vec3 relativeVelocity = body1.velocity - body2.velocity;
  float closingSpeed = relativeVelocity.dot(collisionNormal);

  if (closingSpeed >= 0)
    return;

  const float restitution = std::max(body1.material.restitution, body2.material.restitution);

  float impulseMagnitude = -(1.f + restitution) * (closingSpeed) / inverseMassSum;

  body1.velocity += collisionNormal * impulseMagnitude * body1.invMass;
  body2.velocity -= collisionNormal * impulseMagnitude * body2.invMass;
}
