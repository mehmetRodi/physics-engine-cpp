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
  const std::size_t pairCapacity = capacity * (capacity - 1) / 2;
  m_collisionPairs.reserve(pairCapacity);
  m_contacts.reserve(pairCapacity);
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

  for (const CollisionPair& pair : m_collisionPairs) {
    const RigidBody& body1 = m_bodies[pair.a];
    const RigidBody& body2 = m_bodies[pair.b];

    const Vec3 offset = body1.position - body2.position;
    const float distanceSq = offset.lengthSq();

    if (distanceSq < 1e-12f) {
      continue;
    }

    const float distance = offset.length();
    const float penetration = body1.radius + body2.radius - distance;

    if (penetration <= 0.f) {
      continue;
    }

    m_contacts.push_back({
        pair.a,
        pair.b,
        offset / distance,
        penetration,
        std::max(body1.material.restitution, body2.material.restitution),
    });
  }
}
