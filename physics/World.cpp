#include "World.hpp"
#include "math/Vec3.hpp"
#include "physics/RigidBody.hpp"
World::World(Vec3 gravity) : m_gravity(gravity) {}

World::BodyId World::createBody(float mass, float radius) {
  BodyId id = m_bodies.size();
  m_bodies.emplace_back(mass, radius);
  return id;
}

RigidBody &World::body(BodyId id) { return m_bodies[id]; }

const RigidBody &World::body(BodyId id) const { return m_bodies[id]; }

void World::reserveBodies(std::size_t capacity) { m_bodies.reserve(capacity); }

void World::step(float dt) {
  for (RigidBody &body : m_bodies) {
    body.applyForce(m_gravity * body.mass);
    body.update(dt);
  }
  resolveCollisions();
}

void World::resolveCollisions() {
  for (size_t i = 0; i < m_bodies.size(); i++) {
    RigidBody &body1 = m_bodies[i];

    for (size_t j = i + 1; j < m_bodies.size(); j++) {
      RigidBody &body2 = m_bodies[j];

      float radiusSum = body1.radius + body2.radius;
      float radiusSumSq = radiusSum * radiusSum;
      Vec3 diffVector = body1.position - body2.position;
      float diffSq = diffVector.lengthSq();

      if (radiusSumSq > diffSq) {
        resolveContact(body1, body2);
      }
    }
  }
}

void World::resolveContact(RigidBody &body1, RigidBody &body2) {
  Vec3 offset = body1.position - body2.position;
  if (offset.lengthSq() < 1e-12f) {
    return;
  }

  Vec3 collisionNormal = offset.normalized();
  Vec3 relativeVelocity = body1.velocity - body2.velocity;
  float closingSpeed = relativeVelocity.dot(collisionNormal);

  if (closingSpeed >= 0)
    return;

  float e = 1; // coefficient of restitution (0 = perfectly inelastic, 1 =
               // perfectly elastic)

  float impulseMagnitude =
      -(1.f + e) * (closingSpeed) / (body1.invMass + body2.invMass);

  body1.velocity += collisionNormal * impulseMagnitude * body1.invMass;
  body2.velocity -= collisionNormal * impulseMagnitude * body2.invMass;
}
