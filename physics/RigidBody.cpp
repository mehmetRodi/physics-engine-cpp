#include "physics/RigidBody.hpp"
#include <algorithm>
RigidBody::RigidBody(float mass, float radius) : mass(mass), radius(radius) {
  if (mass > 0.0f)
    invMass = 1.0f / mass;
  else
    invMass = 0.0f; // represents unmovable object
}
void RigidBody::applyForce(const Vec3& force) {

  acceleration = acceleration + (force * invMass);
}

void RigidBody::update(float dt) {
  // semi-implicit Euler Integration
  velocity = velocity + (acceleration * dt);
  float dampingScale = std::clamp(1.0f - material.linearDamping * dt, 0.0f, 1.0f);
  velocity = velocity * dampingScale;
  position = position + (velocity * dt);

  acceleration = Vec3(0.0f, 0.0f, 0.0f); // reset acceleration for the next frame
}
