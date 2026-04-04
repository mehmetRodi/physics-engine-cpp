#pragma once
#include "math/Vec3.hpp"

struct RigidBody {
  Vec3 position;
  Vec3 velocity;
  Vec3 acceleration;
  float mass;
  float invMass; // Used for multiplication cuz it is faster

  RigidBody(float m) : mass(m) {
    if (m > 0.0f)
      invMass = 1.0f / m;
    else
      invMass = 0.0f; // represents unmovable object
  }

  void applyForce(const Vec3 &force) {

    acceleration = acceleration + (force * invMass);
  }

  void update(float dt) {
    // semi-implicit Euler Integration
    velocity = velocity + (acceleration * dt);
    position = position + (velocity * dt);

    acceleration =
        Vec3(0.0f, 0.0f, 0.0f); // reset acceleration for the next frame
  }
};
