#pragma once
#include "math/Vec3.hpp"

struct RigidBody {
  Vec3 position;
  Vec3 velocity;
  Vec3 acceleration;
  float mass;
  float invMass; // Used for multiplication cuz it is faster
  float radius;  // radius to be used in collision detection

  RigidBody(float m, float r);

  void applyForce(const Vec3 &force);

  void update(float dt);
};
