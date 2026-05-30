#pragma once
#include "math/Vec3.hpp"

struct BodyMaterial {
  float restitution = 1.0f;
  float linearDamping = 0.0f;
};

enum class ShapeType {
  Sphere,
};

struct RigidBodyShape {
  ShapeType type = ShapeType::Sphere;
  float sphereRadius = 0.0f;
};

struct RigidBody {
  Vec3 position;
  Vec3 velocity;
  Vec3 acceleration;
  float mass;
  float invMass; // Used for multiplication cuz it is faster
  RigidBodyShape shape;
  BodyMaterial material;

  RigidBody(float mass, float radius);

  void applyForce(const Vec3& force);

  void update(float dt);
};
