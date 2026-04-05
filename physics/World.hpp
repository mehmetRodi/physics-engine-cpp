#pragma once
#include "math/Vec3.hpp"
#include "physics/RigidBody.hpp"
#include <vector>
class World {
public:
  World(Vec3 gravity);
  World(World &&) = default;
  World(const World &) = default;
  World &operator=(World &&) = default;
  World &operator=(const World &) = default;
  ~World();

  void addBody(RigidBody *body);

  void step(float dt);

private:
  void resolveCollisions();
  void resolveContact(RigidBody *body1, RigidBody *body2);

  Vec3 m_gravity;
  std::vector<RigidBody *> m_bodies;
};
