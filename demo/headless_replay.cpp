#include "physics/World.hpp"

#include <iostream>

int main() {
  constexpr float dt = 1.0f / 60.0f;
  constexpr int steps = 120;

  World world(Vec3(0.0f, -9.8f, 0.0f));
  world.reserveRigidBodies(2);

  const World::RigidBodyId falling = world.createRigidBody(1.0f, 0.5f);
  world.rigidBody(falling).position = Vec3(0.0f, 5.0f, 0.0f);
  world.rigidBody(falling).velocity = Vec3(1.0f, 0.0f, 0.0f);
  world.rigidBody(falling).material.restitution = 0.5f;
  world.rigidBody(falling).material.linearDamping = 0.05f;

  const World::RigidBodyId anchor = world.createRigidBody(0.0f, 1.0f);
  world.rigidBody(anchor).position = Vec3(0.0f, 0.0f, 0.0f);

  for (int i = 0; i < steps; ++i) {
    world.step(dt);
  }

  const RigidBody& body = world.rigidBody(falling);

  std::cout << "headless_replay\n";
  std::cout << "steps: " << steps << '\n';
  std::cout << "dt: " << dt << '\n';
  std::cout << "falling.position: " << body.position.x << ", " << body.position.y << ", "
            << body.position.z << '\n';
  std::cout << "falling.velocity: " << body.velocity.x << ", " << body.velocity.y << ", "
            << body.velocity.z << '\n';

  return 0;
}
