#include "physics/World.hpp"

World::World(Vec3 gravity) : m_gravity(gravity) {}

World::RigidBodyId World::createRigidBody(float mass, float radius) {
  return m_rigidBodySystem.createRigidBody(mass, radius);
}

RigidBody& World::rigidBody(RigidBodyId id) {
  return m_rigidBodySystem.rigidBody(id);
}

const RigidBody& World::rigidBody(RigidBodyId id) const {
  return m_rigidBodySystem.rigidBody(id);
}

void World::reserveRigidBodies(std::size_t capacity) {
  m_rigidBodySystem.reserveRigidBodies(capacity);
}

void World::step(float dt) {
  m_rigidBodySystem.step(dt, m_gravity);
}
