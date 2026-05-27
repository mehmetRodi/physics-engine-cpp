#include <gtest/gtest.h>

#include "physics/RigidBodySystem.hpp"

#include <vector>

class RigidBodySystemTestAccess {
public:
  static const std::vector<RigidBodyContact>& contacts(const RigidBodySystem& system) {
    return system.m_contacts;
  }
};

TEST(RigidBodyContactTests, ContactsFollowCollisionPairOrder) {
  RigidBodySystem system;
  system.reserveRigidBodies(3);

  const RigidBodySystem::RigidBodyId body0 = system.createRigidBody(1.0f, 1.0f);
  const RigidBodySystem::RigidBodyId body1 = system.createRigidBody(1.0f, 1.0f);
  const RigidBodySystem::RigidBodyId body2 = system.createRigidBody(1.0f, 1.0f);

  system.rigidBody(body0).position = Vec3(0.0f, 0.0f, 0.0f);
  system.rigidBody(body1).position = Vec3(1.5f, 0.0f, 0.0f);
  system.rigidBody(body2).position = Vec3(0.0f, 1.5f, 0.0f);

  system.rigidBody(body0).material.restitution = 0.25f;
  system.rigidBody(body1).material.restitution = 0.5f;
  system.rigidBody(body2).material.restitution = 0.75f;

  system.step(0.0f, Vec3(0.0f, 0.0f, 0.0f));

  const std::vector<RigidBodyContact>& contacts = RigidBodySystemTestAccess::contacts(system);

  ASSERT_EQ(contacts.size(), 2u);

  EXPECT_EQ(contacts[0].a, body0);
  EXPECT_EQ(contacts[0].b, body1);
  EXPECT_FLOAT_EQ(contacts[0].normal.x, -1.0f);
  EXPECT_FLOAT_EQ(contacts[0].normal.y, 0.0f);
  EXPECT_FLOAT_EQ(contacts[0].penetration, 0.5f);
  EXPECT_FLOAT_EQ(contacts[0].restitution, 0.5f);

  EXPECT_EQ(contacts[1].a, body0);
  EXPECT_EQ(contacts[1].b, body2);
  EXPECT_FLOAT_EQ(contacts[1].normal.x, 0.0f);
  EXPECT_FLOAT_EQ(contacts[1].normal.y, -1.0f);
  EXPECT_FLOAT_EQ(contacts[1].penetration, 0.5f);
  EXPECT_FLOAT_EQ(contacts[1].restitution, 0.75f);
}
