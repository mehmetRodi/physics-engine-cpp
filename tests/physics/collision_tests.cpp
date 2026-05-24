#include "physics/World.hpp"
#include <gtest/gtest.h>

TEST(CollisionTests, SeparatedBodiesDoNotChangeVelocity) {
  World world(Vec3(0.f, 0.f, 0.f));
  RigidBody body1(1.0f, 1.0f);
  body1.position = Vec3(0.f, 0.f, 0.f);
  body1.velocity = Vec3(1.f, 0.f, 0.f);
  world.addBody(&body1);

  RigidBody body2(1.0f, 1.0f);
  body2.position = Vec3(3.f, 0.f, 0.f);
  body2.velocity = Vec3(-1.f, 0.f, 0.f);
  world.addBody(&body2);

  world.step(0.0f);

  EXPECT_FLOAT_EQ(body1.velocity.x, 1.f);
  EXPECT_FLOAT_EQ(body2.velocity.x, -1.f);
}

TEST(CollisionTests, OverlappingBodiesMovingTogetherDoNotReceiveImpulse) {
  World world(Vec3(0.f, 0.f, 0.f));
  RigidBody body1(1.0f, 1.0f);
  body1.position = Vec3(0.f, 0.f, 0.f);
  body1.velocity = Vec3(1.f, 0.f, 0.f);
  world.addBody(&body1);

  RigidBody body2(1.0f, 1.0f);
  body2.position = Vec3(0.5f, 0.f, 0.f);
  body2.velocity = Vec3(1.f, 0.f, 0.f);
  world.addBody(&body2);

  world.step(0.0f);

  EXPECT_FLOAT_EQ(body1.velocity.x, 1.f);
  EXPECT_FLOAT_EQ(body2.velocity.x, 1.f);
}

TEST(CollisionTests, HeadOnEqualMassCollisionSwapsVelocities) {
  World world(Vec3(0.f, 0.f, 0.f));
  RigidBody body1(1.0f, 1.0f);
  body1.position = Vec3(0.f, 0.f, 0.f);
  body1.velocity = Vec3(1.f, 0.f, 0.f);
  world.addBody(&body1);

  RigidBody body2(1.0f, 1.0f);
  body2.position = Vec3(0.5f, 0.f, 0.f);
  body2.velocity = Vec3(-1.f, 0.f, 0.f);
  world.addBody(&body2);

  world.step(0.0f);

  EXPECT_FLOAT_EQ(body1.velocity.x, -1.f);
  EXPECT_FLOAT_EQ(body2.velocity.x, 1.f);
}

TEST(CollisionTests, DynamicBodyBouncesOffStaticBody) {
  World world(Vec3(0.f, 0.f, 0.f));
  RigidBody dynamicBody(1.0f, 1.0f);
  dynamicBody.position = Vec3(0.f, 0.f, 0.f);
  dynamicBody.velocity = Vec3(1.f, 0.f, 0.f);
  world.addBody(&dynamicBody);

  RigidBody staticBody(0.0f, 1.0f);
  staticBody.position = Vec3(0.5f, 0.f, 0.f);
  staticBody.velocity = Vec3(0.f, 0.f, 0.f);
  world.addBody(&staticBody);

  world.step(0.0f);

  EXPECT_FLOAT_EQ(dynamicBody.velocity.x, -1.f);
  EXPECT_FLOAT_EQ(staticBody.velocity.x, 0.f);
}

TEST(CollisionTests, ExactlyTouchingBodiesDoNotCollide) {
  World world(Vec3(0.f, 0.f, 0.f));
  RigidBody body1(1.0f, 1.0f);
  body1.position = Vec3(0.f, 0.f, 0.f);
  body1.velocity = Vec3(1.f, 0.f, 0.f);
  world.addBody(&body1);

  RigidBody body2(1.0f, 1.0f);
  body2.position = Vec3(2.f, 0.f, 0.f);
  body2.velocity = Vec3(-1.f, 0.f, 0.f);
  world.addBody(&body2);

  world.step(0.0f);

  EXPECT_FLOAT_EQ(body1.velocity.x, 1.f);
  EXPECT_FLOAT_EQ(body2.velocity.x, -1.f);
}
