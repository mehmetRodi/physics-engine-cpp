#include <gtest/gtest.h>

#include "physics/World.hpp"

TEST(WorldTests, GravityAffectsDynamicBodyDuringStep) {
  World world(Vec3(0.f, -9.8f, 0.f));
  RigidBody body(1.0f, 20.0f);
  world.addBody(&body);
  world.step(1.0f);
  EXPECT_FLOAT_EQ(body.velocity.y, -9.8f);
  EXPECT_FLOAT_EQ(body.position.y, -9.8f);
}

TEST(WorldTests, ZeroGravityLeavesRestingBodyAtRest) {
  World world(Vec3(0.f, 0.f, 0.f));
  RigidBody body(1.0f, 20.0f);
  world.addBody(&body);
  world.step(1.0f);
  EXPECT_FLOAT_EQ(body.velocity.y, 0.f);
  EXPECT_FLOAT_EQ(body.position.y, 0.f);
}

TEST(WorldTests, SameInitialStateProducesSameResult) {
  World world1(Vec3(0.f, -9.8f, 0.f));
  RigidBody body1(1.0f, 20.0f);
  world1.addBody(&body1);

  World world2(Vec3(0.f, -9.8f, 0.f));
  RigidBody body2(1.0f, 20.0f);
  world2.addBody(&body2);

  world1.step(1.0f);
  world2.step(1.0f);

  EXPECT_FLOAT_EQ(body1.velocity.y, body2.velocity.y);
  EXPECT_FLOAT_EQ(body1.position.y, body2.position.y);
}

TEST(WorldTests, StaticBodyDoesNotMoveUnderGravity) {
  World world(Vec3(0.f, -9.8f, 0.f));
  RigidBody body(0.0f, 20.0f); // Zero mass means static body.
  world.addBody(&body);
  world.step(1.0f);
  EXPECT_FLOAT_EQ(body.velocity.y, 0.f);
  EXPECT_FLOAT_EQ(body.position.y, 0.f);
}
