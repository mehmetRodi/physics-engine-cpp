#include <gtest/gtest.h>

#include "physics/World.hpp"

TEST(WorldTests, GravityAffectsDynamicBodyDuringStep) {
  World world(Vec3(0.f, -9.8f, 0.f));
  const World::BodyId body = world.createBody(1.0f, 20.0f);
  world.step(1.0f);
  EXPECT_FLOAT_EQ(world.body(body).velocity.y, -9.8f);
  EXPECT_FLOAT_EQ(world.body(body).position.y, -9.8f);
}

TEST(WorldTests, ZeroGravityLeavesRestingBodyAtRest) {
  World world(Vec3(0.f, 0.f, 0.f));
  const World::BodyId body = world.createBody(1.0f, 20.0f);
  world.step(1.0f);
  EXPECT_FLOAT_EQ(world.body(body).velocity.y, 0.f);
  EXPECT_FLOAT_EQ(world.body(body).position.y, 0.f);
}

TEST(WorldTests, SameInitialStateProducesSameResult) {
  World world1(Vec3(0.f, -9.8f, 0.f));
  const World::BodyId body1 = world1.createBody(1.0f, 20.0f);

  World world2(Vec3(0.f, -9.8f, 0.f));
  const World::BodyId body2 = world2.createBody(1.0f, 20.0f);

  world1.step(1.0f);
  world2.step(1.0f);

  EXPECT_FLOAT_EQ(world1.body(body1).velocity.y, world2.body(body2).velocity.y);
  EXPECT_FLOAT_EQ(world1.body(body1).position.y, world2.body(body2).position.y);
}

TEST(WorldTests, StaticBodyDoesNotMoveUnderGravity) {
  World world(Vec3(0.f, -9.8f, 0.f));
  const World::BodyId body = world.createBody(0.0f, 20.0f);
  world.step(1.0f);
  EXPECT_FLOAT_EQ(world.body(body).velocity.y, 0.f);
  EXPECT_FLOAT_EQ(world.body(body).position.y, 0.f);
}
