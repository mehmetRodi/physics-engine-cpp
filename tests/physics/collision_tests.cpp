#include "physics/World.hpp"
#include <gtest/gtest.h>

TEST(CollisionTests, SeparatedBodiesDoNotChangeVelocity) {
  World world(Vec3(0.f, 0.f, 0.f));
  const World::BodyId body1 = world.createBody(1.0f, 1.0f);
  world.body(body1).position = Vec3(0.f, 0.f, 0.f);
  world.body(body1).velocity = Vec3(1.f, 0.f, 0.f);

  const World::BodyId body2 = world.createBody(1.0f, 1.0f);
  world.body(body2).position = Vec3(3.f, 0.f, 0.f);
  world.body(body2).velocity = Vec3(-1.f, 0.f, 0.f);

  world.step(0.0f);

  EXPECT_FLOAT_EQ(world.body(body1).velocity.x, 1.f);
  EXPECT_FLOAT_EQ(world.body(body2).velocity.x, -1.f);
}

TEST(CollisionTests, OverlappingBodiesMovingTogetherDoNotReceiveImpulse) {
  World world(Vec3(0.f, 0.f, 0.f));
  const World::BodyId body1 = world.createBody(1.0f, 1.0f);
  world.body(body1).position = Vec3(0.f, 0.f, 0.f);
  world.body(body1).velocity = Vec3(1.f, 0.f, 0.f);

  const World::BodyId body2 = world.createBody(1.0f, 1.0f);
  world.body(body2).position = Vec3(0.5f, 0.f, 0.f);
  world.body(body2).velocity = Vec3(1.f, 0.f, 0.f);

  world.step(0.0f);

  EXPECT_FLOAT_EQ(world.body(body1).velocity.x, 1.f);
  EXPECT_FLOAT_EQ(world.body(body2).velocity.x, 1.f);
}

TEST(CollisionTests, HeadOnEqualMassCollisionSwapsVelocities) {
  World world(Vec3(0.f, 0.f, 0.f));
  const World::BodyId body1 = world.createBody(1.0f, 1.0f);
  world.body(body1).position = Vec3(0.f, 0.f, 0.f);
  world.body(body1).velocity = Vec3(1.f, 0.f, 0.f);

  const World::BodyId body2 = world.createBody(1.0f, 1.0f);
  world.body(body2).position = Vec3(0.5f, 0.f, 0.f);
  world.body(body2).velocity = Vec3(-1.f, 0.f, 0.f);

  world.step(0.0f);

  EXPECT_FLOAT_EQ(world.body(body1).velocity.x, -1.f);
  EXPECT_FLOAT_EQ(world.body(body2).velocity.x, 1.f);
}

TEST(CollisionTests, DynamicBodyBouncesOffStaticBody) {
  World world(Vec3(0.f, 0.f, 0.f));
  const World::BodyId dynamicBody = world.createBody(1.0f, 1.0f);
  world.body(dynamicBody).position = Vec3(0.f, 0.f, 0.f);
  world.body(dynamicBody).velocity = Vec3(1.f, 0.f, 0.f);

  const World::BodyId staticBody = world.createBody(0.0f, 1.0f);
  world.body(staticBody).position = Vec3(0.5f, 0.f, 0.f);
  world.body(staticBody).velocity = Vec3(0.f, 0.f, 0.f);

  world.step(0.0f);

  EXPECT_FLOAT_EQ(world.body(dynamicBody).velocity.x, -1.f);
  EXPECT_FLOAT_EQ(world.body(staticBody).velocity.x, 0.f);
}

TEST(CollisionTests, ExactlyTouchingBodiesDoNotCollide) {
  World world(Vec3(0.f, 0.f, 0.f));
  const World::BodyId body1 = world.createBody(1.0f, 1.0f);
  world.body(body1).position = Vec3(0.f, 0.f, 0.f);
  world.body(body1).velocity = Vec3(1.f, 0.f, 0.f);

  const World::BodyId body2 = world.createBody(1.0f, 1.0f);
  world.body(body2).position = Vec3(2.f, 0.f, 0.f);
  world.body(body2).velocity = Vec3(-1.f, 0.f, 0.f);

  world.step(0.0f);

  EXPECT_FLOAT_EQ(world.body(body1).velocity.x, 1.f);
  EXPECT_FLOAT_EQ(world.body(body2).velocity.x, -1.f);
}

TEST(CollisionTests, IdenticalPositionsDoNotApplyAmbiguousImpulse) {
  World world(Vec3(0.f, 0.f, 0.f));
  const World::BodyId body1 = world.createBody(1.0f, 1.0f);
  world.body(body1).position = Vec3(0.f, 0.f, 0.f);
  world.body(body1).velocity = Vec3(1.f, 0.f, 0.f);

  const World::BodyId body2 = world.createBody(1.0f, 1.0f);
  world.body(body2).position = Vec3(0.f, 0.f, 0.f);
  world.body(body2).velocity = Vec3(-1.f, 0.f, 0.f);

  world.step(0.0f);

  EXPECT_FLOAT_EQ(world.body(body1).velocity.x, 1.f);
  EXPECT_FLOAT_EQ(world.body(body1).velocity.y, 0.f);
  EXPECT_FLOAT_EQ(world.body(body1).velocity.z, 0.f);
  EXPECT_FLOAT_EQ(world.body(body2).velocity.x, -1.f);
  EXPECT_FLOAT_EQ(world.body(body2).velocity.y, 0.f);
  EXPECT_FLOAT_EQ(world.body(body2).velocity.z, 0.f);
}

TEST(CollisionTests, OverlappingStaticBodiesDoNotChangeVelocity) {
  World world(Vec3(0.f, 0.f, 0.f));
  const World::BodyId body1 = world.createBody(0.0f, 1.0f);
  world.body(body1).position = Vec3(0.f, 0.f, 0.f);
  world.body(body1).velocity = Vec3(0.f, 0.f, 0.f);

  const World::BodyId body2 = world.createBody(0.0f, 1.0f);
  world.body(body2).position = Vec3(0.5f, 0.f, 0.f);
  world.body(body2).velocity = Vec3(0.f, 0.f, 0.f);

  world.step(0.0f);

  EXPECT_FLOAT_EQ(world.body(body1).velocity.x, 0.f);
  EXPECT_FLOAT_EQ(world.body(body1).velocity.y, 0.f);
  EXPECT_FLOAT_EQ(world.body(body1).velocity.z, 0.f);
  EXPECT_FLOAT_EQ(world.body(body2).velocity.x, 0.f);
  EXPECT_FLOAT_EQ(world.body(body2).velocity.y, 0.f);
  EXPECT_FLOAT_EQ(world.body(body2).velocity.z, 0.f);
}
