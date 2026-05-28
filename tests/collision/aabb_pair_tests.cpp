#include "collision/AABBPair.hpp"

#include <gtest/gtest.h>

#include <vector>

TEST(AABBPairTests, SeparatedProxiesProduceNoPairs) {
  const std::vector<AABBProxy> proxies = {
      {0, {Vec3(0.f, 0.f, 0.f), Vec3(1.f, 1.f, 1.f)}},
      {1, {Vec3(2.f, 2.f, 2.f), Vec3(3.f, 3.f, 3.f)}},
  };

  const std::vector<AABBPair> pairs = findAABBPairs(proxies);

  EXPECT_TRUE(pairs.empty());
}

TEST(AABBPairTests, OverlappingProxiesProduceOnePair) {
  const std::vector<AABBProxy> proxies = {
      {0, {Vec3(0.f, 0.f, 0.f), Vec3(2.f, 2.f, 2.f)}},
      {1, {Vec3(1.f, 1.f, 1.f), Vec3(3.f, 3.f, 3.f)}},
  };

  const std::vector<AABBPair> pairs = findAABBPairs(proxies);

  ASSERT_EQ(pairs.size(), 1u);
  EXPECT_EQ(pairs[0].a, 0u);
  EXPECT_EQ(pairs[0].b, 1u);
}

TEST(AABBPairTests, TouchingProxiesProduceOnePair) {
  const std::vector<AABBProxy> proxies = {
      {0, {Vec3(0.f, 0.f, 0.f), Vec3(1.f, 1.f, 1.f)}},
      {1, {Vec3(1.f, 0.f, 0.f), Vec3(2.f, 1.f, 1.f)}},
  };

  const std::vector<AABBPair> pairs = findAABBPairs(proxies);

  ASSERT_EQ(pairs.size(), 1u);
  EXPECT_EQ(pairs[0].a, 0u);
  EXPECT_EQ(pairs[0].b, 1u);
}

TEST(AABBPairTests, PairOrderIsDeterministic) {
  const std::vector<AABBProxy> proxies = {
      {10, {Vec3(0.f, 0.f, 0.f), Vec3(2.f, 2.f, 2.f)}},
      {20, {Vec3(1.f, 0.f, 0.f), Vec3(3.f, 2.f, 2.f)}},
      {30, {Vec3(0.f, 1.f, 0.f), Vec3(2.f, 3.f, 2.f)}},
  };

  const std::vector<AABBPair> pairs = findAABBPairs(proxies);

  ASSERT_EQ(pairs.size(), 3u);

  EXPECT_EQ(pairs[0].a, 10u);
  EXPECT_EQ(pairs[0].b, 20u);

  EXPECT_EQ(pairs[1].a, 10u);
  EXPECT_EQ(pairs[1].b, 30u);

  EXPECT_EQ(pairs[2].a, 20u);
  EXPECT_EQ(pairs[2].b, 30u);
}

TEST(AABBPairTests, OutputVectorOverloadClearsPreviousContents) {
  const std::vector<AABBProxy> proxies = {
      {0, {Vec3(0.f, 0.f, 0.f), Vec3(1.f, 1.f, 1.f)}},
      {1, {Vec3(3.f, 3.f, 3.f), Vec3(4.f, 4.f, 4.f)}},
  };

  std::vector<AABBPair> pairs = {{100, 200}};
  findAABBPairs(proxies, pairs);

  EXPECT_TRUE(pairs.empty());
}
