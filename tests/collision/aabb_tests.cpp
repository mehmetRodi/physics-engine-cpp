#include "collision/AABB.hpp"

#include <gtest/gtest.h>

TEST(AABBTests, SeparatedBoxesDoNotOverlap) {
  const AABB a{Vec3(0.f, 0.f, 0.f), Vec3(1.f, 1.f, 1.f)};
  const AABB b{Vec3(2.f, 2.f, 2.f), Vec3(3.f, 3.f, 3.f)};

  EXPECT_FALSE(a.overlaps(b));
  EXPECT_FALSE(b.overlaps(a));
}

TEST(AABBTests, PartiallyOverlappingBoxesOverlap) {
  const AABB a{Vec3(0.f, 0.f, 0.f), Vec3(2.f, 2.f, 2.f)};
  const AABB b{Vec3(1.f, 1.f, 1.f), Vec3(3.f, 3.f, 3.f)};

  EXPECT_TRUE(a.overlaps(b));
  EXPECT_TRUE(b.overlaps(a));
}

TEST(AABBTests, ContainedBoxOverlaps) {
  const AABB outer{Vec3(0.f, 0.f, 0.f), Vec3(4.f, 4.f, 4.f)};
  const AABB inner{Vec3(1.f, 1.f, 1.f), Vec3(2.f, 2.f, 2.f)};

  EXPECT_TRUE(outer.overlaps(inner));
  EXPECT_TRUE(inner.overlaps(outer));
}

TEST(AABBTests, BoxesTouchingAtFaceOverlap) {
  const AABB a{Vec3(0.f, 0.f, 0.f), Vec3(1.f, 1.f, 1.f)};
  const AABB b{Vec3(1.f, 0.f, 0.f), Vec3(2.f, 1.f, 1.f)};

  EXPECT_TRUE(a.overlaps(b));
  EXPECT_TRUE(b.overlaps(a));
}

TEST(AABBTests, BoxesSeparatedOnOneAxisDoNotOverlap) {
  const AABB a{Vec3(0.f, 0.f, 0.f), Vec3(1.f, 1.f, 1.f)};
  const AABB b{Vec3(1.5f, 0.25f, 0.25f), Vec3(2.5f, 0.75f, 0.75f)};

  EXPECT_FALSE(a.overlaps(b));
  EXPECT_FALSE(b.overlaps(a));
}
