#include "math/Vec3.hpp"
#include <gtest/gtest.h>

TEST(Vec3Tests, DefaultConstructorStartsAtOrigin) {
  const Vec3 value;

  EXPECT_FLOAT_EQ(value.x, 0.0f);
  EXPECT_FLOAT_EQ(value.y, 0.0f);
  EXPECT_FLOAT_EQ(value.z, 0.0f);
}
