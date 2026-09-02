#include <cmath>
#include <cstdlib>
#include <cstdint>

#include <gtest/gtest.h>

#include "elfin3_jog/jog_math.hpp"

TEST(JogMath, UsesOneHundredCountsEveryTwoMilliseconds)
{
  EXPECT_EQ(elfin3_jog::kJogCountsPerUpdate, 100);
  const double step_rad = elfin3_jog::counts_to_radians(100);
  EXPECT_NEAR(step_rad, 4.746227685e-5, 1.0e-14);
  EXPECT_NEAR(step_rad / 0.002, 0.02373113842645, 1.0e-12);
}

TEST(JogMath, AppliesPositiveNegativeAndBoundedSteps)
{
  std::int64_t next = 0;
  EXPECT_TRUE(elfin3_jog::bounded_jog_step(1000, 1, -2000, 2000, next));
  EXPECT_EQ(next, 1100);
  EXPECT_TRUE(elfin3_jog::bounded_jog_step(1000, -1, -2000, 2000, next));
  EXPECT_EQ(next, 900);
  EXPECT_FALSE(elfin3_jog::bounded_jog_step(1950, 1, -2000, 2000, next));
  EXPECT_EQ(next, 1950);
}

TEST(JogMath, PulseRadianRoundTripIsWithinOneCount)
{
  for (const std::int64_t counts : {-13238272, -100, 0, 100, 13238272}) {
    EXPECT_LE(
      std::llabs(elfin3_jog::radians_to_counts(
        elfin3_jog::counts_to_radians(counts)) - counts),
      1);
  }
}
