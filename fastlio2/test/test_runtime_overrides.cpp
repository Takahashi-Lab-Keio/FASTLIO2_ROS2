#include <gtest/gtest.h>

#include "runtime_overrides.h"

TEST(RuntimeOverrides, EmptyFrameOverridePreservesYamlValue)
{
    EXPECT_EQ(
        FastlioRuntimeOverrides::selectFrame("fastlio_odom", ""),
        "fastlio_odom");
}

TEST(RuntimeOverrides, NonEmptyFrameOverrideWins)
{
    EXPECT_EQ(
        FastlioRuntimeOverrides::selectFrame("fastlio_odom", "odom"),
        "odom");
}

TEST(RuntimeOverrides, MissingPointTimeFlagOnlyRelaxesRequirement)
{
    EXPECT_TRUE(FastlioRuntimeOverrides::requirePointTime(true, false));
    EXPECT_FALSE(FastlioRuntimeOverrides::requirePointTime(false, false));
    EXPECT_FALSE(FastlioRuntimeOverrides::requirePointTime(true, true));
    EXPECT_FALSE(FastlioRuntimeOverrides::requirePointTime(false, true));
}
