#include <gtest/gtest.h>

#include "runtime_overrides.h"

TEST(RuntimeOverrides, EmptyLocalFrameOverridePreservesYamlValue)
{
    EXPECT_EQ(
        FastlioLocalizerRuntimeOverrides::selectFrame(
            "fastlio_odom", ""),
        "fastlio_odom");
}

TEST(RuntimeOverrides, NonEmptyLocalFrameOverrideWins)
{
    EXPECT_EQ(
        FastlioLocalizerRuntimeOverrides::selectFrame(
            "fastlio_odom", "odom"),
        "odom");
}
