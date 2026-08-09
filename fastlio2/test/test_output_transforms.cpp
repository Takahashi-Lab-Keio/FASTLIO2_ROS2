#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <sophus/so3.hpp>

#include "output_transforms.h"

TEST(OutputTransforms, KeepsOusterImuAccelerationInSiUnits)
{
    const V3D measured(0.2, -0.1, 9.81);
    EXPECT_TRUE(FastlioTransforms::scaleImuAcceleration(measured, 1.0).isApprox(measured));
    EXPECT_TRUE(FastlioTransforms::scaleImuAcceleration(measured, 10.0).isApprox(measured * 10.0));
}

TEST(OutputTransforms, ConvertsImuAndLidarPosesIntoBaseFrame)
{
    constexpr double pitch = -0.0436;
    const M3D r_bi = Sophus::SO3d::exp(V3D(0.0, pitch, 0.0)).matrix();
    const V3D t_bi(-0.094086, -0.011775, 0.507910);
    const M3D r_il = Sophus::SO3d::exp(V3D(0.0, 0.0, M_PI)).matrix();
    const V3D t_il(-0.006253, 0.011775, 0.028535);
    const M3D r_wi = Sophus::SO3d::exp(V3D(0.1, -0.2, 0.3)).matrix();
    const V3D t_wi(1.0, 2.0, 3.0);

    const auto base_pose = FastlioTransforms::imuPoseToBasePose(
        r_wi, t_wi, r_bi, t_bi);
    EXPECT_TRUE(base_pose.rotation.isApprox(r_wi * r_bi.transpose(), 1e-12));
    EXPECT_TRUE((base_pose.rotation * t_bi + base_pose.translation).isApprox(t_wi, 1e-12));

    const auto lidar_to_base = FastlioTransforms::lidarToBasePose(
        r_il, t_il, r_bi, t_bi);
    const V3D lidar_point(2.0, -1.0, 0.5);
    const V3D composed = r_bi * (r_il * lidar_point + t_il) + t_bi;
    EXPECT_TRUE((lidar_to_base.rotation * lidar_point + lidar_to_base.translation).isApprox(composed, 1e-12));

    const auto anchored = FastlioTransforms::compose(
        FastlioTransforms::inverse(base_pose), base_pose);
    EXPECT_TRUE(anchored.rotation.isApprox(M3D::Identity(), 1e-12));
    EXPECT_TRUE(anchored.translation.isZero(1e-12));
}

TEST(OutputTransforms, ClassifiesDuplicateAndResetTimestamps)
{
    using FastlioTransforms::TimestampStatus;
    EXPECT_EQ(FastlioTransforms::classifyTimestamp(10.1, 10.0), TimestampStatus::ACCEPT);
    EXPECT_EQ(
        FastlioTransforms::classifyTimestamp(10.0, 10.0),
        TimestampStatus::DROP_DUPLICATE_OR_REORDERED);
    EXPECT_EQ(
        FastlioTransforms::classifyTimestamp(9.8, 10.0),
        TimestampStatus::DROP_DUPLICATE_OR_REORDERED);
    EXPECT_EQ(
        FastlioTransforms::classifyTimestamp(9.0, 10.0),
        TimestampStatus::FATAL_REGRESSION);
    EXPECT_EQ(
        FastlioTransforms::classifyTimestamp(1.0, -std::numeric_limits<double>::infinity()),
        TimestampStatus::ACCEPT);
}
