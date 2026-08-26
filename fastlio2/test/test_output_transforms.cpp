#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include "output_transforms.h"
#include "map_builder/imu_processor.h"
#include "map_builder/so3_math.h"

TEST(OutputTransforms, KeepsSiImuAccelerationUnscaled)
{
    const V3D measured(0.2, -0.1, 9.81);
    EXPECT_TRUE(FastlioTransforms::scaleImuAcceleration(measured, 1.0).isApprox(measured));
    EXPECT_TRUE(FastlioTransforms::scaleImuAcceleration(measured, 10.0).isApprox(measured * 10.0));
}

TEST(ImuInitialization, RejectsAccelerationOutsideConfiguredSiRange)
{
    Config config;
    config.imu_init_num = 2;
    config.imu_init_accel_min = 5.0;
    config.imu_init_accel_max = 15.0;
    auto filter = std::make_shared<IESKF>();
    IMUProcessor processor(config, filter);
    SyncPackage package;
    package.cloud_end_time = 1.0;
    package.imus.emplace_back(V3D(0.0, 0.0, 1.0), V3D::Zero(), 0.9);
    package.imus.emplace_back(V3D(0.0, 0.0, 1.0), V3D::Zero(), 1.0);

    EXPECT_THROW(processor.initialize(package), std::runtime_error);
}

TEST(OutputTransforms, ConvertsImuAndLidarPosesIntoBaseFrame)
{
    constexpr double pitch = -0.0436;
    const M3D r_bi = FastlioSo3::exp(V3D(0.0, pitch, 0.0));
    const V3D t_bi(-0.094086, -0.011775, 0.507910);
    const M3D r_il = FastlioSo3::exp(V3D(0.0, 0.0, M_PI));
    const V3D t_il(-0.006253, 0.011775, 0.028535);
    const M3D r_wi = FastlioSo3::exp(V3D(0.1, -0.2, 0.3));
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

TEST(OutputTransforms, KeepsFixedBaseLidarMountWhileEstimatingImuExtrinsic)
{
    const M3D r_wi = FastlioSo3::exp(V3D(0.1, -0.2, 0.3));
    const V3D t_wi(1.0, 2.0, 3.0);
    const M3D r_il = FastlioSo3::exp(V3D(-0.02, 0.01, 0.03));
    const V3D t_il(-0.008, 0.009, 0.039);
    const M3D r_bl = FastlioSo3::exp(V3D(0.0, 0.0, -M_PI_2));
    const V3D t_bl(0.06125, 0.0, 0.728);

    const auto base_pose = FastlioTransforms::lidarPoseToBasePose(
        r_wi, t_wi, r_il, t_il, r_bl, t_bl);
    const auto lidar_pose = FastlioTransforms::compose(
        {base_pose.rotation, base_pose.translation}, {r_bl, t_bl});

    EXPECT_TRUE(lidar_pose.rotation.isApprox(r_wi * r_il, 1e-12));
    EXPECT_TRUE(lidar_pose.translation.isApprox(r_wi * t_il + t_wi, 1e-12));
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
    EXPECT_EQ(
        FastlioTransforms::classifyTimestamp(10.2, 10.0, 0.5, 0.1),
        TimestampStatus::FATAL_FORWARD_GAP);
}
