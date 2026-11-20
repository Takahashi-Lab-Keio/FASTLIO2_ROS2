#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include "localizers/icp_localizer.h"

namespace
{
std::filesystem::path temporaryPath(const std::string &suffix)
{
    return std::filesystem::temp_directory_path() /
           ("fastlio2_localizer_" +
            std::to_string(
                std::chrono::steady_clock::now().time_since_epoch().count()) +
            suffix);
}

CloudType::Ptr makeAsymmetricMap()
{
    CloudType::Ptr cloud(new CloudType);
    cloud->reserve(600U);
    for (int index = 0; index < 600; ++index)
    {
        PointType point;
        point.x = static_cast<float>(
            (index % 29) * 0.11 + 0.03 * std::sin(index * 0.31));
        point.y = static_cast<float>(
            ((index * 7) % 31) * 0.09 + 0.02 * std::cos(index * 0.23));
        point.z = static_cast<float>(
            0.25 * std::sin(index * 0.17) + (index % 5) * 0.04);
        point.intensity = static_cast<float>(index % 255);
        cloud->push_back(point);
    }
    return cloud;
}

ICPConfig testConfig()
{
    ICPConfig config;
    config.rough_scan_resolution = 0.0;
    config.rough_map_resolution = 0.0;
    config.rough_max_iteration = 60;
    config.rough_score_thresh = 0.02;
    config.rough_max_correspondence_distance = 0.8;
    config.refine_scan_resolution = 0.0;
    config.refine_map_resolution = 0.0;
    config.refine_max_iteration = 60;
    config.refine_score_thresh = 0.005;
    config.refine_max_correspondence_distance = 0.3;
    return config;
}
} // namespace

TEST(IcpLocalizer, RecoversKnownTransformFromNearbyGuess)
{
    const auto map = makeAsymmetricMap();
    const auto path = temporaryPath("_map.pcd");
    ASSERT_EQ(pcl::io::savePCDFileBinary(path.string(), *map), 0);

    const Eigen::Matrix3f rotation =
        Eigen::AngleAxisf(0.12F, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    const Eigen::Vector3f translation(0.30F, -0.20F, 0.08F);
    Eigen::Matrix4f expected = Eigen::Matrix4f::Identity();
    expected.block<3, 3>(0, 0) = rotation;
    expected.block<3, 1>(0, 3) = translation;

    CloudType::Ptr scan(new CloudType);
    pcl::transformPointCloud(*map, *scan, expected.inverse());

    ICPLocalizer localizer(testConfig());
    ASSERT_TRUE(localizer.loadMap(path.string()));
    localizer.setInput(scan);
    Eigen::Matrix4f guess = expected;
    guess(0, 3) += 0.04F;
    guess(1, 3) -= 0.03F;

    ASSERT_TRUE(localizer.align(guess));
    EXPECT_TRUE(guess.isApprox(expected, 2e-2F))
        << "expected:\n" << expected << "\nactual:\n" << guess;
    const Eigen::Matrix3f recovered_rotation = guess.block<3, 3>(0, 0);
    EXPECT_TRUE(
        (recovered_rotation.transpose() * recovered_rotation)
            .isApprox(Eigen::Matrix3f::Identity(), 1e-5F));
    EXPECT_NEAR(recovered_rotation.determinant(), 1.0F, 1e-5F);

    std::filesystem::remove(path);
}

TEST(IcpLocalizer, RejectsPoorGuessAndInvalidMap)
{
    const auto map = makeAsymmetricMap();
    const auto map_path = temporaryPath("_map.pcd");
    const auto corrupt_path = temporaryPath("_corrupt.pcd");
    ASSERT_EQ(pcl::io::savePCDFileBinary(map_path.string(), *map), 0);
    {
        std::ofstream corrupt(corrupt_path);
        corrupt << "not a PCD file\n";
    }

    ICPConfig config = testConfig();
    config.rough_max_correspondence_distance = 0.2;
    config.refine_max_correspondence_distance = 0.1;
    ICPLocalizer localizer(config);
    ASSERT_TRUE(localizer.loadMap(map_path.string()));
    EXPECT_FALSE(localizer.loadMap(corrupt_path.string()));
    localizer.setInput(map);

    Eigen::Matrix4f retained_map_guess = Eigen::Matrix4f::Identity();
    EXPECT_TRUE(localizer.align(retained_map_guess));

    Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
    guess(0, 3) = 10.0F;
    EXPECT_FALSE(localizer.align(guess));

    std::filesystem::remove(map_path);
    std::filesystem::remove(corrupt_path);
}
