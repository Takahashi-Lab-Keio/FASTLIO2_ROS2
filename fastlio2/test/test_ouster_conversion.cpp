#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "utils.h"

namespace
{
sensor_msgs::msg::PointCloud2 makeCloud(bool include_time, uint8_t time_datatype)
{
    sensor_msgs::msg::PointCloud2 message;
    sensor_msgs::PointCloud2Modifier modifier(message);
    if (include_time)
    {
        modifier.setPointCloud2Fields(
            5,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
            "t", 1, time_datatype);
    }
    else
    {
        modifier.setPointCloud2Fields(
            4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    }
    modifier.resize(4);
    message.width = 2;
    message.height = 2;
    message.row_step = message.point_step * message.width;

    sensor_msgs::PointCloud2Iterator<float> x(message, "x");
    sensor_msgs::PointCloud2Iterator<float> y(message, "y");
    sensor_msgs::PointCloud2Iterator<float> z(message, "z");
    sensor_msgs::PointCloud2Iterator<float> intensity(message, "intensity");
    for (size_t index = 0; index < 4U; ++index, ++x, ++y, ++z, ++intensity)
    {
        *x = static_cast<float>(index + 1U);
        *y = 0.0F;
        *z = 0.0F;
        *intensity = static_cast<float>(10U + index);
    }
    return message;
}
} // namespace

TEST(OusterConversion, SortsPointTimeStablyAndPreservesOrganizedData)
{
    auto message = makeCloud(true, sensor_msgs::msg::PointField::UINT32);
    sensor_msgs::PointCloud2Iterator<uint32_t> time(message, "t");
    const uint32_t offsets[] = {90000000U, 10000000U, 10000000U, 50000000U};
    for (const uint32_t offset : offsets)
    {
        *time = offset;
        ++time;
    }

    const auto result = Utils::ouster2PCL(message, 1, 0.5, 10.0, true, 200.0);
    ASSERT_TRUE(result.success) << result.error;
    ASSERT_EQ(result.cloud->size(), 4U);
    EXPECT_FLOAT_EQ(result.cloud->points[0].x, 2.0F);
    EXPECT_FLOAT_EQ(result.cloud->points[1].x, 3.0F);
    EXPECT_FLOAT_EQ(result.cloud->points[2].x, 4.0F);
    EXPECT_FLOAT_EQ(result.cloud->points[3].x, 1.0F);
    EXPECT_DOUBLE_EQ(result.maximum_offset_ms, 90.0);
    for (const auto &point : result.cloud->points)
    {
        EXPECT_FLOAT_EQ(point.normal_x, 0.0F);
        EXPECT_FLOAT_EQ(point.normal_y, 0.0F);
        EXPECT_FLOAT_EQ(point.normal_z, 0.0F);
    }
}

TEST(OusterConversion, FiltersNonFiniteAndOutOfRangePoints)
{
    auto message = makeCloud(true, sensor_msgs::msg::PointField::UINT32);
    sensor_msgs::PointCloud2Iterator<float> x(message, "x");
    *x = std::numeric_limits<float>::quiet_NaN();
    ++x;
    *x = 0.1F;
    sensor_msgs::PointCloud2Iterator<uint32_t> time(message, "t");
    for (size_t index = 0; index < 4U; ++index, ++time)
    {
        *time = static_cast<uint32_t>(index * 1000000U);
    }

    const auto result = Utils::ouster2PCL(message, 1, 0.5, 10.0, true, 200.0);
    ASSERT_TRUE(result.success) << result.error;
    ASSERT_EQ(result.cloud->size(), 2U);
    EXPECT_FLOAT_EQ(result.cloud->points[0].x, 3.0F);
    EXPECT_FLOAT_EQ(result.cloud->points[1].x, 4.0F);
}

TEST(OusterConversion, RequiresUint32PointTimeByDefault)
{
    const auto missing = makeCloud(false, sensor_msgs::msg::PointField::UINT32);
    EXPECT_FALSE(Utils::ouster2PCL(missing, 1, 0.5, 10.0, true).success);

    const auto simulation = Utils::ouster2PCL(missing, 1, 0.5, 10.0, false);
    ASSERT_TRUE(simulation.success) << simulation.error;
    EXPECT_FLOAT_EQ(simulation.cloud->back().curvature, 0.0F);

    const auto wrong_type = makeCloud(true, sensor_msgs::msg::PointField::FLOAT32);
    EXPECT_FALSE(Utils::ouster2PCL(wrong_type, 1, 0.5, 10.0, true).success);
}

TEST(OusterConversion, RejectsEmptyAndExcessivePointTime)
{
    sensor_msgs::msg::PointCloud2 empty;
    EXPECT_FALSE(Utils::ouster2PCL(empty, 1).success);

    auto message = makeCloud(true, sensor_msgs::msg::PointField::UINT32);
    sensor_msgs::PointCloud2Iterator<uint32_t> time(message, "t");
    *time = 250000000U;
    EXPECT_FALSE(Utils::ouster2PCL(message, 1, 0.5, 10.0, true, 200.0).success);
}

TEST(OusterConversion, RejectsStorageWhoseUint32DimensionsWouldOverflow)
{
    sensor_msgs::msg::PointCloud2 malformed;
    malformed.width = 2U;
    malformed.height = 1U;
    malformed.point_step = 0x80000000U;
    malformed.row_step = 1U;
    malformed.data.resize(1U);
    for (const auto &name : {"x", "y", "z", "intensity"})
    {
        sensor_msgs::msg::PointField field;
        field.name = name;
        field.offset = 0U;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1U;
        malformed.fields.push_back(field);
    }
    EXPECT_FALSE(Utils::ouster2PCL(malformed, 1, 0.5, 10.0, false).success);
}
