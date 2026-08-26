#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <limits>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "utils.h"

namespace
{
using PointField = sensor_msgs::msg::PointField;

template <typename T>
void writeScalar(
    sensor_msgs::msg::PointCloud2 &message,
    size_t point_index,
    uint32_t field_offset,
    T value)
{
    const size_t row = point_index / message.width;
    const size_t column = point_index % message.width;
    const size_t offset =
        row * message.row_step +
        column * message.point_step +
        field_offset;
    std::memcpy(message.data.data() + offset, &value, sizeof(T));
}

PointField field(
    const std::string &name,
    uint32_t offset,
    uint8_t datatype)
{
    PointField output;
    output.name = name;
    output.offset = offset;
    output.datatype = datatype;
    output.count = 1U;
    return output;
}

sensor_msgs::msg::PointCloud2 makeJt128Cloud(
    const std::array<double, 4> &timestamps)
{
    sensor_msgs::msg::PointCloud2 message;
    message.header.frame_id = "hesai_lidar";
    message.header.stamp.sec = 1700000000;
    message.header.stamp.nanosec = 250000000U;
    message.width = 2U;
    message.height = 2U;
    message.point_step = 26U;
    // Include row padding to exercise organized PointCloud2 indexing.
    message.row_step = message.point_step * message.width + 4U;
    message.is_bigendian = false;
    message.fields = {
        field("x", 0U, PointField::FLOAT32),
        field("y", 4U, PointField::FLOAT32),
        field("z", 8U, PointField::FLOAT32),
        field("intensity", 12U, PointField::FLOAT32),
        field("ring", 16U, PointField::UINT16),
        field("timestamp", 18U, PointField::FLOAT64),
    };
    message.data.resize(
        static_cast<size_t>(message.row_step) * message.height, 0U);

    for (size_t index = 0; index < timestamps.size(); ++index)
    {
        writeScalar<float>(
            message, index, 0U, static_cast<float>(index + 1U));
        writeScalar<float>(message, index, 4U, 0.0F);
        writeScalar<float>(message, index, 8U, 0.0F);
        writeScalar<float>(
            message, index, 12U, static_cast<float>(10U + index));
        writeScalar<uint16_t>(
            message, index, 16U, static_cast<uint16_t>(index));
        writeScalar<double>(
            message, index, 18U, timestamps[index]);
    }
    return message;
}

sensor_msgs::msg::PointCloud2 makeOusterCloud(bool include_time)
{
    sensor_msgs::msg::PointCloud2 message;
    sensor_msgs::PointCloud2Modifier modifier(message);
    if (include_time)
    {
        modifier.setPointCloud2Fields(
            5,
            "x", 1, PointField::FLOAT32,
            "y", 1, PointField::FLOAT32,
            "z", 1, PointField::FLOAT32,
            "intensity", 1, PointField::FLOAT32,
            "t", 1, PointField::UINT32);
    }
    else
    {
        modifier.setPointCloud2Fields(
            4,
            "x", 1, PointField::FLOAT32,
            "y", 1, PointField::FLOAT32,
            "z", 1, PointField::FLOAT32,
            "intensity", 1, PointField::FLOAT32);
    }
    modifier.resize(4U);
    message.width = 4U;
    message.height = 1U;
    message.row_step = message.point_step * message.width;

    sensor_msgs::PointCloud2Iterator<float> x(message, "x");
    sensor_msgs::PointCloud2Iterator<float> y(message, "y");
    sensor_msgs::PointCloud2Iterator<float> z(message, "z");
    sensor_msgs::PointCloud2Iterator<float> intensity(message, "intensity");
    for (size_t index = 0; index < 4U;
         ++index, ++x, ++y, ++z, ++intensity)
    {
        *x = static_cast<float>(index + 1U);
        *y = 0.0F;
        *z = 0.0F;
        *intensity = static_cast<float>(20U + index);
    }
    return message;
}
} // namespace

TEST(PointCloud2Conversion, ReadsPackedJt128AbsoluteTimestampsAndSortsStably)
{
    const double start = 1700000000.25;
    auto message = makeJt128Cloud(
        {start + 0.090, start + 0.010, start + 0.010, start + 0.050});

    const auto result = Utils::pointCloud2ToPCL(
        message, 1, 0.5, 10.0, true, 200.0,
        "timestamp", "absolute_seconds");
    ASSERT_TRUE(result.success) << result.error;
    ASSERT_EQ(result.cloud->size(), 4U);
    EXPECT_FLOAT_EQ(result.cloud->points[0].x, 2.0F);
    EXPECT_FLOAT_EQ(result.cloud->points[1].x, 3.0F);
    EXPECT_FLOAT_EQ(result.cloud->points[2].x, 4.0F);
    EXPECT_FLOAT_EQ(result.cloud->points[3].x, 1.0F);
    EXPECT_NEAR(result.maximum_offset_ms, 90.0, 1e-3);
    EXPECT_NEAR(result.cloud->back().curvature, 90.0F, 1e-3F);
}

TEST(PointCloud2Conversion, ScanEndUsesPointsRemovedByFiltering)
{
    const double start = 1700000000.25;
    auto message = makeJt128Cloud(
        {start, start + 0.010, start + 0.050, start + 0.100});
    // The final point is both decimated and outside the configured range.
    writeScalar<float>(message, 3U, 0U, 100.0F);

    const auto result = Utils::pointCloud2ToPCL(
        message, 2, 0.5, 10.0, true, 200.0,
        "timestamp", "absolute_seconds");
    ASSERT_TRUE(result.success) << result.error;
    ASSERT_EQ(result.cloud->size(), 2U);
    EXPECT_NEAR(result.cloud->back().curvature, 50.0F, 1e-3F);
    EXPECT_NEAR(result.maximum_offset_ms, 100.0, 1e-3);
}

TEST(PointCloud2Conversion, SupportsOusterRelativeNanoseconds)
{
    auto message = makeOusterCloud(true);
    sensor_msgs::PointCloud2Iterator<uint32_t> time(message, "t");
    const uint32_t offsets[] = {
        90000000U, 10000000U, 10000000U, 50000000U};
    for (const uint32_t offset : offsets)
    {
        *time = offset;
        ++time;
    }

    const auto result = Utils::pointCloud2ToPCL(
        message, 1, 0.5, 10.0, true, 200.0,
        "t", "relative_nanoseconds");
    ASSERT_TRUE(result.success) << result.error;
    EXPECT_DOUBLE_EQ(result.maximum_offset_ms, 90.0);
    EXPECT_FLOAT_EQ(result.cloud->points[0].x, 2.0F);
    EXPECT_FLOAT_EQ(result.cloud->points[1].x, 3.0F);
}

TEST(PointCloud2Conversion, FiltersNonFiniteAndOutOfRangePoints)
{
    const double start = 1700000000.25;
    auto message = makeJt128Cloud(
        {start, start + 0.010, start + 0.020, start + 0.030});
    writeScalar<float>(
        message, 0U, 0U, std::numeric_limits<float>::quiet_NaN());
    writeScalar<float>(message, 1U, 0U, 0.1F);

    const auto result = Utils::pointCloud2ToPCL(
        message, 1, 0.5, 10.0, true, 200.0,
        "timestamp", "absolute_seconds");
    ASSERT_TRUE(result.success) << result.error;
    ASSERT_EQ(result.cloud->size(), 2U);
    EXPECT_FLOAT_EQ(result.cloud->points[0].x, 3.0F);
    EXPECT_FLOAT_EQ(result.cloud->points[1].x, 4.0F);
    EXPECT_NEAR(result.maximum_offset_ms, 30.0, 1e-3);
}

TEST(PointCloud2Conversion, EnforcesTimeSchemaAndBounds)
{
    const double start = 1700000000.25;
    auto message = makeJt128Cloud({start, start, start, start});
    message.fields.back().datatype = PointField::FLOAT32;
    EXPECT_FALSE(
        Utils::pointCloud2ToPCL(
            message, 1, 0.5, 10.0, true, 200.0,
            "timestamp", "absolute_seconds")
            .success);

    auto excessive = makeJt128Cloud(
        {start, start + 0.010, start + 0.020, start + 0.250});
    EXPECT_FALSE(
        Utils::pointCloud2ToPCL(
            excessive, 1, 0.5, 10.0, true, 200.0,
            "timestamp", "absolute_seconds")
            .success);

    auto negative = makeJt128Cloud(
        {start - 0.010, start, start, start});
    EXPECT_FALSE(
        Utils::pointCloud2ToPCL(
            negative, 1, 0.5, 10.0, true, 200.0,
            "timestamp", "absolute_seconds")
            .success);
}

TEST(PointCloud2Conversion, CanAcceptMissingTimeForSimulationOnly)
{
    const auto message = makeOusterCloud(false);
    EXPECT_FALSE(
        Utils::pointCloud2ToPCL(
            message, 1, 0.5, 10.0, true, 200.0,
            "t", "relative_nanoseconds")
            .success);

    const auto result = Utils::pointCloud2ToPCL(
        message, 1, 0.5, 10.0, false, 200.0,
        "t", "relative_nanoseconds");
    ASSERT_TRUE(result.success) << result.error;
    EXPECT_FLOAT_EQ(result.cloud->back().curvature, 0.0F);
}

TEST(PointCloud2Conversion, RejectsMalformedStorage)
{
    sensor_msgs::msg::PointCloud2 malformed;
    malformed.width = 2U;
    malformed.height = 1U;
    malformed.point_step = 0x80000000U;
    malformed.row_step = 1U;
    malformed.data.resize(1U);
    for (const auto &name : {"x", "y", "z", "intensity"})
    {
        malformed.fields.push_back(
            field(name, 0U, PointField::FLOAT32));
    }

    EXPECT_FALSE(
        Utils::pointCloud2ToPCL(
            malformed, 1, 0.5, 10.0, false, 200.0,
            "timestamp", "absolute_seconds")
            .success);
}
