#include "utils.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <sensor_msgs/msg/point_field.hpp>

namespace
{
const sensor_msgs::msg::PointField *findField(
    const sensor_msgs::msg::PointCloud2 &msg, const std::string &name)
{
    const auto it = std::find_if(
        msg.fields.begin(), msg.fields.end(),
        [&name](const sensor_msgs::msg::PointField &field) { return field.name == name; });
    return it == msg.fields.end() ? nullptr : &(*it);
}

bool validateField(
    const sensor_msgs::msg::PointCloud2 &msg,
    const std::string &name,
    uint8_t datatype,
    const sensor_msgs::msg::PointField *&field,
    std::string &error)
{
    field = findField(msg, name);
    if (field == nullptr)
    {
        error = "missing PointCloud2 field '" + name + "'";
        return false;
    }
    if (field->datatype != datatype || field->count != 1U)
    {
        error = "PointCloud2 field '" + name + "' has an unexpected datatype or count";
        return false;
    }
    if (static_cast<size_t>(field->offset) + sizeof(float) > static_cast<size_t>(msg.point_step))
    {
        error = "PointCloud2 field '" + name + "' exceeds point_step";
        return false;
    }
    return true;
}

template <typename T>
T readScalar(const uint8_t *point, uint32_t offset)
{
    T value;
    std::memcpy(&value, point + offset, sizeof(T));
    return value;
}
} // namespace

PointCloudConversionResult Utils::ouster2PCL(
    const sensor_msgs::msg::PointCloud2 &msg,
    int filter_num,
    double min_range,
    double max_range,
    bool require_point_time,
    double maximum_point_time_ms)
{
    PointCloudConversionResult result;
    if (filter_num < 1)
    {
        result.error = "lidar_filter_num must be at least one";
        return result;
    }
    if (!std::isfinite(min_range) || !std::isfinite(max_range) || min_range < 0.0 || max_range <= min_range)
    {
        result.error = "invalid lidar range limits";
        return result;
    }
    if (msg.is_bigendian)
    {
        result.error = "big-endian PointCloud2 is not supported";
        return result;
    }
    if (msg.width == 0U || msg.height == 0U || msg.point_step == 0U)
    {
        result.error = "empty PointCloud2";
        return result;
    }
    const size_t row_payload_size = static_cast<size_t>(msg.point_step) * static_cast<size_t>(msg.width);
    const size_t minimum_size = static_cast<size_t>(msg.row_step) * static_cast<size_t>(msg.height);
    if (static_cast<size_t>(msg.row_step) < row_payload_size || msg.data.size() < minimum_size)
    {
        result.error = "malformed PointCloud2 storage";
        return result;
    }

    const sensor_msgs::msg::PointField *x_field = nullptr;
    const sensor_msgs::msg::PointField *y_field = nullptr;
    const sensor_msgs::msg::PointField *z_field = nullptr;
    const sensor_msgs::msg::PointField *intensity_field = nullptr;
    const sensor_msgs::msg::PointField *time_field = nullptr;
    if (!validateField(msg, "x", sensor_msgs::msg::PointField::FLOAT32, x_field, result.error) ||
        !validateField(msg, "y", sensor_msgs::msg::PointField::FLOAT32, y_field, result.error) ||
        !validateField(msg, "z", sensor_msgs::msg::PointField::FLOAT32, z_field, result.error) ||
        !validateField(msg, "intensity", sensor_msgs::msg::PointField::FLOAT32, intensity_field, result.error))
    {
        return result;
    }

    time_field = findField(msg, "t");
    if (time_field == nullptr)
    {
        if (require_point_time)
        {
            result.error = "missing PointCloud2 field 't'";
            return result;
        }
    }
    else if (time_field->datatype != sensor_msgs::msg::PointField::UINT32 ||
             time_field->count != 1U ||
             static_cast<size_t>(time_field->offset) + sizeof(uint32_t) > static_cast<size_t>(msg.point_step))
    {
        result.error = "PointCloud2 field 't' must be a scalar uint32";
        return result;
    }

    const size_t point_count = static_cast<size_t>(msg.width) * msg.height;
    result.cloud->reserve(point_count / static_cast<size_t>(filter_num) + 1U);
    const double min_range_sq = min_range * min_range;
    const double max_range_sq = max_range * max_range;

    for (size_t index = 0; index < point_count; index += static_cast<size_t>(filter_num))
    {
        const size_t row = index / msg.width;
        const size_t column = index % msg.width;
        const uint8_t *point = msg.data.data() + row * msg.row_step + column * msg.point_step;
        const float x = readScalar<float>(point, x_field->offset);
        const float y = readScalar<float>(point, y_field->offset);
        const float z = readScalar<float>(point, z_field->offset);
        const float intensity = readScalar<float>(point, intensity_field->offset);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(intensity))
        {
            continue;
        }
        const double squared_range = static_cast<double>(x) * x + static_cast<double>(y) * y + static_cast<double>(z) * z;
        if (squared_range < min_range_sq || squared_range > max_range_sq)
        {
            continue;
        }

        double offset_ms = 0.0;
        if (time_field != nullptr)
        {
            offset_ms = static_cast<double>(readScalar<uint32_t>(point, time_field->offset)) * 1e-6;
            if (!std::isfinite(offset_ms) ||
                (maximum_point_time_ms > 0.0 && offset_ms > maximum_point_time_ms))
            {
                result.error = "PointCloud2 field 't' exceeds maximum_point_time_ms";
                result.cloud->clear();
                return result;
            }
        }

        pcl::PointXYZINormal output{};
        output.x = x;
        output.y = y;
        output.z = z;
        output.intensity = intensity;
        output.normal_x = 0.0F;
        output.normal_y = 0.0F;
        output.normal_z = 0.0F;
        output.normal[3] = 0.0F;
        output.curvature = static_cast<float>(offset_ms);
        result.cloud->push_back(output);
        result.maximum_offset_ms = std::max(result.maximum_offset_ms, offset_ms);
    }

    if (result.cloud->empty())
    {
        result.error = "PointCloud2 contains no usable points";
        return result;
    }
    std::stable_sort(
        result.cloud->points.begin(), result.cloud->points.end(),
        [](const pcl::PointXYZINormal &lhs, const pcl::PointXYZINormal &rhs) {
            return lhs.curvature < rhs.curvature;
        });
    result.cloud->width = static_cast<uint32_t>(result.cloud->size());
    result.cloud->height = 1U;
    result.cloud->is_dense = true;
    result.success = true;
    return result;
}

double Utils::getSec(const std_msgs::msg::Header &header)
{
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1e-9;
}
builtin_interfaces::msg::Time Utils::getTime(const double &sec)
{
    builtin_interfaces::msg::Time time_msg;
    const double integral = std::floor(sec);
    time_msg.sec = static_cast<int32_t>(integral);
    time_msg.nanosec = static_cast<uint32_t>(std::llround((sec - integral) * 1e9));
    if (time_msg.nanosec >= 1000000000U)
    {
        ++time_msg.sec;
        time_msg.nanosec -= 1000000000U;
    }
    return time_msg;
}
