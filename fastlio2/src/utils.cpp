#include "utils.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <sensor_msgs/msg/point_field.hpp>

namespace
{
using PointField = sensor_msgs::msg::PointField;

const PointField *findField(
    const sensor_msgs::msg::PointCloud2 &msg, const std::string &name)
{
    const auto it = std::find_if(
        msg.fields.begin(), msg.fields.end(),
        [&name](const PointField &field) { return field.name == name; });
    return it == msg.fields.end() ? nullptr : &(*it);
}

size_t datatypeSize(uint8_t datatype)
{
    switch (datatype)
    {
    case PointField::INT8:
    case PointField::UINT8:
        return 1U;
    case PointField::INT16:
    case PointField::UINT16:
        return 2U;
    case PointField::INT32:
    case PointField::UINT32:
    case PointField::FLOAT32:
        return 4U;
    case PointField::FLOAT64:
        return 8U;
    default:
        return 0U;
    }
}

bool validateField(
    const sensor_msgs::msg::PointCloud2 &msg,
    const std::string &name,
    uint8_t datatype,
    const PointField *&field,
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
    const size_t size = datatypeSize(field->datatype);
    if (size == 0U ||
        static_cast<size_t>(field->offset) + size > static_cast<size_t>(msg.point_step))
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

bool validatePointTimeField(
    const sensor_msgs::msg::PointCloud2 &msg,
    const PointField &field,
    const std::string &mode,
    std::string &error)
{
    bool valid_datatype = false;
    if (mode == "absolute_seconds" || mode == "relative_seconds")
    {
        valid_datatype =
            field.datatype == PointField::FLOAT64 || field.datatype == PointField::FLOAT32;
    }
    else if (mode == "relative_nanoseconds")
    {
        valid_datatype = field.datatype == PointField::UINT32;
    }
    else
    {
        error =
            "point_time_mode must be absolute_seconds, relative_seconds, or relative_nanoseconds";
        return false;
    }

    const size_t size = datatypeSize(field.datatype);
    if (!valid_datatype || field.count != 1U || size == 0U ||
        static_cast<size_t>(field.offset) + size > static_cast<size_t>(msg.point_step))
    {
        error = "PointCloud2 field '" + field.name +
                "' has a datatype incompatible with point_time_mode '" + mode + "'";
        return false;
    }
    return true;
}

double readPointTime(const uint8_t *point, const PointField &field)
{
    switch (field.datatype)
    {
    case PointField::UINT32:
        return static_cast<double>(readScalar<uint32_t>(point, field.offset));
    case PointField::FLOAT32:
        return static_cast<double>(readScalar<float>(point, field.offset));
    case PointField::FLOAT64:
        return readScalar<double>(point, field.offset);
    default:
        return std::numeric_limits<double>::quiet_NaN();
    }
}

double pointTimeToOffsetMilliseconds(
    double raw_time,
    const std::string &mode,
    double header_time_seconds)
{
    if (mode == "absolute_seconds")
    {
        return (raw_time - header_time_seconds) * 1e3;
    }
    if (mode == "relative_seconds")
    {
        return raw_time * 1e3;
    }
    return raw_time * 1e-6;
}
} // namespace

PointCloudConversionResult Utils::pointCloud2ToPCL(
    const sensor_msgs::msg::PointCloud2 &msg,
    int filter_num,
    double min_range,
    double max_range,
    bool require_point_time,
    double maximum_point_time_ms,
    const std::string &point_time_field,
    const std::string &point_time_mode)
{
    PointCloudConversionResult result;
    if (filter_num < 1)
    {
        result.error = "lidar_filter_num must be at least one";
        return result;
    }
    if (!std::isfinite(min_range) || !std::isfinite(max_range) ||
        min_range < 0.0 || max_range <= min_range)
    {
        result.error = "invalid lidar range limits";
        return result;
    }
    if (!std::isfinite(maximum_point_time_ms) || maximum_point_time_ms <= 0.0)
    {
        result.error = "maximum_point_time_ms must be finite and positive";
        return result;
    }
    if (point_time_field.empty())
    {
        result.error = "point_time_field must not be empty";
        return result;
    }
    if (point_time_mode != "absolute_seconds" &&
        point_time_mode != "relative_seconds" &&
        point_time_mode != "relative_nanoseconds")
    {
        result.error =
            "point_time_mode must be absolute_seconds, relative_seconds, or relative_nanoseconds";
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
    if (static_cast<size_t>(msg.width) >
        std::numeric_limits<size_t>::max() / static_cast<size_t>(msg.height))
    {
        result.error = "PointCloud2 dimensions overflow";
        return result;
    }

    const size_t row_payload_size =
        static_cast<size_t>(msg.point_step) * static_cast<size_t>(msg.width);
    const size_t minimum_size =
        static_cast<size_t>(msg.row_step) * static_cast<size_t>(msg.height);
    if (static_cast<size_t>(msg.row_step) < row_payload_size ||
        msg.data.size() < minimum_size)
    {
        result.error = "malformed PointCloud2 storage";
        return result;
    }

    const PointField *x_field = nullptr;
    const PointField *y_field = nullptr;
    const PointField *z_field = nullptr;
    const PointField *intensity_field = nullptr;
    if (!validateField(msg, "x", PointField::FLOAT32, x_field, result.error) ||
        !validateField(msg, "y", PointField::FLOAT32, y_field, result.error) ||
        !validateField(msg, "z", PointField::FLOAT32, z_field, result.error) ||
        !validateField(
            msg, "intensity", PointField::FLOAT32, intensity_field, result.error))
    {
        return result;
    }

    const PointField *time_field = findField(msg, point_time_field);
    if (time_field == nullptr)
    {
        if (require_point_time)
        {
            result.error = "missing PointCloud2 field '" + point_time_field + "'";
            return result;
        }
    }
    else if (!validatePointTimeField(msg, *time_field, point_time_mode, result.error))
    {
        return result;
    }

    const double header_time_seconds = getSec(msg.header);
    if (!std::isfinite(header_time_seconds))
    {
        result.error = "PointCloud2 header stamp is not finite";
        return result;
    }

    const size_t point_count = static_cast<size_t>(msg.width) * msg.height;
    result.cloud->reserve(point_count / static_cast<size_t>(filter_num) + 1U);
    const double min_range_sq = min_range * min_range;
    const double max_range_sq = max_range * max_range;
    constexpr double negative_time_tolerance_ms = 0.1;

    for (size_t index = 0; index < point_count; ++index)
    {
        const size_t row = index / msg.width;
        const size_t column = index % msg.width;
        const uint8_t *point =
            msg.data.data() + row * msg.row_step + column * msg.point_step;

        double offset_ms = 0.0;
        if (time_field != nullptr)
        {
            const double raw_time = readPointTime(point, *time_field);
            offset_ms = pointTimeToOffsetMilliseconds(
                raw_time, point_time_mode, header_time_seconds);
            if (!std::isfinite(offset_ms) ||
                offset_ms < -negative_time_tolerance_ms ||
                offset_ms > maximum_point_time_ms)
            {
                result.error =
                    "PointCloud2 field '" + point_time_field +
                    "' is outside the configured scan time range";
                result.cloud->clear();
                return result;
            }
            offset_ms = std::max(0.0, offset_ms);
            // Scan end must use all structurally valid timestamps, including
            // points later removed by decimation or range filtering.
            result.maximum_offset_ms =
                std::max(result.maximum_offset_ms, offset_ms);
        }

        if (index % static_cast<size_t>(filter_num) != 0U)
        {
            continue;
        }

        const float x = readScalar<float>(point, x_field->offset);
        const float y = readScalar<float>(point, y_field->offset);
        const float z = readScalar<float>(point, z_field->offset);
        const float intensity = readScalar<float>(point, intensity_field->offset);
        if (!std::isfinite(x) || !std::isfinite(y) ||
            !std::isfinite(z) || !std::isfinite(intensity))
        {
            continue;
        }
        const double squared_range =
            static_cast<double>(x) * x +
            static_cast<double>(y) * y +
            static_cast<double>(z) * z;
        if (squared_range < min_range_sq || squared_range > max_range_sq)
        {
            continue;
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
    return static_cast<double>(header.stamp.sec) +
           static_cast<double>(header.stamp.nanosec) * 1e-9;
}

builtin_interfaces::msg::Time Utils::getTime(const double &sec)
{
    builtin_interfaces::msg::Time time_msg;
    const double integral = std::floor(sec);
    time_msg.sec = static_cast<int32_t>(integral);
    time_msg.nanosec =
        static_cast<uint32_t>(std::llround((sec - integral) * 1e9));
    if (time_msg.nanosec >= 1000000000U)
    {
        ++time_msg.sec;
        time_msg.nanosec -= 1000000000U;
    }
    return time_msg;
}
