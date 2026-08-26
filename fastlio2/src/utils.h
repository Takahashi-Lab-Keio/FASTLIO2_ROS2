#pragma once
#include <iomanip>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <memory>
#include <string>

#define RESET "\033[0m"
#define BLACK "\033[30m"  /* Black */
#define RED "\033[31m"    /* Red */
#define GREEN "\033[32m"  /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m"   /* Blue */
#define PURPLE "\033[35m" /* Purple */
#define CYAN "\033[36m"   /* Cyan */
#define WHITE "\033[37m"  /* White */

struct PointCloudConversionResult
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud =
        std::make_shared<pcl::PointCloud<pcl::PointXYZINormal>>();
    bool success = false;
    std::string error;
    double maximum_offset_ms = 0.0;
};

class Utils
{
public:
    static double getSec(const std_msgs::msg::Header &header);

    // Convert a PointCloud2 into FAST-LIO's point type. Supported time modes:
    // - absolute_seconds: FLOAT64/FLOAT32 seconds in the same epoch as header.stamp
    // - relative_nanoseconds: UINT32 nanoseconds from header.stamp
    // - relative_seconds: FLOAT64/FLOAT32 seconds from header.stamp
    // The relative offset is stored in PointXYZINormal::curvature in milliseconds.
    static PointCloudConversionResult pointCloud2ToPCL(
        const sensor_msgs::msg::PointCloud2 &msg,
        int filter_num,
        double min_range = 0.5,
        double max_range = 20.0,
        bool require_point_time = true,
        double maximum_point_time_ms = 200.0,
        const std::string &point_time_field = "timestamp",
        const std::string &point_time_mode = "absolute_seconds");

    static builtin_interfaces::msg::Time getTime(const double &sec);
};
