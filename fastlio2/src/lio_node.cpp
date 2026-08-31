#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>

#include "map_builder/commons.h"
#include "map_builder/map_builder.h"
#include "output_transforms.h"
#include "runtime_overrides.h"
#include "utils.h"

using namespace std::chrono_literals;

namespace
{
struct NodeConfig
{
    std::string imu_topic = "/lidar_imu";
    std::string lidar_topic = "/lidar_points";
    std::string body_frame = "base_footprint";
    std::string lidar_frame = "hesai_lidar";
    std::string imu_frame = "hesai_lidar";
    std::string world_frame = "fastlio_odom";
    std::string point_time_field = "timestamp";
    std::string point_time_mode = "absolute_seconds";
    bool print_time_cost = false;
    bool require_point_time = true;
    bool anchor_output_frame = true;
    bool publish_tf = true;
    double maximum_point_time_ms = 200.0;
    double imu_acc_scale = 1.0;
    double maximum_imu_gap_seconds = 0.1;
    double maximum_lidar_gap_seconds = 1.0;
    size_t maximum_imu_queue_size = 4000;
    size_t maximum_lidar_queue_size = 20;

    // p_base = r_bi * p_imu + t_bi
    M3D r_bi = M3D::Identity();
    V3D t_bi = V3D::Zero();

    // Optional fixed LiDAR mounting from the URDF:
    // p_base = r_bl * p_lidar + t_bl. Prefer this for integrated LiDAR/IMU
    // units whose online LiDAR-to-IMU extrinsic is estimated.
    bool use_base_lidar_extrinsic = false;
    M3D r_bl = M3D::Identity();
    V3D t_bl = V3D::Zero();
};

struct StateData
{
    bool lidar_pushed = false;
    bool anchor_initialized = false;
    std::mutex imu_mutex;
    std::mutex lidar_mutex;
    double last_lidar_time = -std::numeric_limits<double>::infinity();
    double last_imu_time = -std::numeric_limits<double>::infinity();
    std::deque<IMUData> imu_buffer;
    struct LidarMeasurement
    {
        double start_time = 0.0;
        double end_time = 0.0;
        CloudType::Ptr cloud;
    };
    std::deque<LidarMeasurement> lidar_buffer;
    nav_msgs::msg::Path path;

    // p_output = r_ow * p_internal_world + t_ow
    M3D r_ow = M3D::Identity();
    V3D t_ow = V3D::Zero();
};

M3D matrixFromRowMajor(const std::vector<double> &values, const std::string &name)
{
    if (values.size() != 9U)
    {
        throw std::runtime_error(name + " must contain nine row-major values");
    }
    M3D matrix;
    matrix << values[0], values[1], values[2],
        values[3], values[4], values[5],
        values[6], values[7], values[8];
    if (!matrix.allFinite() ||
        std::abs(matrix.determinant() - 1.0) > 1e-3 ||
        !(matrix.transpose() * matrix).isApprox(M3D::Identity(), 1e-3))
    {
        throw std::runtime_error(name + " must be a finite rotation matrix");
    }
    return matrix;
}

V3D vector3FromYaml(const std::vector<double> &values, const std::string &name)
{
    if (values.size() != 3U)
    {
        throw std::runtime_error(name + " must contain three values");
    }
    const V3D vector(values[0], values[1], values[2]);
    if (!vector.allFinite())
    {
        throw std::runtime_error(name + " must be finite");
    }
    return vector;
}
} // namespace

class LIONode : public rclcpp::Node
{
public:
    LIONode() : Node("lio_node")
    {
        loadParameters();

        const auto imu_qos = rclcpp::QoS(200).reliable().durability_volatile();
        const auto lidar_qos = rclcpp::QoS(10).reliable().durability_volatile();
        m_imu_sub = create_subscription<sensor_msgs::msg::Imu>(
            m_node_config.imu_topic, imu_qos,
            std::bind(&LIONode::imuCB, this, std::placeholders::_1));
        m_lidar_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
            m_node_config.lidar_topic, lidar_qos,
            std::bind(&LIONode::lidarCB, this, std::placeholders::_1));

        m_body_cloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud", 10);
        m_world_cloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>("world_cloud", 10);
        m_path_pub = create_publisher<nav_msgs::msg::Path>("lio_path", 10);
        m_odom_pub = create_publisher<nav_msgs::msg::Odometry>("lio_odom", 10);
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        m_state_data.path.header.frame_id = m_node_config.world_frame;
        m_kf = std::make_shared<IESKF>();
        m_kf->setMaxIter(static_cast<size_t>(m_builder_config.ieskf_max_iter));
        m_builder = std::make_shared<MapBuilder>(m_builder_config, m_kf);
        m_timer = create_wall_timer(20ms, std::bind(&LIONode::timerCB, this));

        RCLCPP_INFO(
            get_logger(),
            "FAST-LIO2 PointCloud2 input ready: points=%s imu=%s output=%s->%s time=%s/%s imu_acc_scale=%.6f",
            m_node_config.lidar_topic.c_str(), m_node_config.imu_topic.c_str(),
            m_node_config.world_frame.c_str(), m_node_config.body_frame.c_str(),
            m_node_config.point_time_field.c_str(), m_node_config.point_time_mode.c_str(),
            m_node_config.imu_acc_scale);
    }

private:
    void loadParameters()
    {
        declare_parameter<std::string>("config_path", "");
        const std::string config_path = get_parameter("config_path").as_string();
        if (config_path.empty())
        {
            throw std::runtime_error("config_path is required");
        }

        const YAML::Node config = YAML::LoadFile(config_path);
        if (!config || !config.IsMap())
        {
            throw std::runtime_error("failed to load FAST-LIO2 YAML config: " + config_path);
        }
        RCLCPP_INFO(get_logger(), "Loading configuration from %s", config_path.c_str());

        m_node_config.imu_topic = config["imu_topic"].as<std::string>(m_node_config.imu_topic);
        m_node_config.lidar_topic = config["lidar_topic"].as<std::string>(m_node_config.lidar_topic);
        m_node_config.body_frame = config["body_frame"].as<std::string>(m_node_config.body_frame);
        m_node_config.lidar_frame = config["lidar_frame"].as<std::string>(m_node_config.lidar_frame);
        m_node_config.imu_frame = config["imu_frame"].as<std::string>(m_node_config.imu_frame);
        m_node_config.world_frame = config["world_frame"].as<std::string>(m_node_config.world_frame);
        m_node_config.point_time_field = config["point_time_field"].as<std::string>(m_node_config.point_time_field);
        m_node_config.point_time_mode = config["point_time_mode"].as<std::string>(m_node_config.point_time_mode);
        m_node_config.print_time_cost = config["print_time_cost"].as<bool>(false);
        m_node_config.require_point_time = config["require_point_time"].as<bool>(true);
        m_node_config.anchor_output_frame = config["anchor_output_frame"].as<bool>(true);
        m_node_config.publish_tf = config["publish_tf"].as<bool>(true);
        m_node_config.maximum_point_time_ms = config["maximum_point_time_ms"].as<double>(200.0);
        m_node_config.maximum_imu_gap_seconds =
            config["maximum_imu_gap_seconds"].as<double>(0.1);
        m_node_config.maximum_lidar_gap_seconds =
            config["maximum_lidar_gap_seconds"].as<double>(1.0);
        m_node_config.maximum_imu_queue_size =
            config["maximum_imu_queue_size"].as<size_t>(4000U);
        m_node_config.maximum_lidar_queue_size =
            config["maximum_lidar_queue_size"].as<size_t>(20U);
        m_node_config.imu_acc_scale = config["imu_acc_scale"].as<double>(1.0);

        if (config["r_bi"])
        {
            m_node_config.r_bi = matrixFromRowMajor(config["r_bi"].as<std::vector<double>>(), "r_bi");
        }
        if (config["t_bi"])
        {
            m_node_config.t_bi = vector3FromYaml(config["t_bi"].as<std::vector<double>>(), "t_bi");
        }
        const bool has_r_bl = static_cast<bool>(config["r_bl"]);
        const bool has_t_bl = static_cast<bool>(config["t_bl"]);
        if (has_r_bl != has_t_bl)
        {
            throw std::runtime_error("r_bl and t_bl must be configured together");
        }
        if (has_r_bl)
        {
            m_node_config.r_bl = matrixFromRowMajor(
                config["r_bl"].as<std::vector<double>>(), "r_bl");
            m_node_config.t_bl = vector3FromYaml(
                config["t_bl"].as<std::vector<double>>(), "t_bl");
            m_node_config.use_base_lidar_extrinsic = true;
        }

        // ROS parameter overrides take precedence over YAML; defaults come from YAML.
        const std::string world_frame_override = declare_parameter<std::string>(
            "world_frame_override", "");
        m_node_config.world_frame = FastlioRuntimeOverrides::selectFrame(
            m_node_config.world_frame, world_frame_override);
        m_node_config.require_point_time = declare_parameter<bool>(
            "require_point_time", m_node_config.require_point_time);
        const bool allow_missing_point_time = declare_parameter<bool>(
            "allow_missing_point_time", false);
        m_node_config.require_point_time = FastlioRuntimeOverrides::requirePointTime(
            m_node_config.require_point_time, allow_missing_point_time);
        m_node_config.anchor_output_frame = declare_parameter<bool>(
            "anchor_output_frame", m_node_config.anchor_output_frame);
        m_node_config.publish_tf = declare_parameter<bool>(
            "publish_tf", m_node_config.publish_tf);
        m_node_config.imu_acc_scale = declare_parameter<double>(
            "imu_acc_scale", m_node_config.imu_acc_scale);
        m_node_config.maximum_point_time_ms = declare_parameter<double>(
            "maximum_point_time_ms", m_node_config.maximum_point_time_ms);

        if (!world_frame_override.empty())
        {
            RCLCPP_INFO(
                get_logger(), "Overriding FAST-LIO2 world frame with '%s'",
                m_node_config.world_frame.c_str());
        }
        if (allow_missing_point_time)
        {
            RCLCPP_WARN(
                get_logger(),
                "Point time is optional; clouds without '%s' are treated as instantaneous scans",
                m_node_config.point_time_field.c_str());
        }

        if (m_node_config.imu_topic.empty() || m_node_config.lidar_topic.empty() ||
            m_node_config.body_frame.empty() || m_node_config.lidar_frame.empty() || m_node_config.imu_frame.empty() ||
            m_node_config.world_frame.empty() || m_node_config.point_time_field.empty())
        {
            throw std::runtime_error("topics and frame IDs must not be empty");
        }
        if (!std::isfinite(m_node_config.imu_acc_scale) || m_node_config.imu_acc_scale <= 0.0)
        {
            throw std::runtime_error("imu_acc_scale must be finite and positive");
        }
        if (!std::isfinite(m_node_config.maximum_point_time_ms) || m_node_config.maximum_point_time_ms <= 0.0)
        {
            throw std::runtime_error("maximum_point_time_ms must be finite and positive");
        }
        if (!std::isfinite(m_node_config.maximum_imu_gap_seconds) ||
            !std::isfinite(m_node_config.maximum_lidar_gap_seconds) ||
            m_node_config.maximum_imu_gap_seconds <= 0.0 ||
            m_node_config.maximum_lidar_gap_seconds <= 0.0 ||
            m_node_config.maximum_imu_queue_size < 2U ||
            m_node_config.maximum_lidar_queue_size < 1U)
        {
            throw std::runtime_error(
                "sensor gap limits must be positive and queue limits must be non-zero");
        }
        if (m_node_config.point_time_mode != "absolute_seconds" &&
            m_node_config.point_time_mode != "relative_seconds" &&
            m_node_config.point_time_mode != "relative_nanoseconds")
        {
            throw std::runtime_error(
                "point_time_mode must be absolute_seconds, relative_seconds, or relative_nanoseconds");
        }

        m_builder_config.lidar_filter_num = config["lidar_filter_num"].as<int>();
        m_builder_config.lidar_min_range = config["lidar_min_range"].as<double>();
        m_builder_config.lidar_max_range = config["lidar_max_range"].as<double>();
        m_builder_config.scan_resolution = config["scan_resolution"].as<double>();
        m_builder_config.map_resolution = config["map_resolution"].as<double>();
        m_builder_config.cube_len = config["cube_len"].as<double>();
        m_builder_config.det_range = config["det_range"].as<double>();
        m_builder_config.move_thresh = config["move_thresh"].as<double>();
        m_builder_config.na = config["na"].as<double>();
        m_builder_config.ng = config["ng"].as<double>();
        m_builder_config.nba = config["nba"].as<double>();
        m_builder_config.nbg = config["nbg"].as<double>();
        m_builder_config.imu_init_num = config["imu_init_num"].as<int>();
        m_builder_config.imu_init_accel_min =
            config["imu_init_accel_min"].as<double>(5.0);
        m_builder_config.imu_init_accel_max =
            config["imu_init_accel_max"].as<double>(15.0);
        m_builder_config.near_search_num = config["near_search_num"].as<int>();
        m_builder_config.ieskf_max_iter = config["ieskf_max_iter"].as<int>();
        m_builder_config.gravity_align = config["gravity_align"].as<bool>();
        m_builder_config.esti_il = config["esti_il"].as<bool>();
        m_builder_config.r_il = matrixFromRowMajor(config["r_il"].as<std::vector<double>>(), "r_il");
        m_builder_config.t_il = vector3FromYaml(config["t_il"].as<std::vector<double>>(), "t_il");
        m_builder_config.lidar_cov_inv = config["lidar_cov_inv"].as<double>();

        if (m_builder_config.lidar_filter_num < 1 || m_builder_config.imu_init_num < 1 ||
            m_builder_config.near_search_num < 1 || m_builder_config.ieskf_max_iter < 1)
        {
            throw std::runtime_error("filter, initialization, neighbour and IESKF counts must be positive");
        }
        if (!std::isfinite(m_builder_config.imu_init_accel_min) ||
            !std::isfinite(m_builder_config.imu_init_accel_max) ||
            m_builder_config.imu_init_accel_min <= 0.0 ||
            m_builder_config.imu_init_accel_max <=
                m_builder_config.imu_init_accel_min)
        {
            throw std::runtime_error(
                "imu_init_accel_min/max must define a finite positive range");
        }
    }

    void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (msg->header.frame_id != m_node_config.imu_frame)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Dropping IMU message in frame '%s'; expected '%s'",
                msg->header.frame_id.c_str(), m_node_config.imu_frame.c_str());
            return;
        }
        const double timestamp = Utils::getSec(msg->header);
        const V3D acceleration(
            msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        const V3D angular_velocity(
            msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        if (!std::isfinite(timestamp) || !acceleration.allFinite() || !angular_velocity.allFinite())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Dropping non-finite IMU message");
            return;
        }

        std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
        const auto timestamp_status = FastlioTransforms::classifyTimestamp(
            timestamp, m_state_data.last_imu_time, 0.5,
            m_node_config.maximum_imu_gap_seconds);
        if (timestamp_status == FastlioTransforms::TimestampStatus::FATAL_REGRESSION)
        {
            RCLCPP_FATAL(
                get_logger(),
                "IMU clock regressed by %.3f s; restart lio_node before replaying a reset clock",
                m_state_data.last_imu_time - timestamp);
            rclcpp::shutdown();
            return;
        }
        if (timestamp_status == FastlioTransforms::TimestampStatus::DROP_DUPLICATE_OR_REORDERED)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Dropping out-of-order IMU message (%.9f <= %.9f)",
                timestamp, m_state_data.last_imu_time);
            return;
        }
        if (timestamp_status == FastlioTransforms::TimestampStatus::FATAL_FORWARD_GAP)
        {
            RCLCPP_FATAL(
                get_logger(),
                "IMU clock jumped forward by %.3f s (limit %.3f s); restart lio_node",
                timestamp - m_state_data.last_imu_time,
                m_node_config.maximum_imu_gap_seconds);
            rclcpp::shutdown();
            return;
        }
        while (m_state_data.imu_buffer.size() >=
               m_node_config.maximum_imu_queue_size)
        {
            m_state_data.imu_buffer.pop_front();
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "IMU queue reached its limit; dropping oldest samples while waiting for LiDAR");
        }
        m_state_data.imu_buffer.emplace_back(
            FastlioTransforms::scaleImuAcceleration(acceleration, m_node_config.imu_acc_scale),
            angular_velocity, timestamp);
        m_state_data.last_imu_time = timestamp;
    }

    void lidarCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (msg->header.frame_id != m_node_config.lidar_frame)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Dropping lidar message in frame '%s'; expected '%s'",
                msg->header.frame_id.c_str(), m_node_config.lidar_frame.c_str());
            return;
        }
        const double timestamp = Utils::getSec(msg->header);
        if (!std::isfinite(timestamp))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Dropping lidar message with non-finite stamp");
            return;
        }
        PointCloudConversionResult converted = Utils::pointCloud2ToPCL(
            *msg, m_builder_config.lidar_filter_num,
            m_builder_config.lidar_min_range, m_builder_config.lidar_max_range,
            m_node_config.require_point_time, m_node_config.maximum_point_time_ms,
            m_node_config.point_time_field, m_node_config.point_time_mode);
        if (!converted.success)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Dropping invalid PointCloud2: %s", converted.error.c_str());
            return;
        }

        std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
        const auto timestamp_status = FastlioTransforms::classifyTimestamp(
            timestamp, m_state_data.last_lidar_time, 0.5,
            m_node_config.maximum_lidar_gap_seconds);
        if (timestamp_status == FastlioTransforms::TimestampStatus::FATAL_REGRESSION)
        {
            RCLCPP_FATAL(
                get_logger(),
                "Lidar clock regressed by %.3f s; restart lio_node before replaying a reset clock",
                m_state_data.last_lidar_time - timestamp);
            rclcpp::shutdown();
            return;
        }
        if (timestamp_status == FastlioTransforms::TimestampStatus::DROP_DUPLICATE_OR_REORDERED)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Dropping out-of-order lidar message (%.9f <= %.9f)",
                timestamp, m_state_data.last_lidar_time);
            return;
        }
        if (timestamp_status == FastlioTransforms::TimestampStatus::FATAL_FORWARD_GAP)
        {
            RCLCPP_FATAL(
                get_logger(),
                "Lidar clock jumped forward by %.3f s (limit %.3f s); restart lio_node",
                timestamp - m_state_data.last_lidar_time,
                m_node_config.maximum_lidar_gap_seconds);
            rclcpp::shutdown();
            return;
        }
        m_state_data.lidar_buffer.push_back(
            {timestamp, timestamp + converted.maximum_offset_ms * 1e-3, converted.cloud});
        while (m_state_data.lidar_buffer.size() >
               m_node_config.maximum_lidar_queue_size)
        {
            m_state_data.lidar_buffer.pop_front();
            m_state_data.lidar_pushed = false;
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "LiDAR queue reached its limit; dropping oldest scans while waiting for IMU");
        }
        m_state_data.last_lidar_time = timestamp;
    }

    bool syncPackage()
    {
        std::scoped_lock lock(m_state_data.imu_mutex, m_state_data.lidar_mutex);
        if (m_state_data.imu_buffer.empty() || m_state_data.lidar_buffer.empty())
        {
            return false;
        }
        if (!m_state_data.lidar_pushed)
        {
            const auto &measurement = m_state_data.lidar_buffer.front();
            m_package.cloud = measurement.cloud;
            if (!m_package.cloud || m_package.cloud->empty())
            {
                m_state_data.lidar_buffer.pop_front();
                return false;
            }
            m_package.cloud_start_time = measurement.start_time;
            m_package.cloud_end_time = measurement.end_time;
            if (!std::isfinite(m_package.cloud_end_time) ||
                m_package.cloud_end_time < m_package.cloud_start_time)
            {
                RCLCPP_WARN(get_logger(), "Dropping cloud with invalid point time range");
                m_state_data.lidar_buffer.pop_front();
                return false;
            }
            m_state_data.lidar_pushed = true;
        }
        if (m_state_data.last_imu_time < m_package.cloud_end_time)
        {
            return false;
        }

        m_package.imus.clear();
        while (!m_state_data.imu_buffer.empty() &&
               m_state_data.imu_buffer.front().time <= m_package.cloud_end_time)
        {
            m_package.imus.emplace_back(m_state_data.imu_buffer.front());
            m_state_data.imu_buffer.pop_front();
        }
        m_state_data.lidar_buffer.pop_front();
        m_state_data.lidar_pushed = false;
        if (m_package.imus.empty())
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Dropping cloud without an IMU sample at or before scan end");
            return false;
        }
        return true;
    }

    void initializeOutputAnchor()
    {
        if (m_state_data.anchor_initialized)
        {
            return;
        }
        if (m_node_config.anchor_output_frame)
        {
            const auto base_pose = internalBasePose();
            const auto anchor = FastlioTransforms::inverse(base_pose);
            m_state_data.r_ow = anchor.rotation;
            m_state_data.t_ow = anchor.translation;
        }
        m_state_data.anchor_initialized = true;
    }

    std::pair<M3D, V3D> outputBasePose() const
    {
        const auto base_pose = internalBasePose();
        const auto output_pose = FastlioTransforms::compose(
            {m_state_data.r_ow, m_state_data.t_ow}, base_pose);
        return {output_pose.rotation, output_pose.translation};
    }

    FastlioTransforms::RigidPose internalBasePose() const
    {
        if (m_node_config.use_base_lidar_extrinsic)
        {
            return FastlioTransforms::lidarPoseToBasePose(
                m_kf->x().r_wi, m_kf->x().t_wi,
                m_kf->x().r_il, m_kf->x().t_il,
                m_node_config.r_bl, m_node_config.t_bl);
        }
        return FastlioTransforms::imuPoseToBasePose(
            m_kf->x().r_wi, m_kf->x().t_wi,
            m_node_config.r_bi, m_node_config.t_bi);
    }

    void publishCloud(
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher,
        const CloudType::Ptr &cloud, const std::string &frame_id, double time)
    {
        if (publisher->get_subscription_count() == 0U || !cloud || cloud->empty())
        {
            return;
        }
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = frame_id;
        cloud_msg.header.stamp = Utils::getTime(time);
        publisher->publish(cloud_msg);
    }

    void publishOdometry(double time)
    {
        if (m_odom_pub->get_subscription_count() == 0U)
        {
            return;
        }
        const auto [rotation, translation] = outputBasePose();
        Eigen::Quaterniond quaternion(rotation);
        quaternion.normalize();
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = m_node_config.world_frame;
        odom.child_frame_id = m_node_config.body_frame;
        odom.header.stamp = Utils::getTime(time);
        odom.pose.pose.position.x = translation.x();
        odom.pose.pose.position.y = translation.y();
        odom.pose.pose.position.z = translation.z();
        odom.pose.pose.orientation.x = quaternion.x();
        odom.pose.pose.orientation.y = quaternion.y();
        odom.pose.pose.orientation.z = quaternion.z();
        odom.pose.pose.orientation.w = quaternion.w();
        const V3D velocity_base = rotation.transpose() * m_state_data.r_ow * m_kf->x().v;
        odom.twist.twist.linear.x = velocity_base.x();
        odom.twist.twist.linear.y = velocity_base.y();
        odom.twist.twist.linear.z = velocity_base.z();
        m_odom_pub->publish(odom);
    }

    void publishPath(double time)
    {
        if (m_path_pub->get_subscription_count() == 0U)
        {
            return;
        }
        const auto [rotation, translation] = outputBasePose();
        Eigen::Quaterniond quaternion(rotation);
        quaternion.normalize();
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = m_node_config.world_frame;
        pose.header.stamp = Utils::getTime(time);
        pose.pose.position.x = translation.x();
        pose.pose.position.y = translation.y();
        pose.pose.position.z = translation.z();
        pose.pose.orientation.x = quaternion.x();
        pose.pose.orientation.y = quaternion.y();
        pose.pose.orientation.z = quaternion.z();
        pose.pose.orientation.w = quaternion.w();
        m_state_data.path.header.stamp = pose.header.stamp;
        m_state_data.path.poses.push_back(pose);
        m_path_pub->publish(m_state_data.path);
    }

    void broadcastTF(double time)
    {
        const auto [rotation, translation] = outputBasePose();
        Eigen::Quaterniond quaternion(rotation);
        quaternion.normalize();
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = m_node_config.world_frame;
        transform.child_frame_id = m_node_config.body_frame;
        transform.header.stamp = Utils::getTime(time);
        transform.transform.translation.x = translation.x();
        transform.transform.translation.y = translation.y();
        transform.transform.translation.z = translation.z();
        transform.transform.rotation.x = quaternion.x();
        transform.transform.rotation.y = quaternion.y();
        transform.transform.rotation.z = quaternion.z();
        transform.transform.rotation.w = quaternion.w();
        m_tf_broadcaster->sendTransform(transform);
    }

    void timerCB()
    {
        if (!syncPackage())
        {
            return;
        }
        const auto start = std::chrono::high_resolution_clock::now();
        try
        {
            m_builder->process(m_package);
        }
        catch (const std::exception &error)
        {
            RCLCPP_FATAL(
                get_logger(), "FAST-LIO2 processing failed: %s", error.what());
            rclcpp::shutdown();
            return;
        }
        const auto finish = std::chrono::high_resolution_clock::now();
        if (m_node_config.print_time_cost)
        {
            const double milliseconds =
                std::chrono::duration_cast<std::chrono::duration<double>>(finish - start).count() * 1000.0;
            RCLCPP_INFO(get_logger(), "Processing time: %.2f ms", milliseconds);
        }
        if (m_builder->status() != BuilderStatus::MAPPING)
        {
            return;
        }

        initializeOutputAnchor();
        if (m_node_config.publish_tf)
        {
            broadcastTF(m_package.cloud_end_time);
        }
        publishOdometry(m_package.cloud_end_time);

        const auto lidar_to_base = m_node_config.use_base_lidar_extrinsic
                                       ? FastlioTransforms::RigidPose{
                                             m_node_config.r_bl,
                                             m_node_config.t_bl}
                                       : FastlioTransforms::lidarToBasePose(
                                             m_kf->x().r_il, m_kf->x().t_il,
                                             m_node_config.r_bi, m_node_config.t_bi);
        const CloudType::Ptr body_cloud =
            LidarProcessor::transformCloud(
                m_package.cloud, lidar_to_base.rotation, lidar_to_base.translation);
        publishCloud(m_body_cloud_pub, body_cloud, m_node_config.body_frame, m_package.cloud_end_time);

        const M3D r_ol = m_state_data.r_ow * m_builder->lidar_processor()->r_wl();
        const V3D t_ol = m_state_data.r_ow * m_builder->lidar_processor()->t_wl() + m_state_data.t_ow;
        const CloudType::Ptr world_cloud =
            LidarProcessor::transformCloud(m_package.cloud, r_ol, t_ol);
        publishCloud(m_world_cloud_pub, world_cloud, m_node_config.world_frame, m_package.cloud_end_time);
        publishPath(m_package.cloud_end_time);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_lidar_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_world_cloud_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;
    rclcpp::TimerBase::SharedPtr m_timer;
    StateData m_state_data;
    SyncPackage m_package;
    NodeConfig m_node_config;
    Config m_builder_config;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<MapBuilder> m_builder;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LIONode>());
    rclcpp::shutdown();
    return 0;
}
