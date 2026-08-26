#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/point_tests.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>

#include "fastlio2_interfaces/srv/is_valid.hpp"
#include "fastlio2_interfaces/srv/relocalize.hpp"
#include "localizers/commons.h"
#include "localizers/icp_localizer.h"

using namespace std::chrono_literals;

namespace
{
struct NodeConfig
{
    std::string cloud_topic = "/fastlio2/body_cloud";
    std::string odom_topic = "/fastlio2/lio_odom";
    std::string map_frame = "map";
    std::string local_frame = "fastlio_odom";
    std::string body_frame = "base_footprint";
    double update_hz = 1.0;
};

struct NodeState
{
    std::mutex message_mutex;
    std::mutex service_mutex;

    bool message_received = false;
    bool service_received = false;
    bool localize_success = false;
    double last_update_seconds = -std::numeric_limits<double>::infinity();
    builtin_interfaces::msg::Time last_message_time;
    CloudType::Ptr last_cloud = std::make_shared<CloudType>();
    M3D last_r = M3D::Identity();         // T_local_body rotation
    V3D last_t = V3D::Zero();             // T_local_body translation
    M3D last_offset_r = M3D::Identity();  // T_map_local rotation
    V3D last_offset_t = V3D::Zero();      // T_map_local translation
    M4F initial_guess = M4F::Identity();  // T_map_body
};

bool isFinitePose(const geometry_msgs::msg::Pose &pose)
{
    return std::isfinite(pose.position.x) &&
           std::isfinite(pose.position.y) &&
           std::isfinite(pose.position.z) &&
           std::isfinite(pose.orientation.x) &&
           std::isfinite(pose.orientation.y) &&
           std::isfinite(pose.orientation.z) &&
           std::isfinite(pose.orientation.w);
}

M4F poseMatrix(const M3D &rotation, const V3D &translation)
{
    M4F matrix = M4F::Identity();
    matrix.block<3, 3>(0, 0) = rotation.cast<float>();
    matrix.block<3, 1>(0, 3) = translation.cast<float>();
    return matrix;
}
} // namespace

class LocalizerNode : public rclcpp::Node
{
public:
    LocalizerNode() : Node("localizer_node")
    {
        loadParameters();

        const auto input_qos =
            rclcpp::QoS(10).reliable().durability_volatile().get_rmw_qos_profile();
        m_cloud_sub.subscribe(this, m_config.cloud_topic, input_qos);
        m_odom_sub.subscribe(this, m_config.odom_topic, input_qos);

        m_sync = std::make_shared<Synchronizer>(
            SyncPolicy(10), m_cloud_sub, m_odom_sub);
        m_sync->setAgePenalty(0.1);
        m_sync->registerCallback(
            std::bind(
                &LocalizerNode::syncCB, this,
                std::placeholders::_1, std::placeholders::_2));

        m_localizer = std::make_shared<ICPLocalizer>(m_localizer_config);
        m_tf_broadcaster =
            std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        m_reloc_srv =
            create_service<fastlio2_interfaces::srv::Relocalize>(
                "relocalize",
                std::bind(
                    &LocalizerNode::relocCB, this,
                    std::placeholders::_1, std::placeholders::_2));
        m_reloc_check_srv =
            create_service<fastlio2_interfaces::srv::IsValid>(
                "relocalize_check",
                std::bind(
                    &LocalizerNode::relocCheckCB, this,
                    std::placeholders::_1, std::placeholders::_2));

        m_initial_pose_sub =
            create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/initialpose", rclcpp::QoS(10).reliable(),
                std::bind(
                    &LocalizerNode::initialPoseCB, this,
                    std::placeholders::_1));

        m_map_cloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>(
            "map_cloud",
            rclcpp::QoS(1).reliable().transient_local());
        m_pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>(
            "pose", rclcpp::QoS(10).reliable());

        m_timer = create_wall_timer(10ms, std::bind(&LocalizerNode::timerCB, this));

        if (!m_startup_map_path.empty())
        {
            std::string error;
            if (!loadMap(m_startup_map_path, error))
            {
                throw std::runtime_error(
                    "failed to preload map '" + m_startup_map_path + "': " + error);
            }
            RCLCPP_INFO(
                get_logger(), "Preloaded prior map: %s",
                m_startup_map_path.c_str());
        }

        if (m_initialize_from_parameters)
        {
            if (!mapLoaded())
            {
                throw std::runtime_error(
                    "initialize_from_parameters requires a non-empty map_path");
            }
            queueInitialGuess(initialGuessFromParameters());
            RCLCPP_INFO(
                get_logger(),
                "Queued startup initial pose: xyz=[%.3f %.3f %.3f], rpy=[%.3f %.3f %.3f] rad",
                m_initial_x, m_initial_y, m_initial_z,
                m_initial_roll, m_initial_pitch, m_initial_yaw);
        }

        RCLCPP_INFO(
            get_logger(),
            "FAST-LIO2 localizer ready; set /initialpose or call relocalize");
    }

private:
    using SyncPolicy =
        message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

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
            throw std::runtime_error(
                "failed to load localizer YAML config: " + config_path);
        }
        RCLCPP_INFO(
            get_logger(), "Loading configuration from %s", config_path.c_str());

        m_config.cloud_topic =
            config["cloud_topic"].as<std::string>(m_config.cloud_topic);
        m_config.odom_topic =
            config["odom_topic"].as<std::string>(m_config.odom_topic);
        m_config.map_frame =
            config["map_frame"].as<std::string>(m_config.map_frame);
        m_config.local_frame =
            config["local_frame"].as<std::string>(m_config.local_frame);
        m_config.body_frame =
            config["body_frame"].as<std::string>(m_config.body_frame);
        m_config.update_hz =
            config["update_hz"].as<double>(m_config.update_hz);

        m_localizer_config.rough_scan_resolution =
            config["rough_scan_resolution"].as<double>();
        m_localizer_config.rough_map_resolution =
            config["rough_map_resolution"].as<double>();
        m_localizer_config.rough_max_iteration =
            config["rough_max_iteration"].as<int>();
        m_localizer_config.rough_score_thresh =
            config["rough_score_thresh"].as<double>();
        m_localizer_config.rough_max_correspondence_distance =
            config["rough_max_correspondence_distance"].as<double>(2.0);

        m_localizer_config.refine_scan_resolution =
            config["refine_scan_resolution"].as<double>();
        m_localizer_config.refine_map_resolution =
            config["refine_map_resolution"].as<double>();
        m_localizer_config.refine_max_iteration =
            config["refine_max_iteration"].as<int>();
        m_localizer_config.refine_score_thresh =
            config["refine_score_thresh"].as<double>();
        m_localizer_config.refine_max_correspondence_distance =
            config["refine_max_correspondence_distance"].as<double>(0.5);

        if (m_config.cloud_topic.empty() || m_config.odom_topic.empty() ||
            m_config.map_frame.empty() || m_config.local_frame.empty() ||
            m_config.body_frame.empty())
        {
            throw std::runtime_error("topics and frame IDs must not be empty");
        }
        if (!std::isfinite(m_config.update_hz) || m_config.update_hz <= 0.0)
        {
            throw std::runtime_error("update_hz must be finite and positive");
        }
        if (m_localizer_config.rough_max_iteration < 1 ||
            m_localizer_config.refine_max_iteration < 1 ||
            !std::isfinite(
                m_localizer_config.rough_max_correspondence_distance) ||
            !std::isfinite(
                m_localizer_config.refine_max_correspondence_distance) ||
            m_localizer_config.rough_max_correspondence_distance <= 0.0 ||
            m_localizer_config.refine_max_correspondence_distance <= 0.0)
        {
            throw std::runtime_error(
                "ICP iterations and correspondence distances must be positive");
        }

        m_startup_map_path =
            declare_parameter<std::string>("map_path", "");
        m_initialize_from_parameters =
            declare_parameter<bool>("initialize_from_parameters", false);
        m_initial_x = declare_parameter<double>("initial_x", 0.0);
        m_initial_y = declare_parameter<double>("initial_y", 0.0);
        m_initial_z = declare_parameter<double>("initial_z", 0.0);
        m_initial_roll = declare_parameter<double>("initial_roll", 0.0);
        m_initial_pitch = declare_parameter<double>("initial_pitch", 0.0);
        m_initial_yaw = declare_parameter<double>("initial_yaw", 0.0);

        if (!std::isfinite(m_initial_x) || !std::isfinite(m_initial_y) ||
            !std::isfinite(m_initial_z) || !std::isfinite(m_initial_roll) ||
            !std::isfinite(m_initial_pitch) || !std::isfinite(m_initial_yaw))
        {
            throw std::runtime_error("startup initial pose must be finite");
        }
    }

    M4F initialGuessFromParameters() const
    {
        const Eigen::AngleAxisd yaw(
            m_initial_yaw, Eigen::Vector3d::UnitZ());
        const Eigen::AngleAxisd pitch(
            m_initial_pitch, Eigen::Vector3d::UnitY());
        const Eigen::AngleAxisd roll(
            m_initial_roll, Eigen::Vector3d::UnitX());
        return poseMatrix(
            (yaw * pitch * roll).toRotationMatrix(),
            V3D(m_initial_x, m_initial_y, m_initial_z));
    }

    bool mapLoaded()
    {
        std::lock_guard<std::mutex> lock(m_map_mutex);
        return m_map_loaded;
    }

    bool loadMap(const std::string &path, std::string &error)
    {
        if (path.empty())
        {
            error = "pcd_path is empty and no preloaded map was requested";
            return false;
        }
        if (!std::filesystem::exists(path) ||
            !std::filesystem::is_regular_file(path))
        {
            error = "PCD file not found";
            return false;
        }

        {
            std::lock_guard<std::mutex> lock(m_localizer_mutex);
            if (!m_localizer->loadMap(path))
            {
                error = "PCD file is unreadable, empty, or invalid";
                return false;
            }
        }
        {
            std::lock_guard<std::mutex> lock(m_map_mutex);
            m_map_loaded = true;
            m_loaded_map_path = path;
        }
        publishMapCloud();
        return true;
    }

    void queueInitialGuess(const M4F &guess)
    {
        std::lock_guard<std::mutex> lock(m_state.service_mutex);
        m_state.initial_guess = guess;
        m_state.service_received = true;
        m_state.localize_success = false;
    }

    void timerCB()
    {
        bool request_pending;
        bool localized;
        {
            std::lock_guard<std::mutex> lock(m_state.service_mutex);
            request_pending = m_state.service_received;
            localized = m_state.localize_success;
        }
        if (!request_pending && !localized)
        {
            return;
        }

        CloudType::Ptr current_cloud;
        M3D current_local_r;
        V3D current_local_t;
        builtin_interfaces::msg::Time current_time;
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            if (!m_state.message_received)
            {
                return;
            }
            current_cloud = m_state.last_cloud;
            current_local_r = m_state.last_r;
            current_local_t = m_state.last_t;
            current_time = m_state.last_message_time;
        }

        const double current_clock_seconds = now().seconds();
        if (current_clock_seconds - m_state.last_update_seconds <=
            (1.0 / m_config.update_hz))
        {
            return;
        }
        m_state.last_update_seconds = current_clock_seconds;

        M4F guess = M4F::Identity();
        if (request_pending)
        {
            std::lock_guard<std::mutex> lock(m_state.service_mutex);
            guess = m_state.initial_guess;
        }
        else
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            guess = poseMatrix(
                m_state.last_offset_r * current_local_r,
                m_state.last_offset_r * current_local_t +
                    m_state.last_offset_t);
        }

        bool result;
        {
            std::lock_guard<std::mutex> lock(m_localizer_mutex);
            m_localizer->setInput(current_cloud);
            result = m_localizer->align(guess);
        }

        if (result)
        {
            const M3D map_body_r =
                guess.block<3, 3>(0, 0).cast<double>();
            const V3D map_body_t =
                guess.block<3, 1>(0, 3).cast<double>();
            {
                std::lock_guard<std::mutex> lock(m_state.message_mutex);
                m_state.last_offset_r =
                    map_body_r * current_local_r.transpose();
                m_state.last_offset_t =
                    map_body_t -
                    m_state.last_offset_r * current_local_t;
            }
            if (request_pending)
            {
                std::lock_guard<std::mutex> lock(m_state.service_mutex);
                m_state.localize_success = true;
                m_state.service_received = false;
            }
        }
        else
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 3000,
                "ICP did not converge; keeping the previous transform or retrying the initial pose");
        }

        {
            std::lock_guard<std::mutex> lock(m_state.service_mutex);
            localized = m_state.localize_success;
        }
        if (!localized)
        {
            return;
        }

        M3D map_body_r;
        V3D map_body_t;
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            map_body_r = m_state.last_offset_r * current_local_r;
            map_body_t =
                m_state.last_offset_r * current_local_t +
                m_state.last_offset_t;
        }
        sendBroadcastTF(current_time);
        publishPose(current_time, map_body_r, map_body_t);
    }

    void syncCB(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg,
        const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {
        if (cloud_msg->header.frame_id != m_config.body_frame ||
            odom_msg->header.frame_id != m_config.local_frame ||
            odom_msg->child_frame_id != m_config.body_frame)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Dropping LIO pair with unexpected frames cloud=%s odom=%s->%s",
                cloud_msg->header.frame_id.c_str(),
                odom_msg->header.frame_id.c_str(),
                odom_msg->child_frame_id.c_str());
            return;
        }

        const auto &pose = odom_msg->pose.pose;
        const Eigen::Quaterniond quaternion(
            pose.orientation.w, pose.orientation.x,
            pose.orientation.y, pose.orientation.z);
        const V3D translation(
            pose.position.x, pose.position.y, pose.position.z);
        if (!translation.allFinite() ||
            !quaternion.coeffs().allFinite() ||
            std::abs(quaternion.norm() - 1.0) > 1e-3)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Dropping non-finite or non-unit LIO pose");
            return;
        }

        CloudType::Ptr cloud = std::make_shared<CloudType>();
        pcl::fromROSMsg(*cloud_msg, *cloud);
        if (cloud->empty() ||
            !std::all_of(
                cloud->begin(), cloud->end(),
                [](const PointType &point) {
                    return pcl::isFinite(point) &&
                           std::isfinite(point.intensity);
                }))
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Dropping empty or non-finite LIO cloud");
            return;
        }

        const M3D current_local_r = quaternion.normalized().toRotationMatrix();
        M3D offset_r;
        V3D offset_t;
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            m_state.last_cloud = cloud;
            m_state.last_r = current_local_r;
            m_state.last_t = translation;
            m_state.last_message_time = cloud_msg->header.stamp;
            m_state.message_received = true;
            offset_r = m_state.last_offset_r;
            offset_t = m_state.last_offset_t;
        }

        bool localized;
        {
            std::lock_guard<std::mutex> lock(m_state.service_mutex);
            localized = m_state.localize_success;
        }
        if (localized)
        {
            // ICP updates the slowly varying map->local offset at update_hz.
            // Republish that latest offset at the incoming LIO rate so TF and
            // the global pose do not become stale between ICP iterations.
            sendBroadcastTF(cloud_msg->header.stamp);
            publishPose(
                cloud_msg->header.stamp,
                offset_r * current_local_r,
                offset_r * translation + offset_t);
        }
    }

    void initialPoseCB(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        if (!mapLoaded())
        {
            RCLCPP_WARN(
                get_logger(),
                "Ignoring /initialpose because no prior map is loaded");
            return;
        }
        if (msg->header.frame_id != m_config.map_frame)
        {
            RCLCPP_WARN(
                get_logger(),
                "Ignoring /initialpose in frame '%s'; expected '%s'",
                msg->header.frame_id.c_str(), m_config.map_frame.c_str());
            return;
        }
        if (!isFinitePose(msg->pose.pose))
        {
            RCLCPP_WARN(get_logger(), "Ignoring non-finite /initialpose");
            return;
        }

        const auto &pose = msg->pose.pose;
        Eigen::Quaterniond quaternion(
            pose.orientation.w, pose.orientation.x,
            pose.orientation.y, pose.orientation.z);
        if (quaternion.norm() < 1e-6)
        {
            RCLCPP_WARN(get_logger(), "Ignoring /initialpose with zero quaternion");
            return;
        }
        quaternion.normalize();
        queueInitialGuess(
            poseMatrix(
                quaternion.toRotationMatrix(),
                V3D(
                    pose.position.x,
                    pose.position.y,
                    pose.position.z)));
        RCLCPP_INFO(get_logger(), "Accepted /initialpose; waiting for ICP");
    }

    void sendBroadcastTF(const builtin_interfaces::msg::Time &time)
    {
        M3D offset_r;
        V3D offset_t;
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            offset_r = m_state.last_offset_r;
            offset_t = m_state.last_offset_t;
        }

        Eigen::Quaterniond quaternion(offset_r);
        quaternion.normalize();
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = m_config.map_frame;
        transform.child_frame_id = m_config.local_frame;
        transform.header.stamp = time;
        transform.transform.translation.x = offset_t.x();
        transform.transform.translation.y = offset_t.y();
        transform.transform.translation.z = offset_t.z();
        transform.transform.rotation.x = quaternion.x();
        transform.transform.rotation.y = quaternion.y();
        transform.transform.rotation.z = quaternion.z();
        transform.transform.rotation.w = quaternion.w();
        m_tf_broadcaster->sendTransform(transform);
    }

    void publishPose(
        const builtin_interfaces::msg::Time &time,
        const M3D &rotation,
        const V3D &translation)
    {
        Eigen::Quaterniond quaternion(rotation);
        quaternion.normalize();
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = m_config.map_frame;
        pose.header.stamp = time;
        pose.pose.position.x = translation.x();
        pose.pose.position.y = translation.y();
        pose.pose.position.z = translation.z();
        pose.pose.orientation.x = quaternion.x();
        pose.pose.orientation.y = quaternion.y();
        pose.pose.orientation.z = quaternion.z();
        pose.pose.orientation.w = quaternion.w();
        m_pose_pub->publish(pose);
    }

    void relocCB(
        const std::shared_ptr<
            fastlio2_interfaces::srv::Relocalize::Request> request,
        std::shared_ptr<
            fastlio2_interfaces::srv::Relocalize::Response> response)
    {
        if (!std::isfinite(request->x) || !std::isfinite(request->y) ||
            !std::isfinite(request->z) || !std::isfinite(request->yaw) ||
            !std::isfinite(request->pitch) || !std::isfinite(request->roll))
        {
            response->success = false;
            response->message = "initial pose must be finite";
            return;
        }

        if (!request->pcd_path.empty())
        {
            std::string error;
            if (!loadMap(request->pcd_path, error))
            {
                response->success = false;
                response->message = error;
                return;
            }
        }
        else if (!mapLoaded())
        {
            response->success = false;
            response->message =
                "pcd_path is empty and no prior map is preloaded";
            return;
        }

        const Eigen::AngleAxisd yaw(
            request->yaw, Eigen::Vector3d::UnitZ());
        const Eigen::AngleAxisd pitch(
            request->pitch, Eigen::Vector3d::UnitY());
        const Eigen::AngleAxisd roll(
            request->roll, Eigen::Vector3d::UnitX());
        queueInitialGuess(
            poseMatrix(
                (yaw * pitch * roll).toRotationMatrix(),
                V3D(request->x, request->y, request->z)));

        response->success = true;
        response->message =
            "relocalize request accepted; waiting for ICP";
    }

    void relocCheckCB(
        const std::shared_ptr<
            fastlio2_interfaces::srv::IsValid::Request> request,
        std::shared_ptr<
            fastlio2_interfaces::srv::IsValid::Response> response)
    {
        std::lock_guard<std::mutex> lock(m_state.service_mutex);
        response->valid =
            request->code == 0 && m_state.localize_success;
    }

    void publishMapCloud()
    {
        CloudType::Ptr map_cloud;
        {
            std::lock_guard<std::mutex> lock(m_localizer_mutex);
            map_cloud = m_localizer->refineMap();
        }
        if (!map_cloud || map_cloud->empty())
        {
            return;
        }

        sensor_msgs::msg::PointCloud2 map_cloud_msg;
        pcl::toROSMsg(*map_cloud, map_cloud_msg);
        map_cloud_msg.header.frame_id = m_config.map_frame;
        map_cloud_msg.header.stamp = now();
        m_map_cloud_pub->publish(map_cloud_msg);
    }

    NodeConfig m_config;
    NodeState m_state;
    ICPConfig m_localizer_config;
    std::shared_ptr<ICPLocalizer> m_localizer;

    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
    message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
    std::shared_ptr<Synchronizer> m_sync;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        m_initial_pose_sub;
    rclcpp::Service<
        fastlio2_interfaces::srv::Relocalize>::SharedPtr m_reloc_srv;
    rclcpp::Service<
        fastlio2_interfaces::srv::IsValid>::SharedPtr m_reloc_check_srv;
    rclcpp::Publisher<
        sensor_msgs::msg::PointCloud2>::SharedPtr m_map_cloud_pub;
    rclcpp::Publisher<
        geometry_msgs::msg::PoseStamped>::SharedPtr m_pose_pub;
    rclcpp::TimerBase::SharedPtr m_timer;

    std::mutex m_localizer_mutex;
    std::mutex m_map_mutex;
    bool m_map_loaded = false;
    std::string m_loaded_map_path;
    std::string m_startup_map_path;
    bool m_initialize_from_parameters = false;
    double m_initial_x = 0.0;
    double m_initial_y = 0.0;
    double m_initial_z = 0.0;
    double m_initial_roll = 0.0;
    double m_initial_pitch = 0.0;
    double m_initial_yaw = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizerNode>());
    rclcpp::shutdown();
    return 0;
}
