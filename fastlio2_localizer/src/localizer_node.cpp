#include <algorithm>
#include <queue>
#include <mutex>
#include <filesystem>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/point_tests.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "localizers/commons.h"
#include "localizers/icp_localizer.h"
#include "fastlio2_interfaces/srv/relocalize.hpp"
#include "fastlio2_interfaces/srv/is_valid.hpp"
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

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
    double last_send_tf_seconds = -std::numeric_limits<double>::infinity();
    builtin_interfaces::msg::Time last_message_time;
    CloudType::Ptr last_cloud = std::make_shared<CloudType>();
    M3D last_r;                          // localmap_body_r
    V3D last_t;                          // localmap_body_t
    M3D last_offset_r = M3D::Identity(); // map_localmap_r
    V3D last_offset_t = V3D::Zero();     // map_localmap_t
    M4F initial_guess = M4F::Identity();
};

class LocalizerNode : public rclcpp::Node
{
public:
    LocalizerNode() : Node("localizer_node")
    {
        RCLCPP_INFO(this->get_logger(), "Localizer Node Started");
        loadParameters();
        rclcpp::QoS qos = rclcpp::QoS(10);
        m_cloud_sub.subscribe(this, m_config.cloud_topic, qos.get_rmw_qos_profile());
        m_odom_sub.subscribe(this, m_config.odom_topic, qos.get_rmw_qos_profile());

        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        m_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(10), m_cloud_sub, m_odom_sub);
        m_sync->setAgePenalty(0.1);
        m_sync->registerCallback(std::bind(&LocalizerNode::syncCB, this, std::placeholders::_1, std::placeholders::_2));
        m_localizer = std::make_shared<ICPLocalizer>(m_localizer_config);

        m_reloc_srv = this->create_service<fastlio2_interfaces::srv::Relocalize>("relocalize", std::bind(&LocalizerNode::relocCB, this, std::placeholders::_1, std::placeholders::_2));

        m_reloc_check_srv = this->create_service<fastlio2_interfaces::srv::IsValid>("relocalize_check", std::bind(&LocalizerNode::relocCheckCB, this, std::placeholders::_1, std::placeholders::_2));

        m_map_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", 10);

        m_timer = this->create_wall_timer(10ms, std::bind(&LocalizerNode::timerCB, this));
    }

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);
        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

        m_config.cloud_topic = config["cloud_topic"].as<std::string>();
        m_config.odom_topic = config["odom_topic"].as<std::string>();
        m_config.map_frame = config["map_frame"].as<std::string>();
        m_config.local_frame = config["local_frame"].as<std::string>();
        m_config.body_frame = config["body_frame"].as<std::string>(m_config.body_frame);
        m_config.update_hz = config["update_hz"].as<double>();
        if (m_config.update_hz <= 0.0)
            throw std::runtime_error("update_hz must be positive");

        m_localizer_config.rough_scan_resolution = config["rough_scan_resolution"].as<double>();
        m_localizer_config.rough_map_resolution = config["rough_map_resolution"].as<double>();
        m_localizer_config.rough_max_iteration = config["rough_max_iteration"].as<int>();
        m_localizer_config.rough_score_thresh = config["rough_score_thresh"].as<double>();

        m_localizer_config.refine_scan_resolution = config["refine_scan_resolution"].as<double>();
        m_localizer_config.refine_map_resolution = config["refine_map_resolution"].as<double>();
        m_localizer_config.refine_max_iteration = config["refine_max_iteration"].as<int>();
        m_localizer_config.refine_score_thresh = config["refine_score_thresh"].as<double>();
    }
    void timerCB()
    {
        if (!m_state.message_received)
            return;

        bool request_pending;
        bool localized;
        {
            std::lock_guard<std::mutex> lock(m_state.service_mutex);
            request_pending = m_state.service_received;
            localized = m_state.localize_success;
        }
        // Never publish an identity map transform before a real ICP success.
        if (!request_pending && !localized)
            return;

        const double current_clock_seconds = now().seconds();

        const bool update_tf =
            current_clock_seconds - m_state.last_send_tf_seconds > (1.0 / m_config.update_hz);

        if (!update_tf)
        {
            if (localized)
                sendBroadCastTF(m_state.last_message_time);
            return;
        }

        m_state.last_send_tf_seconds = current_clock_seconds;

        M4F initial_guess = M4F::Identity();
        if (request_pending)
        {
            std::lock_guard<std::mutex> lock(m_state.service_mutex);
            initial_guess = m_state.initial_guess;
        }
        else
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            initial_guess.block<3, 3>(0, 0) = (m_state.last_offset_r * m_state.last_r).cast<float>();
            initial_guess.block<3, 1>(0, 3) = (m_state.last_offset_r * m_state.last_t + m_state.last_offset_t).cast<float>();
        }

        M3D current_local_r;
        V3D current_local_t;
        builtin_interfaces::msg::Time current_time;
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            current_local_r = m_state.last_r;
            current_local_t = m_state.last_t;
            current_time = m_state.last_message_time;
            m_localizer->setInput(m_state.last_cloud);
        }

        bool result = m_localizer->align(initial_guess);
        if (result)
        {
            M3D map_body_r = initial_guess.block<3, 3>(0, 0).cast<double>();
            V3D map_body_t = initial_guess.block<3, 1>(0, 3).cast<double>();
            {
                std::lock_guard<std::mutex> lock(m_state.message_mutex);
                m_state.last_offset_r = map_body_r * current_local_r.transpose();
                m_state.last_offset_t = -map_body_r * current_local_r.transpose() * current_local_t + map_body_t;
            }
            if (request_pending)
            {
                std::lock_guard<std::mutex> lock(m_state.service_mutex);
                m_state.localize_success = true;
                m_state.service_received = false;
            }
        }
        {
            std::lock_guard<std::mutex> lock(m_state.service_mutex);
            localized = m_state.localize_success;
        }
        if (localized)
        {
            sendBroadCastTF(current_time);
            publishMapCloud(current_time);
        }
    }
    void syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {
        if (cloud_msg->header.frame_id != m_config.body_frame ||
            odom_msg->header.frame_id != m_config.local_frame ||
            odom_msg->child_frame_id != m_config.body_frame)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Dropping LIO pair with unexpected frames cloud=%s odom=%s->%s",
                cloud_msg->header.frame_id.c_str(), odom_msg->header.frame_id.c_str(),
                odom_msg->child_frame_id.c_str());
            return;
        }
        const auto &pose = odom_msg->pose.pose;
        const Eigen::Quaterniond quaternion(
            pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        const V3D translation(pose.position.x, pose.position.y, pose.position.z);
        if (!translation.allFinite() || !quaternion.coeffs().allFinite() ||
            std::abs(quaternion.norm() - 1.0) > 1e-3)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Dropping non-finite or non-unit LIO pose");
            return;
        }

        CloudType::Ptr cloud = std::make_shared<CloudType>();
        pcl::fromROSMsg(*cloud_msg, *cloud);
        if (cloud->empty() || !std::all_of(cloud->begin(), cloud->end(), [](const PointType &point) {
                return pcl::isFinite(point) && std::isfinite(point.intensity);
            }))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Dropping empty or non-finite LIO cloud");
            return;
        }

        std::lock_guard<std::mutex> lock(m_state.message_mutex);
        m_state.last_cloud = cloud;

        m_state.last_r = quaternion.normalized().toRotationMatrix();
        m_state.last_t = translation;
        m_state.last_message_time = cloud_msg->header.stamp;
        if (!m_state.message_received)
        {
            m_state.message_received = true;
        }
    }

    void sendBroadCastTF(builtin_interfaces::msg::Time &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = m_config.map_frame;
        transformStamped.child_frame_id = m_config.local_frame;
        transformStamped.header.stamp = time;
        M3D offset_r;
        V3D offset_t;
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            offset_r = m_state.last_offset_r;
            offset_t = m_state.last_offset_t;
        }
        Eigen::Quaterniond q(offset_r);
        V3D t = offset_t;
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        m_tf_broadcaster->sendTransform(transformStamped);
    }

    void relocCB(const std::shared_ptr<fastlio2_interfaces::srv::Relocalize::Request> request, std::shared_ptr<fastlio2_interfaces::srv::Relocalize::Response> response)
    {
        std::string pcd_path = request->pcd_path;
        float x = request->x;
        float y = request->y;
        float z = request->z;
        float yaw = request->yaw;
        float roll = request->roll;
        float pitch = request->pitch;

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
            !std::isfinite(yaw) || !std::isfinite(roll) || !std::isfinite(pitch))
        {
            response->success = false;
            response->message = "initial pose must be finite";
            return;
        }

        if (!std::filesystem::exists(pcd_path))
        {
            response->success = false;
            response->message = "pcd file not found";
            return;
        }

        Eigen::AngleAxisd yaw_angle = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd roll_angle = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_angle = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
        bool load_flag = m_localizer->loadMap(pcd_path);
        if (!load_flag)
        {
            response->success = false;
            response->message = "load map failed";
            return;
        }
        {
            std::lock_guard<std::mutex> lock(m_state.service_mutex);
            m_state.initial_guess.setIdentity();
            // Preserve the public service convention: Rz(yaw) * Rx(roll) * Ry(pitch).
            m_state.initial_guess.block<3, 3>(0, 0) =
                (yaw_angle * roll_angle * pitch_angle).toRotationMatrix().cast<float>();
            m_state.initial_guess.block<3, 1>(0, 3) = V3F(x, y, z);
            m_state.service_received = true;
            m_state.localize_success = false;
        }

        response->success = true;
        response->message = "relocalize request accepted; waiting for ICP";
        return;
    }

    void relocCheckCB(const std::shared_ptr<fastlio2_interfaces::srv::IsValid::Request> request, std::shared_ptr<fastlio2_interfaces::srv::IsValid::Response> response)
    {
        std::lock_guard<std::mutex> lock(m_state.service_mutex);
        response->valid = request->code == 0 && m_state.localize_success;
        return;
    }
    void publishMapCloud(builtin_interfaces::msg::Time &time)
    {
        if (m_map_cloud_pub->get_subscription_count() < 1)
            return;
        CloudType::Ptr map_cloud = m_localizer->refineMap();
        if (map_cloud->size() < 1)
            return;
        sensor_msgs::msg::PointCloud2 map_cloud_msg;
        pcl::toROSMsg(*map_cloud, map_cloud_msg);
        map_cloud_msg.header.frame_id = m_config.map_frame;
        map_cloud_msg.header.stamp = time;
        m_map_cloud_pub->publish(map_cloud_msg);
    }

private:
    NodeConfig m_config;
    NodeState m_state;

    ICPConfig m_localizer_config;
    std::shared_ptr<ICPLocalizer> m_localizer;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
    message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
    rclcpp::TimerBase::SharedPtr m_timer;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>> m_sync;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    rclcpp::Service<fastlio2_interfaces::srv::Relocalize>::SharedPtr m_reloc_srv;
    rclcpp::Service<fastlio2_interfaces::srv::IsValid>::SharedPtr m_reloc_check_srv;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_map_cloud_pub;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizerNode>());
    rclcpp::shutdown();
    return 0;
}
