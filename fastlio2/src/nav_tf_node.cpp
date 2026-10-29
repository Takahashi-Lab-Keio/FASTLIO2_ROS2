#include <chrono>
#include <limits>
#include <memory>
#include <stdexcept>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <fastlio2_interfaces/msg/localization_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "output_transforms.h"

using FastlioTransforms::RigidPose;
using namespace std::chrono_literals;

namespace
{
template<class Position, class Quaternion>
bool readPose(const Position &p, const Quaternion &q, RigidPose &pose)
{
    const Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
    pose.translation = V3D(p.x, p.y, p.z);
    if (!pose.translation.allFinite() || !rotation.coeffs().allFinite() ||
        rotation.norm() < 1e-6) return false;
    pose.rotation = rotation.normalized().toRotationMatrix();
    return std::hypot(pose.rotation(0, 0), pose.rotation(1, 0)) > 1e-6;
}

geometry_msgs::msg::TransformStamped transform(
    const std::string &parent, const std::string &child,
    const builtin_interfaces::msg::Time &stamp, const RigidPose &pose)
{
    geometry_msgs::msg::TransformStamped msg;
    msg.header.frame_id = parent;
    msg.child_frame_id = child;
    msg.header.stamp = stamp;
    msg.transform.translation.x = pose.translation.x();
    msg.transform.translation.y = pose.translation.y();
    msg.transform.translation.z = pose.translation.z();
    const Eigen::Quaterniond q(pose.rotation);
    msg.transform.rotation.x = q.x();
    msg.transform.rotation.y = q.y();
    msg.transform.rotation.z = q.z();
    msg.transform.rotation.w = q.w();
    return msg;
}
} // namespace

// Runs in a separate process from ICP; an accepted global correction is held
// while the independent IMU predictor supplies fresh local poses. This node
// never retimestamps or extrapolates incoming poses.
class NavTFNode : public rclcpp::Node
{
public:
    NavTFNode() : Node("nav_tf_node")
    {
        map_frame_ = declare_parameter<std::string>("map_frame", "map");
        odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
        body_frame_ = declare_parameter<std::string>("body_frame", "base_footprint");
        raw_body_frame_ = declare_parameter<std::string>("raw_body_frame", "fastlio_body");
        local_frame_ = declare_parameter<std::string>("local_frame", "fastlio_odom");
        planar_ = declare_parameter<bool>("planar", true);
        height_ = declare_parameter<double>("base_height", 0.0);
        max_pose_age_ = declare_parameter<double>("max_pose_age", 0.1);
        if (!std::isfinite(height_) || !std::isfinite(max_pose_age_) || max_pose_age_ <= 0.0 ||
            map_frame_.empty() || odom_frame_.empty() || body_frame_.empty() || local_frame_.empty() ||
            map_frame_ == odom_frame_ || odom_frame_ == body_frame_ || map_frame_ == body_frame_ ||
            local_frame_ == odom_frame_ || raw_body_frame_.empty() || raw_body_frame_ == body_frame_ ||
            raw_body_frame_ == map_frame_ || raw_body_frame_ == local_frame_ || raw_body_frame_ == odom_frame_ ||
            local_frame_ == map_frame_ || local_frame_ == body_frame_)
            throw std::runtime_error("invalid navigation TF frames or time limits");
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("nav_odom", 10);
        diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("nav_tf/diagnostics", 10);
        correction_sub_ = create_subscription<fastlio2_interfaces::msg::LocalizationState>(
            "localizer/localization_state", rclcpp::QoS(1).reliable().transient_local(),
            [this](fastlio2_interfaces::msg::LocalizationState::ConstSharedPtr msg) {
                const auto &tf = msg->map_to_local;
                localized_ = msg->valid && tf.header.frame_id == map_frame_ &&
                    tf.child_frame_id == local_frame_ &&
                    readPose(tf.transform.translation, tf.transform.rotation, map_to_local_);
                correction_stamp_ = rclcpp::Time(tf.header.stamp).seconds();
            });
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "predicted_odom", rclcpp::QoS(10).reliable(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { publish(*msg); });
        timer_ = create_wall_timer(100ms, [this]() { diagnostics(); });
    }

private:
    void publish(const nav_msgs::msg::Odometry &msg)
    {
        const double stamp = rclcpp::Time(msg.header.stamp).seconds();
        const double age = now().seconds() - stamp;
        RigidPose local_to_base;
        if (msg.header.frame_id != local_frame_ || msg.child_frame_id != raw_body_frame_ ||
            !readPose(msg.pose.pose.position, msg.pose.pose.orientation, local_to_base))
        { reason_ = "invalid_prediction_frame_or_pose"; return; }
        if (age < 0.0 || age > max_pose_age_ || stamp <= last_stamp_)
        { reason_ = "stale_future_or_reordered_prediction"; return; }

        const auto nav_pose = planar_ ? FastlioTransforms::planar(local_to_base, height_) : local_to_base;
        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        transforms.push_back(transform(odom_frame_, body_frame_, msg.header.stamp, nav_pose));
        transforms.push_back(transform(local_frame_, raw_body_frame_, msg.header.stamp, local_to_base));
        const bool map_ready = localized_ && correction_stamp_ <= stamp;
        if (map_ready)
        {
            const auto offset = planar_
                ? FastlioTransforms::planarMapToOdom(map_to_local_, local_to_base, height_)
                : map_to_local_;
            transforms.push_back(transform(map_frame_, odom_frame_, msg.header.stamp, offset));
            transforms.push_back(transform(map_frame_, local_frame_, msg.header.stamp, map_to_local_));
        }
        broadcaster_->sendTransform(transforms);
        // Pose-only output: the controller deliberately continues to use NSK
        // /odom twist. Do not imply that a projected 3D twist is a planar twist.
        nav_msgs::msg::Odometry output;
        output.header = transforms.front().header;
        output.child_frame_id = body_frame_;
        const auto &tf = transforms.front().transform;
        output.pose.pose.position.x = tf.translation.x;
        output.pose.pose.position.y = tf.translation.y;
        output.pose.pose.position.z = tf.translation.z;
        output.pose.pose.orientation = tf.rotation;
        output.twist.covariance[0] = output.twist.covariance[7] = output.twist.covariance[35] = 1e6;
        odom_pub_->publish(output);
        last_stamp_ = stamp;
        reason_ = map_ready ? "publishing" : "waiting_for_localization";
    }

    void diagnostics()
    {
        diagnostic_msgs::msg::DiagnosticArray array;
        array.header.stamp = now();
        const double time = now().seconds();
        const double age = time - last_stamp_;
        // A bag /clock reset must restart all localization nodes together.
        if (age < -0.5)
        {
            RCLCPP_FATAL(get_logger(), "ROS clock regressed; restart localization nodes");
            rclcpp::shutdown();
            return;
        }
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = get_fully_qualified_name() + std::string("/tf");
        status.hardware_id = "FAST-LIO";
        status.message = age > max_pose_age_ ? "prediction_timeout" : reason_;
        status.level = status.message == "publishing" ? status.OK : status.WARN;
        for (const auto &[key, number] : std::vector<std::pair<std::string, double>>{
                 {"pose_age_seconds", age}, {"map_correction_age_seconds", time - correction_stamp_}})
        {
            diagnostic_msgs::msg::KeyValue value;
            value.key = key;
            value.value = std::to_string(number);
            status.values.push_back(value);
        }
        array.status.push_back(status);
        diag_pub_->publish(array);
    }

    std::string map_frame_, odom_frame_, body_frame_, local_frame_, raw_body_frame_;
    bool planar_, localized_ = false;
    double height_, max_pose_age_;
    double last_stamp_ = -std::numeric_limits<double>::infinity();
    double correction_stamp_ = 0.0;
    std::string reason_ = "waiting_for_prediction";
    RigidPose map_to_local_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::Subscription<fastlio2_interfaces::msg::LocalizationState>::SharedPtr correction_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavTFNode>());
    rclcpp::shutdown();
    return 0;
}
