#pragma once

#include <cmath>
#include <limits>

#include "map_builder/commons.h"

namespace FastlioTransforms
{
struct RigidPose
{
    M3D rotation = M3D::Identity();
    V3D translation = V3D::Zero();
};

enum class TimestampStatus
{
    ACCEPT,
    DROP_DUPLICATE_OR_REORDERED,
    FATAL_REGRESSION,
    FATAL_FORWARD_GAP,
};

inline V3D scaleImuAcceleration(const V3D &raw_acceleration, double scale)
{
    return raw_acceleration * scale;
}

inline TimestampStatus classifyTimestamp(
    double timestamp, double previous,
    double fatal_regression_seconds = 0.5,
    double maximum_forward_gap_seconds =
        std::numeric_limits<double>::infinity())
{
    if (timestamp < previous - fatal_regression_seconds)
    {
        return TimestampStatus::FATAL_REGRESSION;
    }
    if (timestamp <= previous)
    {
        return TimestampStatus::DROP_DUPLICATE_OR_REORDERED;
    }
    if (std::isfinite(previous) &&
        timestamp - previous > maximum_forward_gap_seconds)
    {
        return TimestampStatus::FATAL_FORWARD_GAP;
    }
    return TimestampStatus::ACCEPT;
}

inline RigidPose compose(const RigidPose &parent, const RigidPose &child)
{
    return {parent.rotation * child.rotation,
            parent.rotation * child.translation + parent.translation};
}

inline RigidPose inverse(const RigidPose &pose)
{
    const M3D inverse_rotation = pose.rotation.transpose();
    return {inverse_rotation, -inverse_rotation * pose.translation};
}

// T_world_base = T_world_imu * inverse(T_base_imu).
inline RigidPose imuPoseToBasePose(
    const M3D &r_wi, const V3D &t_wi, const M3D &r_bi, const V3D &t_bi)
{
    const M3D r_wb = r_wi * r_bi.transpose();
    return {r_wb, t_wi - r_wb * t_bi};
}

// T_world_base = T_world_imu * T_imu_lidar * inverse(T_base_lidar).
// This keeps the published base pose tied to a fixed URDF LiDAR mounting even
// while the LiDAR-to-IMU extrinsic is being estimated.
inline RigidPose lidarPoseToBasePose(
    const M3D &r_wi, const V3D &t_wi,
    const M3D &r_il, const V3D &t_il,
    const M3D &r_bl, const V3D &t_bl)
{
    const RigidPose world_lidar{
        r_wi * r_il,
        r_wi * t_il + t_wi};
    return compose(world_lidar, inverse({r_bl, t_bl}));
}

// T_base_lidar = T_base_imu * T_imu_lidar.
inline RigidPose lidarToBasePose(
    const M3D &r_il, const V3D &t_il, const M3D &r_bi, const V3D &t_bi)
{
    return {r_bi * r_il, r_bi * t_il + t_bi};
}
} // namespace FastlioTransforms
