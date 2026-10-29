#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Geometry>

#include "commons.h"

namespace FastlioSo3
{
inline M3D hat(const V3D &vector)
{
    M3D matrix;
    matrix << 0.0, -vector.z(), vector.y(),
        vector.z(), 0.0, -vector.x(),
        -vector.y(), vector.x(), 0.0;
    return matrix;
}

inline M3D exp(const V3D &tangent)
{
    const double theta_squared = tangent.squaredNorm();
    const M3D tangent_hat = hat(tangent);
    double a;
    double b;
    if (theta_squared < 1e-12)
    {
        const double theta_fourth = theta_squared * theta_squared;
        a = 1.0 - theta_squared / 6.0 + theta_fourth / 120.0;
        b = 0.5 - theta_squared / 24.0 + theta_fourth / 720.0;
    }
    else
    {
        const double theta = std::sqrt(theta_squared);
        a = std::sin(theta) / theta;
        b = (1.0 - std::cos(theta)) / theta_squared;
    }
    return M3D::Identity() + a * tangent_hat + b * tangent_hat * tangent_hat;
}

inline M3D alignVectors(const V3D &from_input, const V3D &to_input)
{
    const V3D from = from_input.normalized();
    const V3D to = to_input.normalized();
    const double cosine = std::clamp(from.dot(to), -1.0, 1.0);
    if (cosine > 1.0 - 1e-12)
    {
        return M3D::Identity();
    }
    if (cosine < -1.0 + 1e-12)
    {
        return exp(from.unitOrthogonal() * std::acos(-1.0));
    }
    const V3D axis = from.cross(to).normalized();
    return exp(axis * std::acos(cosine));
}

inline V3D log(const M3D &rotation)
{
    Eigen::AngleAxisd angle_axis(rotation);
    if (!std::isfinite(angle_axis.angle()) || !angle_axis.axis().allFinite())
    {
        return V3D::Constant(std::numeric_limits<double>::quiet_NaN());
    }
    return angle_axis.axis() * angle_axis.angle();
}

inline M3D leftJacobian(const V3D &tangent)
{
    const double theta_squared = tangent.squaredNorm();
    const M3D tangent_hat = hat(tangent);
    double a;
    double b;
    if (theta_squared < 1e-12)
    {
        const double theta_fourth = theta_squared * theta_squared;
        a = 0.5 - theta_squared / 24.0 + theta_fourth / 720.0;
        b = 1.0 / 6.0 - theta_squared / 120.0 + theta_fourth / 5040.0;
    }
    else
    {
        const double theta = std::sqrt(theta_squared);
        a = (1.0 - std::cos(theta)) / theta_squared;
        b = (theta - std::sin(theta)) / (theta_squared * theta);
    }
    return M3D::Identity() + a * tangent_hat + b * tangent_hat * tangent_hat;
}

inline M3D leftJacobianInverse(const V3D &tangent)
{
    const double theta_squared = tangent.squaredNorm();
    const M3D tangent_hat = hat(tangent);
    double coefficient;
    if (theta_squared < 1e-12)
    {
        const double theta_fourth = theta_squared * theta_squared;
        coefficient = 1.0 / 12.0 + theta_squared / 720.0 +
                      theta_fourth / 30240.0;
    }
    else
    {
        const double theta = std::sqrt(theta_squared);
        coefficient =
            1.0 / theta_squared -
            (1.0 + std::cos(theta)) /
                (2.0 * theta * std::sin(theta));
    }
    return M3D::Identity() - 0.5 * tangent_hat +
           coefficient * tangent_hat * tangent_hat;
}
} // namespace FastlioSo3
