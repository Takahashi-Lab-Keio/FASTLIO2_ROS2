#include <gtest/gtest.h>

#include "map_builder/lidar_processor.h"
#include "map_builder/so3_math.h"

TEST(LidarJacobian, MatchesFiniteDifferenceWithNonzeroExtrinsicTranslation)
{
    const M3D r_wi = FastlioSo3::exp(V3D(0.2, -0.1, 0.3));
    const M3D r_il = FastlioSo3::exp(V3D(-0.05, 0.08, -0.02));
    const V3D t_il(-0.006253, 0.011775, 0.028535);
    const V3D point(2.0, -1.0, 0.4);
    const V3D normal = V3D(0.3, 0.7, -0.2).normalized();
    const auto analytical =
        LidarProcessor::rotationJacobian(r_wi, r_il, t_il, point, normal);

    Eigen::Matrix<double, 1, 3> numerical;
    constexpr double epsilon = 1e-7;
    const V3D point_imu = r_il * point + t_il;
    for (int axis = 0; axis < 3; ++axis)
    {
        V3D delta = V3D::Zero();
        delta(axis) = epsilon;
        const double plus = normal.dot(r_wi * FastlioSo3::exp(delta) * point_imu);
        const double minus = normal.dot(r_wi * FastlioSo3::exp(-delta) * point_imu);
        numerical(axis) = (plus - minus) / (2.0 * epsilon);
    }
    EXPECT_TRUE(analytical.isApprox(numerical, 1e-7))
        << "analytical=" << analytical << " numerical=" << numerical;
}
