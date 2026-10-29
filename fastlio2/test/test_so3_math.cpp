#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "map_builder/so3_math.h"

TEST(So3Math, ExponentialMatchesEigenAngleAxis)
{
    const V3D tangent(0.2, -0.1, 0.3);
    const double angle = tangent.norm();
    const M3D expected =
        Eigen::AngleAxisd(angle, tangent / angle).toRotationMatrix();
    EXPECT_TRUE(FastlioSo3::exp(tangent).isApprox(expected, 1e-12));
}

TEST(So3Math, LogAndExpRoundTrip)
{
    const V3D tangent(-0.3, 0.15, 0.2);
    EXPECT_TRUE(
        FastlioSo3::log(FastlioSo3::exp(tangent)).isApprox(tangent, 1e-12));
}

TEST(So3Math, AlignsVectorsIncludingOppositeDirections)
{
    for (const V3D &target : {V3D(0.2, -0.3, 0.9).normalized(),
                              V3D(0.0, 0.0, -1.0)})
    {
        const V3D source(0.0, 0.0, 1.0);
        const M3D rotation = FastlioSo3::alignVectors(source, target);
        EXPECT_TRUE((rotation * source).isApprox(target, 1e-12));
        EXPECT_TRUE(
            (rotation.transpose() * rotation).isApprox(M3D::Identity(), 1e-12));
        EXPECT_NEAR(rotation.determinant(), 1.0, 1e-12);
    }
}

TEST(So3Math, LeftJacobianInverseIsInverse)
{
    for (const V3D &tangent : {V3D(1e-8, -2e-8, 3e-8),
                               V3D(0.2, -0.3, 0.1)})
    {
        EXPECT_TRUE(
            (FastlioSo3::leftJacobianInverse(tangent) *
             FastlioSo3::leftJacobian(tangent))
                .isApprox(M3D::Identity(), 1e-11));
    }
}
