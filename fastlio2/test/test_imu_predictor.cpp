#include <chrono>
#include <gtest/gtest.h>
#include "imu_predictor.h"

namespace
{
IMUData sample(double t)
{
    return {V3D(0.2 + 0.1 * t, -0.4, 9.81), V3D(0.02, -0.01, 0.3 + 0.02 * t), t};
}

void expectState(const State &a, const State &b, double tolerance = 1e-11)
{
    EXPECT_LT((a.r_wi - b.r_wi).norm(), tolerance);
    EXPECT_LT((a.t_wi - b.t_wi).norm(), tolerance);
    EXPECT_LT((a.v - b.v).norm(), tolerance);
    EXPECT_LT((a.ba - b.ba).norm(), tolerance);
    EXPECT_LT((a.bg - b.bg).norm(), tolerance);
}

ImuPredictor::Limits testLimits()
{
    ImuPredictor::Limits limits;
    limits.max_correction_age = 2.0;
    return limits;
}
} // namespace

TEST(ImuPredictor, MainFilterDynamicsRemainNumericallyUnchanged)
{
    IESKF filter;
    filter.P().setIdentity();
    filter.x().bg = V3D(0.01, 0.02, -0.05);
    filter.x().ba = V3D(0.05, 0.03, -0.02);
    State legacy = filter.x();
    for (int i = 0; i < 400; ++i)
    {
        Input input;
        const auto imu = sample(i * 0.005);
        input.acc = imu.acc;
        input.gyro = imu.gyro;
        // Original IESKF implementation, independently evaluated.
        const V3D acceleration = legacy.r_wi * (input.acc - legacy.ba) + legacy.g;
        legacy.t_wi += legacy.v * 0.005;
        legacy.v += acceleration * 0.005;
        legacy.r_wi *= FastlioSo3::exp((input.gyro - legacy.bg) * 0.005);
        filter.predict(input, 0.005, M12D::Identity());
    }
    expectState(filter.x(), legacy);
}

TEST(ImuPredictor, DelayedCorrectionReplaysWithNewBiasesAndPartialInterval)
{
    ImuPredictor predictor(testLimits());
    for (int i = 0; i <= 100; ++i) ASSERT_TRUE(predictor.add(sample(10.0 + i * 0.005)));
    State corrected;
    corrected.t_wi = V3D(2, 3, 4);
    corrected.v = V3D(0.2, -0.5, 0.03);
    corrected.r_wi = Eigen::AngleAxisd(0.2, V3D::UnitX()).toRotationMatrix();
    corrected.bg = V3D(0.005, -0.03, 0.08);
    corrected.ba = V3D(0.1, -0.02, 0.06);
    const double anchor = 10.123;
    ASSERT_TRUE(predictor.correct(corrected, anchor));

    IESKF reference;
    reference.P().setIdentity();
    reference.x() = corrected;
    double start = anchor;
    for (int i = 25; i <= 100; ++i)
    {
        const auto a = sample(10.0 + (i - 1) * 0.005);
        const auto b = sample(10.0 + i * 0.005);
        const double f = (start - a.time) / (b.time - a.time);
        const V3D first_acc = a.acc + f * (b.acc - a.acc);
        const V3D first_gyro = a.gyro + f * (b.gyro - a.gyro);
        Input input;
        input.acc = 0.5 * (first_acc + b.acc);
        input.gyro = 0.5 * (first_gyro + b.gyro);
        reference.predict(input, b.time - start, M12D::Identity());
        start = b.time;
    }
    State result;
    std::string reason;
    ASSERT_TRUE(predictor.predict(10.5, result, reason));
    expectState(result, reference.x());
    EXPECT_EQ(predictor.replaySteps(), 76U);
    EXPECT_TRUE(corrected.t_wi.isApprox(V3D(2, 3, 4))); // snapshot untouched
}

TEST(ImuPredictor, IncrementalAndBufferedPredictionAgree)
{
    ImuPredictor incremental(testLimits()), buffered(testLimits());
    ASSERT_TRUE(incremental.add(sample(1.0)));
    ASSERT_TRUE(incremental.correct(State{}, 1.0));
    for (int i = 0; i <= 100; ++i)
    {
        const auto imu = sample(1.0 + i * 0.005);
        buffered.add(imu);
        if (i) incremental.add(imu);
    }
    ASSERT_TRUE(buffered.correct(State{}, 1.0));
    State a, b;
    std::string reason;
    ASSERT_TRUE(buffered.predict(1.53, a, reason));
    ASSERT_TRUE(incremental.predict(1.53, b, reason));
    expectState(a, b);
    // Calling a later prediction does not accumulate extrapolation in state.
    ASSERT_TRUE(incremental.predict(1.5, b, reason));
    ASSERT_TRUE(buffered.predict(1.5, a, reason));
    expectState(a, b);
}

TEST(ImuPredictor, StationaryTiltedImuDoesNotDriftAndYawTurnPropagates)
{
    ImuPredictor predictor(testLimits());
    State state;
    state.r_wi = Eigen::AngleAxisd(0.3, V3D::UnitX()).toRotationMatrix();
    const V3D acc = -state.r_wi.transpose() * state.g;
    predictor.add({acc, V3D::Zero(), 1.0});
    ASSERT_TRUE(predictor.correct(state, 1.0));
    for (int i = 1; i <= 100; ++i) predictor.add({acc, V3D::Zero(), 1.0 + i * 0.005});
    State result;
    std::string reason;
    ASSERT_TRUE(predictor.predict(1.54, result, reason));
    expectState(result, state);

    ImuPredictor turning(testLimits());
    turning.add({V3D(0, 0, 9.81), V3D(0, 0, 0.5), 1.0});
    turning.correct(State{}, 1.0);
    for (int i = 1; i <= 100; ++i) turning.add({V3D(0, 0, 9.81), V3D(0, 0, 0.5), 1.0 + i * 0.005});
    ASSERT_TRUE(turning.predict(1.52, result, reason));
    EXPECT_NEAR(std::atan2(result.r_wi(1, 0), result.r_wi(0, 0)), 0.26, 1e-10);
}

TEST(ImuPredictor, RejectsGapsStaleCorrectionsFutureSamplesAndMissingHistory)
{
    ImuPredictor predictor(ImuPredictor::Limits{});
    State result;
    std::string reason;
    EXPECT_FALSE(predictor.predict(1.0, result, reason));
    predictor.add(sample(1.0));
    EXPECT_FALSE(predictor.correct(State{}, 0.9));
    ASSERT_TRUE(predictor.correct(State{}, 1.0));
    EXPECT_FALSE(predictor.add(sample(1.0)));
    EXPECT_FALSE(predictor.add(sample(0.9)));
    EXPECT_FALSE(predictor.correct(State{}, 0.9));
    EXPECT_TRUE(predictor.predict(1.02, result, reason));
    EXPECT_FALSE(predictor.predict(0.9, result, reason));
    EXPECT_EQ(reason, "imu_in_future");
    EXPECT_FALSE(predictor.predict(1.06, result, reason));
    EXPECT_EQ(reason, "imu_stale");
    for (int i = 1; i <= 110; ++i) predictor.add(sample(1.0 + i * 0.005));
    EXPECT_FALSE(predictor.predict(1.55, result, reason));
    EXPECT_EQ(reason, "lidar_correction_stale");
    ASSERT_TRUE(predictor.correct(State{}, 1.5));
    ASSERT_TRUE(predictor.predict(1.55, result, reason));
    predictor.add(sample(1.8));
    EXPECT_FALSE(predictor.predict(1.8, result, reason));
    EXPECT_FALSE(predictor.correct(State{}, 1.6)); // gap crosses correction
    EXPECT_FALSE(predictor.predict(std::numeric_limits<double>::quiet_NaN(), result, reason));

    auto limits = testLimits();
    limits.max_samples = 3;
    ImuPredictor bounded(limits);
    for (int i = 0; i < 10; ++i) bounded.add(sample(1.0 + i * 0.005));
    EXPECT_FALSE(bounded.correct(State{}, 1.0));
    EXPECT_TRUE(bounded.correct(State{}, 1.04));
}

TEST(ImuPredictor, TypicalCorrectionReplayFitsPublicationPeriod)
{
    std::vector<double> times;
    for (int repeat = 0; repeat < 1000; ++repeat)
    {
        ImuPredictor predictor(testLimits());
        for (int i = 0; i <= 100; ++i) predictor.add(sample(1.0 + i * 0.005));
        const auto start = std::chrono::steady_clock::now();
        ASSERT_TRUE(predictor.correct(State{}, 1.0));
        State result;
        std::string reason;
        ASSERT_TRUE(predictor.predict(1.53, result, reason));
        times.push_back(std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count());
    }
    std::sort(times.begin(), times.end());
    std::cout << "100-sample replay + extrapolation p99_ms=" << times[990] << std::endl;
    EXPECT_LT(times[990], 10.0);
}
