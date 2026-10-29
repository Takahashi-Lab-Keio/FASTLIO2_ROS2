#pragma once

#include <deque>
#include <limits>
#include <string>

#include "map_builder/ieskf.h"

// State-only forward propagation. The owner serializes access; this never
// modifies the estimator, its covariance, or the scan deskewing IMU queue.
// Like LIO-SAM / FAST-LIVO2, each corrected scan state replaces the prediction
// anchor and all subsequent buffered IMU measurements are integrated again.
class ImuPredictor
{
public:
    struct Limits
    {
        double max_imu_gap = 0.1;
        double max_extrapolation = 0.05;
        double max_correction_age = 0.5;
        size_t max_samples = 4000;
    };

    explicit ImuPredictor(Limits limits) : limits_(limits) {}

    void reset()
    {
        samples_.clear();
        ready_ = false;
        state_time_ = 0.0;
        correction_time_ = -std::numeric_limits<double>::infinity();
        replay_steps_ = 0;
    }

    bool add(const IMUData &sample)
    {
        if (!std::isfinite(sample.time) || !sample.acc.allFinite() ||
            !sample.gyro.allFinite() ||
            (!samples_.empty() && sample.time <= samples_.back().time))
            return false;
        if (ready_ && sample.time > state_time_)
        {
            if (sample.time - previous_.time > limits_.max_imu_gap)
                ready_ = false;
            else
            {
                integrate(state_, previous_, sample, state_time_);
                state_time_ = sample.time;
                previous_ = sample;
            }
        }
        samples_.push_back(sample);
        while (samples_.size() > limits_.max_samples)
            samples_.pop_front();
        return true;
    }

    bool correct(const State &state, double stamp)
    {
        if (!std::isfinite(stamp) || stamp <= correction_time_) return false;
        ready_ = false;
        if (samples_.empty() || samples_.front().time > stamp ||
            samples_.back().time < stamp || !finite(state))
            return false;
        while (samples_.size() > 1 && samples_[1].time <= stamp)
            samples_.pop_front();
        state_ = state;
        state_time_ = correction_time_ = stamp;
        previous_ = samples_.front();
        replay_steps_ = 0;
        for (size_t i = 1; i < samples_.size(); ++i)
        {
            const auto &next = samples_[i];
            if (next.time - previous_.time > limits_.max_imu_gap)
                return false;
            integrate(state_, previous_, next, state_time_);
            state_time_ = next.time;
            previous_ = next;
            ++replay_steps_;
        }
        ready_ = finite(state_);
        return ready_;
    }

    // A temporary extrapolation to ROS now; never feed extrapolated samples
    // back into the prediction. No stale pose with a refreshed timestamp.
    bool predict(double now, State &result, std::string &reason) const
    {
        if (!std::isfinite(now)) reason = "invalid_clock";
        else if (!ready_) reason = "waiting_for_correction_and_imu_history";
        else if (now < state_time_) reason = "imu_in_future";
        else if (now - state_time_ > limits_.max_extrapolation) reason = "imu_stale";
        else if (now - correction_time_ > limits_.max_correction_age) reason = "lidar_correction_stale";
        else
        {
            result = state_;
            // Subdivide only the bounded last-sample extrapolation, using the
            // same first-order state dynamics as the main IESKF.
            double remaining = now - state_time_;
            Input input;
            input.acc = previous_.acc;
            input.gyro = previous_.gyro;
            while (remaining > 1e-9)
            {
                const double dt = std::min(remaining, 0.005);
                propagateState(result, input, dt);
                remaining -= dt;
            }
            reason = finite(result) ? "predicting" : "non_finite_prediction";
            return finite(result);
        }
        return false;
    }

    double imuTime() const { return samples_.empty() ? 0.0 : samples_.back().time; }
    double correctionTime() const { return correction_time_; }
    V3D angularVelocity(const State &state) const { return previous_.gyro - state.bg; }
    size_t replaySteps() const { return replay_steps_; }

private:
    static bool finite(const State &s)
    {
        return s.r_wi.allFinite() && s.t_wi.allFinite() && s.v.allFinite() &&
               s.bg.allFinite() && s.ba.allFinite() && s.g.allFinite() &&
               s.r_il.allFinite() && s.t_il.allFinite();
    }

    static void integrate(State &s, const IMUData &a, const IMUData &b, double start)
    {
        // Interpolate the first sample when the LiDAR correction lies between
        // two IMU timestamps, then use midpoint measurements on that interval.
        const double alpha = (start - a.time) / (b.time - a.time);
        Input input;
        input.acc = 0.5 * ((1.0 - alpha) * a.acc + (1.0 + alpha) * b.acc);
        input.gyro = 0.5 * ((1.0 - alpha) * a.gyro + (1.0 + alpha) * b.gyro);
        propagateState(s, input, b.time - start);
    }

    Limits limits_;
    std::deque<IMUData> samples_;
    State state_;
    IMUData previous_;
    double state_time_ = 0.0;
    double correction_time_ = -std::numeric_limits<double>::infinity();
    size_t replay_steps_ = 0;
    bool ready_ = false;
};
