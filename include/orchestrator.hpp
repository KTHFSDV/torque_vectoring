#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <utility>

#include "interfaces.hpp"
#include "types.hpp"

namespace tv {

// Online mean / sample standard deviation (Welford's algorithm).
class RunningStat {
   public:
    void add(double x) noexcept {
        ++n_;
        const double delta = x - mean_;
        mean_ += delta / static_cast<double>(n_);
        m2_ += delta * (x - mean_);
    }
    [[nodiscard]] double mean() const noexcept { return mean_; }
    [[nodiscard]] double stddev() const noexcept {
        return n_ > 1 ? std::sqrt(m2_ / static_cast<double>(n_ - 1)) : 0.0;
    }

   private:
    std::uint64_t n_{0};
    double mean_{0.0};
    double m2_{0.0};  // sum of squared deviations from the running mean
};

/// Runs the full torque vectoring pipeline:
///   VehicleState -> ForceEstimator -> YawMomentGenerator -> TorqueAllocator -> WheelTorques
template <ForceEstimator F, YawMomentGenerator Y, TorqueAllocator A>
class Orchestrator {
   public:
    Orchestrator(F force_estimator, Y yaw_generator, A torque_allocator)
        : force_estimator_(std::move(force_estimator)),
          yaw_generator_(std::move(yaw_generator)),
          torque_allocator_(std::move(torque_allocator)) {}

    /// Runs the pipeline and returns the wheel torques together with every
    /// intermediate signal and per-stage timing, for debug publication. `dt` is the
    /// elapsed time since the previous call [s]. `alloc_ok` is false when the
    /// allocator held its previous solution after a failed solve.
    [[nodiscard]] PipelineResult run(const VehicleState& state, const SteeringCommand& cmd,
                                     double dt) {
        // Optional so the concepts don't have to mandate setDt().
        if constexpr (requires(Y& y) { y.setDt(dt); }) {
            yaw_generator_.setDt(dt);
        }

        using Clock = std::chrono::steady_clock;
        const auto start = Clock::now();
        const ForceVector forces = force_estimator_.estimateForces(state);
        const auto t_forces = Clock::now();
        const YawMomentCommand yaw_cmd = yaw_generator_.compute(state, forces, cmd);
        const auto t_yaw = Clock::now();
        const auto allocated_torques = torque_allocator_.allocate(state, yaw_cmd, forces);
        const auto t_alloc = Clock::now();

        const auto us = [](Clock::time_point a, Clock::time_point b) {
            return std::chrono::duration<double, std::micro>(b - a).count();
        };
        const StageTimings timings{.forces_us = us(start, t_forces),
                                   .yaw_us = us(t_forces, t_yaw),
                                   .alloc_us = us(t_yaw, t_alloc),
                                   .total_us = us(start, t_alloc)};
        timing_stats_[0].add(timings.forces_us);
        timing_stats_[1].add(timings.yaw_us);
        timing_stats_[2].add(timings.alloc_us);
        timing_stats_[3].add(timings.total_us);

        bool alloc_ok = true;
        if constexpr (requires(const A& a) {
                          { a.lastSolveOk() } -> std::convertible_to<bool>;
                      }) {
            alloc_ok = torque_allocator_.lastSolveOk();
        }

        return PipelineResult{.forces = forces,
                              .yaw_cmd = yaw_cmd,
                              .torques = allocated_torques,
                              .timings = timings,
                              .timings_avg = StageTimings{.forces_us = timing_stats_[0].mean(),
                                                          .yaw_us = timing_stats_[1].mean(),
                                                          .alloc_us = timing_stats_[2].mean(),
                                                          .total_us = timing_stats_[3].mean()},
                              .timings_std = StageTimings{.forces_us = timing_stats_[0].stddev(),
                                                          .yaw_us = timing_stats_[1].stddev(),
                                                          .alloc_us = timing_stats_[2].stddev(),
                                                          .total_us = timing_stats_[3].stddev()},
                              .alloc_ok = alloc_ok};
    }

    /// Forwards per-wheel feedback to the force-estimation stage, when that stage
    /// consumes it. No-op otherwise, so the concepts needn't mandate it.
    void updateWheelState(const WheelTorques& motor_torques, const WheelOmegas& wheel_omegas,
                          double dt) {
        if constexpr (requires(F& f) { f.updateWheelState(motor_torques, wheel_omegas, dt); }) {
            force_estimator_.updateWheelState(motor_torques, wheel_omegas, dt);
        }
    }

    [[nodiscard]] const F& forceEstimator() const noexcept { return force_estimator_; }
    [[nodiscard]] const Y& yawGenerator() const noexcept { return yaw_generator_; }
    [[nodiscard]] const A& torqueAllocator() const noexcept { return torque_allocator_; }

   private:
    F force_estimator_;
    Y yaw_generator_;
    A torque_allocator_;

    // [forces, yaw, alloc, total]
    std::array<RunningStat, 4> timing_stats_{};
};

template <ForceEstimator F, YawMomentGenerator Y, TorqueAllocator A>
Orchestrator(F, Y, A) -> Orchestrator<F, Y, A>;

}  // namespace tv
