#pragma once

#include <Eigen/Dense>
#include <algorithm>

#include "interfaces.hpp"
#include "types.hpp"

namespace tv {

// Torque allocator solving the distribution problem in closed form via an
// LDLᵀ factorisation (docs/tv.md §optimal allocation).
//
//   min  (Gu − v)ᵀ W₁ (Gu − v)  +  uᵀ W₂ u     →     (GᵀW₁G + W₂) u = GᵀW₁ v
//    u
//
//   u = [F_FL, F_FR, F_RL, F_RR]ᵀ   per-wheel driving forces [N]
//   v = [Fx_total, Mz]ᵀ              virtual demand
//   G = [[1, 1, 1, 1], [-sf, sf, -sr, sr]]   sf = tf/2, sr = tr/2
//   W₁ = diag(w_fx, w_mz)
//   W₂ = diag(w_reg · Fz_avg / Fz_i)   load-inverse, so the lowest-grip tyre is
//        used least; the Fz_avg normalisation keeps w_reg's scale
//
// GᵀW₁G alone is rank-2 — infinitely many splits realise the same [Fx, Mz]; the
// load-inverse W₂ ≻ 0 makes the Hessian SPD and picks the unique physical split.
//
// No friction-circle limits: adding them would make this a genuinely constrained
// QP for which the closed form no longer holds. Only the per-wheel motor
// saturation t_max is applied, on the output — clipping a wheel independently
// distorts the realised [Fx, Mz].
//
// The wheel radius enters only at the output: T_motor = F · R / gear_ratio.
// Reference: De Novellis et al. (SAE 2013-01-0673)
class LdltTorqueAllocator : public ITorqueAllocator<LdltTorqueAllocator> {
   public:
    LdltTorqueAllocator(double r_w, double gear_ratio, double t_f, double t_r, double w_fx,
                       double w_mz, double w_reg, double t_max)
        : r_w_(r_w), gear_ratio_(gear_ratio), w_reg_(w_reg), t_max_(t_max) {
        const double sf = t_f / 2.0;
        const double sr = t_r / 2.0;

        // clang-format off
        g_mat_ <<  1.0,  1.0,  1.0,  1.0,
                  -sf,   sf,  -sr,   sr;
        // clang-format on

        Eigen::Matrix2d w1_mat = Eigen::Matrix2d::Zero();
        w1_mat(0, 0) = w_fx;
        w1_mat(1, 1) = w_mz;

        gt_w1_ = g_mat_.transpose() * w1_mat;
        h_track_ = gt_w1_ * g_mat_;
    }

    [[nodiscard]] WheelTorques allocateImpl(const VehicleState& state, const YawMomentCommand& cmd,
                                            const ForceVector& forces) {
        (void)state;  // reserved for speed-dependent torque maps

        last_solve_ok_ = false;

        const Eigen::Vector2d v_demand{cmd.fx_total_, cmd.mz_};
        const Eigen::Vector4d rhs = gt_w1_ * v_demand;

        double fz_sum = 0.0;
        for (int i = 0; i < 4; ++i) fz_sum += std::max(0.0, forces.fz_[i]);
        const double fz_avg = std::max(kFzFloor, fz_sum / 4.0);

        Eigen::Matrix4d h = h_track_;
        for (int i = 0; i < 4; ++i) {
            const double fz = std::max(kFzFloor, forces.fz_[i]);
            h(i, i) += w_reg_ * (fz_avg / fz);
        }

        // LDLᵀ rather than LLT: GᵀW₁G is rank-2 and those null-space directions are
        // lifted only by the small W₂, leaving H weakly conditioned there. LDLᵀ's
        // symmetric pivoting degrades gracefully, and still factors at w_reg = 0.
        Eigen::LDLT<Eigen::Matrix4d> ldlt(h);
        if (ldlt.info() != Eigen::Success) {
            return last_torques_;  // hold previous solution on numerical breakdown
        }
        Eigen::Vector4d u = ldlt.solve(rhs);

        // Driving force → motor torque, clipped to actuator authority.
        for (int i = 0; i < 4; ++i) {
            u[i] = std::clamp((u[i] * r_w_) / gear_ratio_, -t_max_, t_max_);
        }

        if (!u.allFinite()) {
            return last_torques_;
        }

        last_solve_ok_ = true;
        last_torques_ = u;
        return u;
    }

    // False when the last allocate() fell back to the previous solution.
    [[nodiscard]] bool lastSolveOk() const noexcept { return last_solve_ok_; }

   private:
    double r_w_;         // wheel radius [m]
    double gear_ratio_;  // motor → wheel gear ratio
    double w_reg_;       // load-inverse regularisation weight
    double t_max_;       // per-wheel motor-torque saturation limit [Nm]

    Eigen::Matrix<double, 2, 4> g_mat_;
    Eigen::Matrix<double, 4, 2> gt_w1_;  // GᵀW₁
    Eigen::Matrix4d h_track_;            // GᵀW₁G; the W₂ diagonal is added each cycle

    bool last_solve_ok_{true};

    // Held and re-returned when a solve fails; zero until the first success.
    WheelTorques last_torques_{WheelTorques::Zero()};

    // Floor on Fz [N] so the 1/Fz weighting stays bounded for an unloaded wheel.
    static constexpr double kFzFloor = 1.0;
};

}  // namespace tv
