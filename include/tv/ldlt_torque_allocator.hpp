#pragma once

#include <Eigen/Dense>
#include <algorithm>

#include "interfaces.hpp"
#include "types.hpp"

namespace tv {

// Torque allocator solving the distribution problem in closed form via an
// LDLᵀ (Cholesky-with-pivoting) factorisation (docs/tv.md §optimal allocation).
//
// Unconstrained weighted least-squares formulation (driving-force decision
// variable — radius-independent G):
//   min  (Gu − v)ᵀ W₁ (Gu − v)  +  uᵀ W₂ u
//    u
//
//   u = [F_FL, F_FR, F_RL, F_RR]ᵀ   per-wheel driving forces [N]
//   v = [Fx_total, Mz]ᵀ              virtual demand
//
//   Dependency matrix G (2×4) — depends only on the half-track widths:
//     Fx row: [  1,    1,    1,    1  ]
//     Mz row: [-sf,   sf,  -sr,   sr ]     sf = tf/2, sr = tr/2
//
//   W₁ = diag(w_fx, w_mz)              demand tracking weights
//   W₂ = diag(w_reg · Fz_avg / Fz_i)   load-inverse regularisation — each wheel is
//                                       penalised inversely to its load, so the
//                                       lowest-grip tyre is used least (normalised by
//                                       the mean load Fz_avg so w_reg keeps its scale)
//
// Because there are no inequality constraints, the optimum is the solution of the
// normal equations (setting ∇J = 0; the factor of 2 cancels):
//   (GᵀW₁G + W₂) u = GᵀW₁ v
//
// G is 2×4, so GᵀW₁G is rank-2 (singular) on its own: infinitely many force splits
// realise the same [Fx, Mz]. The load-inverse W₂ ≻ 0 makes the Hessian symmetric
// positive definite, picking the unique physically-meaningful split and letting us
// factor with LDLᵀ. On a 4×4 system this is essentially free — factor once per
// cycle, then forward/back-substitute.
//
// The wheel radius enters only at the output: T_motor = F · R / gear_ratio.
//
// NOTE: this layer enforces NO actuator or friction-circle limits. Motor-torque and
// tyre-grip bounds are assumed to be handled upstream (or by downstream clipping);
// adding them here would make the problem a genuinely constrained QP for which the
// closed form no longer holds.
//
// Reference: De Novellis et al. (SAE 2013-01-0673)
class LdltTorqueAllocator : public ITorqueAllocator<LdltTorqueAllocator> {
   public:
    // Vehicle geometry / powertrain:
    //   r_w         wheel radius [m]
    //   gear_ratio  motor → wheel gear ratio (output: T_motor = F·R / gear_ratio)
    //   t_f         front track width [m]
    //   t_r         rear track width [m]
    //
    // QP weights:
    //   w_fx   weight on Fx tracking error
    //   w_mz   weight on Mz tracking error  (typically >> w_fx for racing)
    //   w_reg  load-inverse regularisation weight on driving-force magnitude
    LdltTorqueAllocator(double r_w, double gear_ratio, double t_f, double t_r, double w_fx,
                       double w_mz, double w_reg)
        : r_w_(r_w), gear_ratio_(gear_ratio), w_reg_(w_reg) {
        // --- Dependency matrix G (2×4): driving forces → [Fx, Mz] ---
        // Radius-independent — depends only on the half-track widths.
        const double sf = t_f / 2.0;
        const double sr = t_r / 2.0;

        // clang-format off
        g_mat_ <<  1.0,  1.0,  1.0,  1.0,
                  -sf,   sf,  -sr,   sr;
        // clang-format on

        // --- W1 = diag(w_fx, w_mz) ---
        Eigen::Matrix2d w1_mat = Eigen::Matrix2d::Zero();
        w1_mat(0, 0) = w_fx;
        w1_mat(1, 1) = w_mz;

        // Precompute GᵀW₁ (4×2) — reused each cycle for the rhs GᵀW₁·v and the
        // constant tracking part of the Hessian GᵀW₁G.
        gt_w1_ = g_mat_.transpose() * w1_mat;

        // Constant tracking part of the Hessian, GᵀW₁G (rank-2; the per-cycle
        // load-inverse W₂ diagonal added on top makes the full H positive definite).
        h_track_ = gt_w1_ * g_mat_;
    }

    // VehicleState:    [vx, vy, yaw_rate, steering_angle, ax, ay]
    // YawMomentCommand: {mz_, fx_total_}
    // ForceVector:     [Fx×4, Fy×4, Fz×4]
    [[nodiscard]] WheelTorques allocateImpl(const VehicleState& state, const YawMomentCommand& cmd,
                                            const ForceVector& forces) {
        (void)state;  // reserved for speed-dependent torque maps

        // Pessimistically mark the solve as failed; set true only on a clean solve.
        // Queried by the orchestrator via lastSolveOk() so the ROS node can warn
        // when the fallback (previous solution) was used.
        last_solve_ok_ = false;

        // --- rhs: GᵀW₁·v ---
        const Eigen::Vector2d v_demand{cmd.fx_total_, cmd.mz_};
        const Eigen::Vector4d rhs = gt_w1_ * v_demand;

        // --- Load-inverse regularisation: W₂ = diag(w_reg · Fz_avg / Fz_i) ---
        // Penalise each wheel inversely to its load so the lowest-grip tyre is used
        // least. Normalised by the mean load so w_reg keeps its scale and H stays
        // well-conditioned.
        double fz_sum = 0.0;
        for (int i = 0; i < 4; ++i) fz_sum += std::max(0.0, forces.fz_[i]);
        const double fz_avg = std::max(kFzFloor, fz_sum / 4.0);

        Eigen::Matrix4d h = h_track_;
        for (int i = 0; i < 4; ++i) {
            const double fz = std::max(kFzFloor, forces.fz_[i]);
            h(i, i) += w_reg_ * (fz_avg / fz);
        }

        // --- LDLᵀ factorisation + solve of (GᵀW₁G + W₂) u = GᵀW₁ v ---
        // H is SPD while w_reg > 0, so LLT would also be valid; LDLᵀ's symmetric
        // pivoting is preferred because the tracking part GᵀW₁G is rank-2 and the
        // two null-space directions are lifted only by the small load-inverse W₂,
        // leaving H weakly conditioned there. LDLᵀ degrades gracefully (and still
        // factors) if w_reg is ever set to 0, where the matrix becomes singular.
        Eigen::LDLT<Eigen::Matrix4d> ldlt(h);
        if (ldlt.info() != Eigen::Success) {
            return last_torques_;  // hold previous solution on numerical breakdown
        }
        Eigen::Vector4d u = ldlt.solve(rhs);

        // Convert per-wheel driving force to motor torque: T_motor = F·R / gear_ratio.
        for (int i = 0; i < 4; ++i) {
            u[i] = (u[i] * r_w_) / gear_ratio_;
        }

        // Guard against NaN/Inf — hold the previous solution.
        if (!u.allFinite()) {
            return last_torques_;
        }

        last_solve_ok_ = true;
        last_torques_ = u;
        return u;
    }

    // False when the most recent allocate() fell back to the previous solution
    // (numerical breakdown or a non-finite solution).
    [[nodiscard]] bool lastSolveOk() const noexcept { return last_solve_ok_; }

   private:
    // Vehicle geometry / powertrain
    double r_w_;         // wheel radius [m]
    double gear_ratio_;  // motor → wheel gear ratio
    double w_reg_;       // load-inverse regularisation weight

    // Dependency matrix G (2×4), constant across cycles.
    Eigen::Matrix<double, 2, 4> g_mat_;

    // Precomputed GᵀW₁ (4×2, constant across cycles).
    Eigen::Matrix<double, 4, 2> gt_w1_;

    // Constant tracking part of the Hessian, GᵀW₁G (4×4). The load-inverse
    // regularisation diagonal is added on top each cycle.
    Eigen::Matrix4d h_track_;

    // Whether the most recent allocate() produced a clean solution.
    bool last_solve_ok_{true};

    // Most recent clean solution (motor torques), returned when a solve fails.
    // Zero until the first successful solve.
    WheelTorques last_torques_{WheelTorques::Zero()};

    // Lower bound on wheel load [N] used in the 1/Fz weighting to avoid a
    // divide-by-zero / unbounded penalty when a wheel is unloaded.
    static constexpr double kFzFloor = 1.0;
};

}  // namespace tv
