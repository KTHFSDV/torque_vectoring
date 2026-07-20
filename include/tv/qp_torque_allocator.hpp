#pragma once

#include <OsqpEigen/OsqpEigen.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/Constants.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "interfaces.hpp"
#include "types.hpp"

namespace tv {

// Torque allocator solving the distribution problem via OSQP (docs/tv.md §optimal allocation).
//
// QP formulation (driving-force decision variable — radius-independent G):
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
// Rewritten in standard OSQP form  min 0.5 uᵀPu + qᵀu,  s.t. lb ≤ Iu ≤ ub:
//   P = 2(GᵀW₁G + W₂)   (GᵀW₁G constant; the W₂ diagonal is refreshed each cycle
//                        from the current wheel loads via updateHessianMatrix)
//   q = −2GᵀW₁v          (updated each cycle from current demand)
//
// Box constraints per wheel (updated each cycle), as forces:
//   Motor:          |F_i| ≤ T_max / R
//   Friction circle:|F_i| ≤ √(max(0, (μ·Fz_i)² − Fy_i²))
//
// The wheel radius enters only at the output: T_motor = F · R / gear_ratio.
//
// OSQP warm-starts from the previous solution, giving fast convergence in practice.
// Reference: De Novellis et al. (SAE 2013-01-0673)
class QpTorqueAllocator : public ITorqueAllocator<QpTorqueAllocator> {
   public:
    // Vehicle geometry / powertrain:
    //   r_w         wheel radius [m]
    //   gear_ratio  motor → wheel gear ratio (output: T_motor = F·R / gear_ratio)
    //   t_f         front track width [m]
    //   t_r         rear track width [m]
    //
    // Limits:
    //   t_max  per-wheel motor torque limit [Nm]
    //   mu     tyre-road friction coefficient
    //
    // QP weights:
    //   w_fx   weight on Fx tracking error
    //   w_mz   weight on Mz tracking error  (typically >> w_fx for racing)
    //   w_reg  regularisation weight on driving-force magnitude
    QpTorqueAllocator(double r_w, double gear_ratio, double t_f, double t_r, double t_max, double mu,
                      double w_fx, double w_mz, double w_reg)
        : r_w_(r_w), gear_ratio_(gear_ratio), t_max_(t_max), mu_(mu) {
        // --- Dependency matrix G (2×4): driving forces → [Fx, Mz] ---
        // Radius-independent — depends only on the half-track widths.
        const double sf = t_f / 2.0;
        const double sr = t_r / 2.0;

        Eigen::Matrix<double, 2, 4> g_mat;
        // clang-format off
        g_mat <<  1.0,  1.0,  1.0,  1.0,
                 -sf,   sf,  -sr,   sr;
        // clang-format on

        // --- W1 = diag(w_fx, w_mz) ---
        Eigen::Matrix<double, 2, 2> w1_mat = Eigen::Matrix<double, 2, 2>::Zero();
        w1_mat(0, 0) = w_fx;
        w1_mat(1, 1) = w_mz;

        // Precompute GᵀW₁ — reused each cycle for gradient q = −2·GᵀW₁·v
        gt_w1_ = g_mat.transpose() * w1_mat;

        // Constant tracking part of the Hessian: 2·GᵀW₁G (rank-2; the load-inverse
        // regularisation added each cycle makes the full P positive definite).
        p_track_ = 2.0 * (gt_w1_ * g_mat);
        w_reg_ = w_reg;

        // --- P = 2(BᵀW₁B + W₂), upper triangular sparse (OSQP convention) ---
        // Initial Hessian uses an equal-load W₂ = w_reg·I just to establish the
        // (full upper-triangular) sparsity pattern; the diagonal is refreshed with
        // the load-inverse weights every cycle via updateHessianMatrix(). The
        // sparse matrix is a member so the per-cycle update touches values only —
        // no allocation in the control loop.
        const Eigen::Matrix<double, 4, 4> p_dense =
            p_track_ + (2.0 * w_reg_ * Eigen::Matrix<double, 4, 4>::Identity());

        p_upper_.resize(4, 4);
        for (int i = 0; i < 4; ++i) {
            for (int j = i; j < 4; ++j) {
                p_upper_.insert(i, j) = p_dense(i, j);
            }
        }
        p_upper_.makeCompressed();

        // --- Constraint matrix A = I₄ (encodes box constraints) ---
        Eigen::SparseMatrix<double> a_sparse(4, 4);
        a_sparse.setIdentity();

        // --- Configure OSQP ---
        solver_.settings()->setWarmStart(true);
        solver_.settings()->setVerbosity(false);
        // Polishing is an optional high-accuracy refinement step; disabled here.
        // It adds latency in the control loop and prints "Polishing not needed —
        // no active set detected at optimal point" (not gated by verbosity) on
        // every cycle where it's skipped. Warm-start + 1e-5 tolerances suffice.
        solver_.settings()->setPolish(false);
        solver_.settings()->setMaxIteration(500);
        solver_.settings()->setAbsoluteTolerance(1e-5);
        solver_.settings()->setRelativeTolerance(1e-5);

        solver_.data()->setNumberOfVariables(4);
        solver_.data()->setNumberOfConstraints(4);
        solver_.data()->setHessianMatrix(p_upper_);

        Eigen::VectorXd q_init = Eigen::VectorXd::Zero(4);
        solver_.data()->setGradient(q_init);
        solver_.data()->setLinearConstraintsMatrix(a_sparse);

        const double f_motor_max = t_max / r_w;  // wheel-torque cap expressed as a force
        Eigen::VectorXd lb_init = Eigen::VectorXd::Constant(4, -f_motor_max);
        Eigen::VectorXd ub_init = Eigen::VectorXd::Constant(4, f_motor_max);
        solver_.data()->setLowerBound(lb_init);
        solver_.data()->setUpperBound(ub_init);

        if (!solver_.initSolver()) {
            throw std::runtime_error("QpTorqueAllocator: OSQP solver initialisation failed");
        }
    }

    // VehicleState:    [vx, vy, yaw_rate, steering_angle, ax, ay]
    // YawMomentCommand: {mz_, fx_total_}
    // ForceVector:     [Fx×4, Fy×4, Fz×4]
    [[nodiscard]] WheelTorques allocateImpl(const VehicleState& state, const YawMomentCommand& cmd,
                                            const ForceVector& forces) {
        (void)state;  // reserved for speed-dependent torque maps

        // Pessimistically mark the solve as failed; set true only on a clean
        // solve. Queried by the orchestrator via lastSolveOk() so the ROS node
        // can warn when the fallback was used.
        last_solve_ok_ = false;

        // Any failed solver update leaves OSQP with inconsistent data — hold the
        // previous solution for this cycle rather than solving the wrong problem
        // (zero before the first successful solve). A transient failure then
        // doesn't step the torque to zero mid-manoeuvre; persistent failures
        // keep reporting via lastSolveOk().

        // --- Gradient: q = −2·GᵀW₁·v ---
        const Eigen::Vector2d v_demand{cmd.fx_total_, cmd.mz_};
        const Eigen::VectorXd q_vec = -2.0 * (gt_w1_ * v_demand);
        if (!solver_.updateGradient(q_vec)) {
            return last_torques_;
        }

        // --- Per-wheel bounds (driving force): motor cap ∩ friction circle ---
        // The friction circle already gives a force cap; the motor wheel-torque cap
        // t_max is expressed as a force via t_max / R.
        const double f_motor_max = t_max_ / r_w_;
        Eigen::VectorXd ub(4);
        Eigen::VectorXd lb(4);
        for (int i = 0; i < 4; ++i) {
            const double fz = std::max(0.0, forces.fz_[i]);
            const double fy = forces.fy_[i];
            const double fx_cap = std::sqrt(std::max(0.0, ((mu_ * fz) * (mu_ * fz)) - (fy * fy)));
            ub[i] = std::min(f_motor_max, fx_cap);
            lb[i] = -ub[i];
        }
        if (!solver_.updateBounds(lb, ub)) {
            return last_torques_;
        }

        // --- Load-inverse regularisation: W₂ = diag(w_reg · Fz_avg / Fz_i) ---
        // Penalise each wheel inversely to its load so the lowest-grip tyre is used
        // least. Normalised by the mean load so w_reg keeps its scale and P stays
        // well-conditioned. Only the diagonal of P = 2·GᵀW₁G + 2·W₂ changes, so
        // refresh those values in the persistent sparse matrix (pattern is fixed).
        double fz_sum = 0.0;
        for (int i = 0; i < 4; ++i) fz_sum += std::max(0.0, forces.fz_[i]);
        const double fz_avg = std::max(kFzFloor, fz_sum / 4.0);

        for (int i = 0; i < 4; ++i) {
            const double fz = std::max(kFzFloor, forces.fz_[i]);
            p_upper_.coeffRef(i, i) = p_track_(i, i) + 2.0 * w_reg_ * (fz_avg / fz);
        }
        if (!solver_.updateHessianMatrix(p_upper_)) {
            return last_torques_;
        }

        switch (solver_.solveProblem()) {
            case OsqpEigen::ErrorExitFlag::NoError: {
                // Solution is the per-wheel driving force. Convert to motor torque:
                // T_motor = F · R_wheel / gear_ratio.
                WheelTorques u = solver_.getSolution().head<4>();
                for (int i = 0; i < 4; ++i) {
                    u[i] = (u[i] * r_w_) / gear_ratio_;
                }
                // Guard against NaN/Inf in the solution (e.g. non-convergence or
                // numerical breakdown reported as NoError) — hold the previous
                // solution.
                if (!u.allFinite()) {
                    return last_torques_;
                }
                last_solve_ok_ = true;
                last_torques_ = u;
                return u;
            }
            default:
                return last_torques_;
        }
    }

    // False when the most recent allocate() fell back to the previous solution
    // (failed OSQP update/solve or a non-finite solution).
    [[nodiscard]] bool lastSolveOk() const noexcept { return last_solve_ok_; }

   private:
    // Vehicle geometry / powertrain
    double r_w_;         // wheel radius [m]
    double gear_ratio_;  // motor → wheel gear ratio

    // Limits
    double t_max_;  // per-wheel motor torque cap [Nm]
    double mu_;     // tyre-road friction coefficient

    // Precomputed GᵀW₁ (4×2, constant across cycles)
    Eigen::Matrix<double, 4, 2> gt_w1_;

    // Constant tracking part of the Hessian, 2·GᵀW₁G (4×4). The load-inverse
    // regularisation diagonal is added on top each cycle.
    Eigen::Matrix<double, 4, 4> p_track_;
    double w_reg_;  // regularisation weight (load-inverse cost)

    // Persistent upper-triangular Hessian passed to OSQP. The sparsity pattern
    // is fixed in the constructor; only the diagonal values change per cycle.
    Eigen::SparseMatrix<double> p_upper_;

    // Whether the most recent allocate() produced a clean solution.
    bool last_solve_ok_{true};

    // Most recent clean solution (motor torques), returned when a solve fails.
    // Zero until the first successful solve.
    WheelTorques last_torques_{WheelTorques::Zero()};

    // Lower bound on wheel load [N] used in the 1/Fz weighting to avoid a
    // divide-by-zero / unbounded penalty when a wheel is unloaded.
    static constexpr double kFzFloor = 1.0;

    // OSQP solver — maintains warm-start state between control cycles
    OsqpEigen::Solver solver_;
};

}  // namespace tv
