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
//   min  (Gu − v)ᵀ W₁ (Gu − v)  +  uᵀ W₂ u
//    u
//
//   u = [F_FL, F_FR, F_RL, F_RR]ᵀ   per-wheel driving forces [N]
//   v = [Fx_total, Mz]ᵀ              virtual demand
//   G = [[1, 1, 1, 1], [-sf, sf, -sr, sr]]   sf = tf/2, sr = tr/2
//   W₁ = diag(w_fx, w_mz)
//   W₂ = diag(w_reg · Fz_avg / Fz_i)   load-inverse, so the lowest-grip tyre is
//        used least; the Fz_avg normalisation keeps w_reg's scale
//
// In OSQP form  min 0.5 uᵀPu + qᵀu,  s.t. lb ≤ Iu ≤ ub:
//   P = 2(GᵀW₁G + W₂)   (the W₂ diagonal is refreshed each cycle from wheel loads)
//   q = −2GᵀW₁v
//   |F_i| ≤ min(T_max / R, √(max(0, (μ·Fz_i)² − Fy_i²)))   motor cap ∩ friction circle
//
// The wheel radius enters only at the output: T_motor = F · R / gear_ratio.
// Reference: De Novellis et al. (SAE 2013-01-0673)
class QpTorqueAllocator : public ITorqueAllocator<QpTorqueAllocator> {
   public:
    QpTorqueAllocator(double r_w, double gear_ratio, double t_f, double t_r, double t_max, double mu,
                      double w_fx, double w_mz, double w_reg)
        : r_w_(r_w), gear_ratio_(gear_ratio), t_max_(t_max), mu_(mu) {
        const double sf = t_f / 2.0;
        const double sr = t_r / 2.0;

        Eigen::Matrix<double, 2, 4> g_mat;
        // clang-format off
        g_mat <<  1.0,  1.0,  1.0,  1.0,
                 -sf,   sf,  -sr,   sr;
        // clang-format on

        Eigen::Matrix<double, 2, 2> w1_mat = Eigen::Matrix<double, 2, 2>::Zero();
        w1_mat(0, 0) = w_fx;
        w1_mat(1, 1) = w_mz;

        gt_w1_ = g_mat.transpose() * w1_mat;
        p_track_ = 2.0 * (gt_w1_ * g_mat);
        w_reg_ = w_reg;

        // Upper triangular (OSQP convention). The equal-load W₂ = w_reg·I here only
        // establishes the sparsity pattern; the diagonal is refreshed every cycle.
        const Eigen::Matrix<double, 4, 4> p_dense =
            p_track_ + (2.0 * w_reg_ * Eigen::Matrix<double, 4, 4>::Identity());

        p_upper_.resize(4, 4);
        for (int i = 0; i < 4; ++i) {
            for (int j = i; j < 4; ++j) {
                p_upper_.insert(i, j) = p_dense(i, j);
            }
        }
        p_upper_.makeCompressed();

        // A = I₄ — the constraints are per-wheel boxes.
        Eigen::SparseMatrix<double> a_sparse(4, 4);
        a_sparse.setIdentity();

        solver_.settings()->setWarmStart(true);
        solver_.settings()->setVerbosity(false);
        // Polish adds loop latency and prints an ungated message on every cycle it is
        // skipped; warm-start plus 1e-5 tolerances suffice.
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

    [[nodiscard]] WheelTorques allocateImpl(const VehicleState& state, const YawMomentCommand& cmd,
                                            const ForceVector& forces) {
        (void)state;  // reserved for speed-dependent torque maps

        last_solve_ok_ = false;

        const Eigen::Vector2d v_demand{cmd.fx_total_, cmd.mz_};
        const Eigen::VectorXd q_vec = -2.0 * (gt_w1_ * v_demand);
        if (!solver_.updateGradient(q_vec)) {
            return last_torques_;  // a failed update leaves OSQP inconsistent
        }

        // Per-wheel force bounds: motor cap (t_max / R) ∩ friction circle.
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

        // Only P's diagonal changes with load, so refresh those values in place.
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
                // Driving force → motor torque.
                WheelTorques u = solver_.getSolution().head<4>();
                for (int i = 0; i < 4; ++i) {
                    u[i] = (u[i] * r_w_) / gear_ratio_;
                }
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

    // False when the last allocate() fell back to the previous solution.
    [[nodiscard]] bool lastSolveOk() const noexcept { return last_solve_ok_; }

   private:
    double r_w_;         // wheel radius [m]
    double gear_ratio_;  // motor → wheel gear ratio
    double t_max_;       // per-wheel motor torque cap [Nm]
    double mu_;          // tyre-road friction coefficient

    Eigen::Matrix<double, 4, 2> gt_w1_;    // GᵀW₁
    Eigen::Matrix<double, 4, 4> p_track_;  // 2·GᵀW₁G; the W₂ diagonal is added each cycle
    double w_reg_;

    // Sparsity pattern is fixed in the constructor; only the diagonal changes.
    Eigen::SparseMatrix<double> p_upper_;

    bool last_solve_ok_{true};

    // Held and re-returned when a solve fails; zero until the first success.
    WheelTorques last_torques_{WheelTorques::Zero()};

    // Floor on Fz [N] so the 1/Fz weighting stays bounded for an unloaded wheel.
    static constexpr double kFzFloor = 1.0;

    // OSQP solver — maintains warm-start state between control cycles
    OsqpEigen::Solver solver_;
};

}  // namespace tv
