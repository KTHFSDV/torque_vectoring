#pragma once

#include <OsqpEigen/OsqpEigen.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/Constants.hpp>
#include <algorithm>
#include <cmath>

#include "interfaces.hpp"
#include "types.hpp"

namespace tv {

// Torque allocator solving the distribution problem via OSQP (docs/tv.md §optimal allocation).
//
// QP formulation:
//   min  (Bu − v)ᵀ W₁ (Bu − v)  +  uᵀ W₂ u
//    u
//
//   u = [T_FL, T_FR, T_RL, T_RR]ᵀ   wheel torques [Nm]
//   v = [Fx_total, Mz]ᵀ              virtual demand
//
//   Effectiveness matrix B (2×4):
//     Fx row: [  1/R,    1/R,    1/R,    1/R  ]
//     Mz row: [-tf/2R,  tf/2R, -tr/2R,  tr/2R ]
//
//   W₁ = diag(w_fx, w_mz)    demand tracking weights
//   W₂ = w_reg · I₄           regularisation (equalises tyre utilisation)
//
// Rewritten in standard OSQP form  min 0.5 uᵀPu + qᵀu,  s.t. lb ≤ Iu ≤ ub:
//   P = 2(BᵀW₁B + W₂)   (upper triangular, constant — computed once in constructor)
//   q = −2BᵀW₁v          (updated each cycle from current demand)
//
// Box constraints per wheel (updated each cycle):
//   Motor:          |T_i| ≤ T_max
//   Friction circle:|T_i| ≤ R·√(max(0, (μ·Fz_i)² − Fy_i²))
//
// OSQP warm-starts from the previous solution, giving fast convergence in practice.
// Reference: De Novellis et al. (SAE 2013-01-0673)
class QpTorqueAllocator : public ITorqueAllocator<QpTorqueAllocator> {
   public:
    // Vehicle geometry:
    //   r_w    wheel radius [m]
    //   t_f    front track width [m]
    //   t_r    rear track width [m]
    //
    // Limits:
    //   t_max  per-wheel motor torque limit [Nm]
    //   mu     tyre-road friction coefficient
    //
    // QP weights:
    //   w_fx   weight on Fx tracking error
    //   w_mz   weight on Mz tracking error  (typically >> w_fx for racing)
    //   w_reg  regularisation weight on torque magnitude
    QpTorqueAllocator(double r_w, double t_f, double t_r, double t_max, double mu, double w_fx,
                      double w_mz, double w_reg)
        : r_w_(r_w), t_max_(t_max), mu_(mu) {
        // --- Effectiveness matrix B (2×4) ---
        const double inv_r = 1.0 / r_w;
        const double tf_term = t_f / (2.0 * r_w);
        const double tr_term = t_r / (2.0 * r_w);

        Eigen::Matrix<double, 2, 4> b_mat;
        // clang-format off
        b_mat <<  inv_r,    inv_r,    inv_r,    inv_r,
                 -tf_term,  tf_term, -tr_term,  tr_term;
        // clang-format on

        // --- W1 = diag(w_fx, w_mz) ---
        Eigen::Matrix<double, 2, 2> w1_mat = Eigen::Matrix<double, 2, 2>::Zero();
        w1_mat(0, 0) = w_fx;
        w1_mat(1, 1) = w_mz;

        // Precompute BᵀW₁ — reused each cycle for gradient q = −2·BᵀW₁·v
        bt_w1_ = b_mat.transpose() * w1_mat;

        // --- P = 2(BᵀW₁B + W₂), upper triangular sparse (OSQP convention) ---
        const Eigen::Matrix<double, 4, 4> p_dense =
            2.0 * ((bt_w1_ * b_mat) + (w_reg * Eigen::Matrix<double, 4, 4>::Identity()));

        Eigen::SparseMatrix<double> p_upper(4, 4);
        for (int i = 0; i < 4; ++i) {
            for (int j = i; j < 4; ++j) {
                p_upper.insert(i, j) = p_dense(i, j);
            }
        }
        p_upper.makeCompressed();

        // --- Constraint matrix A = I₄ (encodes box constraints) ---
        Eigen::SparseMatrix<double> a_sparse(4, 4);
        a_sparse.setIdentity();

        // --- Configure OSQP ---
        solver_.settings()->setWarmStart(true);
        solver_.settings()->setVerbosity(false);
        solver_.settings()->setPolish(true);
        solver_.settings()->setMaxIteration(500);
        solver_.settings()->setAbsoluteTolerance(1e-5);
        solver_.settings()->setRelativeTolerance(1e-5);

        solver_.data()->setNumberOfVariables(4);
        solver_.data()->setNumberOfConstraints(4);
        solver_.data()->setHessianMatrix(p_upper);

        Eigen::VectorXd q_init = Eigen::VectorXd::Zero(4);
        solver_.data()->setGradient(q_init);
        solver_.data()->setLinearConstraintsMatrix(a_sparse);

        Eigen::VectorXd lb_init = Eigen::VectorXd::Constant(4, -t_max);
        Eigen::VectorXd ub_init = Eigen::VectorXd::Constant(4, t_max);
        solver_.data()->setLowerBound(lb_init);
        solver_.data()->setUpperBound(ub_init);

        solver_.initSolver();
    }

    // VehicleState:    [vx, vy, yaw_rate, steering_angle, ax, ay]
    // YawMomentCommand: {mz_, fx_total_}
    // ForceVector:     [Fx×4, Fy×4, Fz×4]
    [[nodiscard]] WheelTorques allocateImpl(const VehicleState& state, const YawMomentCommand& cmd,
                                            const ForceVector& forces) {
        (void)state;  // reserved for speed-dependent torque maps

        // --- Gradient: q = −2·BᵀW₁·v ---
        const Eigen::Vector2d v_demand{cmd.fx_total_, cmd.mz_};
        const Eigen::VectorXd q_vec = -2.0 * (bt_w1_ * v_demand);
        solver_.updateGradient(q_vec);

        // --- Per-wheel bounds: motor cap ∩ friction circle ---
        // ForceVector: Fy = indices [4..7], Fz = indices [8..11]
        Eigen::VectorXd ub(4);
        Eigen::VectorXd lb(4);
        for (int i = 0; i < 4; ++i) {
            const double fz = std::max(0.0, forces.fz_[i]);
            const double fy = forces.fy_[i];
            const double fx_cap = std::sqrt(std::max(0.0, ((mu_ * fz) * (mu_ * fz)) - (fy * fy)));
            ub[i] = std::min(t_max_, r_w_ * fx_cap);
            lb[i] = -ub[i];
        }
        solver_.updateBounds(lb, ub);

        switch (solver_.solveProblem()) {
            case OsqpEigen::ErrorExitFlag::NoError:
                return solver_.getSolution().head<4>();
                break;
            default:
                return WheelTorques::Zero();
        }
    }

   private:
    // Vehicle geometry
    double r_w_;  // wheel radius [m]

    // Limits
    double t_max_;  // per-wheel motor torque cap [Nm]
    double mu_;     // tyre-road friction coefficient

    // Precomputed BᵀW₁ (4×2, constant across cycles)
    Eigen::Matrix<double, 4, 2> bt_w1_;

    // OSQP solver — maintains warm-start state between control cycles
    OsqpEigen::Solver solver_;
};

}  // namespace tv
