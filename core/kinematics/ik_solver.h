#pragma once
#include "forward_kinematics.h"
#include <Eigen/Dense>
#include <vector>

namespace af {

struct IKResult {
    std::vector<double> q;
    double residual   = 1e9;
    int    iterations = 0;
    bool   converged  = false;
};

// Damped Least Squares (DLS) IK solver — position-only, orientation unconstrained.
// Uses a numerical Jacobian (central-difference column-by-column perturbation).
class IKSolver {
public:
    struct Params {
        double lambda  = 0.05;  // DLS damping factor
        double eps     = 1e-4;  // Jacobian perturbation step (rad)
        double tol     = 5e-4;  // convergence threshold (metres)
        int    maxIter = 150;
    };

    explicit IKSolver(const KinematicChain& chain, Params p = {});

    // q0: initial guess in radians. target: desired EE position in world frame (metres).
    IKResult solvePosition(const Eigen::Vector3d& target,
                           const std::vector<double>& q0) const;

private:
    Eigen::Matrix<double, 3, 6> jacobian(const std::vector<double>& q) const;

    const KinematicChain& m_chain;
    ForwardKinematics     m_fk;
    Params                m_params;
};

} // namespace af
