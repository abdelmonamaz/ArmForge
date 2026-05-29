#include "ik_solver.h"
#include <cmath>

namespace af {

IKSolver::IKSolver(const KinematicChain& chain, Params p)
    : m_chain(chain), m_fk(chain), m_params(p) {}

Eigen::Matrix<double, 3, 6>
IKSolver::jacobian(const std::vector<double>& q) const
{
    const Eigen::Vector3d p0  = m_fk.compute(q).translation();
    const double          eps = m_params.eps;
    Eigen::Matrix<double, 3, 6> J;
    auto qp = q;
    for (int i = 0; i < 6; ++i) {
        qp[i] += eps;
        J.col(i) = (m_fk.compute(qp).translation() - p0) / eps;
        qp[i] = q[i];
    }
    return J;
}

IKResult IKSolver::solvePosition(const Eigen::Vector3d& target,
                                  const std::vector<double>& q0) const
{
    const double lam2 = m_params.lambda * m_params.lambda;
    IKResult res;
    res.q = q0;

    for (int it = 0; it < m_params.maxIter; ++it) {
        const Eigen::Vector3d err = target - m_fk.compute(res.q).translation();
        res.residual   = err.norm();
        res.iterations = it + 1;
        if (res.residual < m_params.tol) { res.converged = true; break; }

        const auto J  = jacobian(res.q);
        // DLS step: Δq = J^T (J J^T + λ²I)^{-1} err
        const auto A  = J * J.transpose() + lam2 * Eigen::Matrix3d::Identity();
        const Eigen::Matrix<double, 6, 1> dq = J.transpose() * A.ldlt().solve(err);

        for (int i = 0; i < 6; ++i) {
            res.q[i] = std::clamp(
                res.q[i] + dq[i],
                m_chain.axes()[i].joint.limits.min,
                m_chain.axes()[i].joint.limits.max);
        }
    }
    return res;
}

} // namespace af
