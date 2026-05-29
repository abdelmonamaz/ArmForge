#include "forward_kinematics.h"
#include <cmath>

namespace af {

ForwardKinematics::ForwardKinematics(const KinematicChain& chain)
    : m_chain(chain) {}

// Standard DH transform :
//   T_i = Rot_z(theta+q) · Trans_z(d) · Trans_x(a) · Rot_x(alpha)
Pose ForwardKinematics::dhTransform(const DHParameters& dh, double q)
{
    const double ct = std::cos(dh.theta + q);
    const double st = std::sin(dh.theta + q);
    const double ca = std::cos(dh.alpha);
    const double sa = std::sin(dh.alpha);

    Eigen::Matrix4d T;
    T <<  ct,  -st * ca,   st * sa,  dh.a * ct,
          st,   ct * ca,  -ct * sa,  dh.a * st,
         0.0,        sa,        ca,       dh.d,
         0.0,       0.0,       0.0,        1.0;

    Pose p;
    p.matrix() = T;
    return p;
}

Pose ForwardKinematics::compute(const std::vector<double>& q) const
{
    Pose T = Pose::Identity();
    for (int i = 0; i < m_chain.dof(); ++i)
        T = T * dhTransform(m_chain.axes()[i].dh, q[i]);
    return T;
}

std::vector<Pose> ForwardKinematics::allFrames(const std::vector<double>& q) const
{
    std::vector<Pose> frames;
    frames.reserve(m_chain.dof() + 1);
    frames.push_back(Pose::Identity());
    for (int i = 0; i < m_chain.dof(); ++i)
        frames.push_back(frames.back() * dhTransform(m_chain.axes()[i].dh, q[i]));
    return frames;
}

} // namespace af
