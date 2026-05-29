#include "kinematic_chain.h"
#include <numbers>
#include <stdexcept>

namespace af {

KinematicChain::KinematicChain(std::vector<ChainAxis> axes)
    : m_axes(std::move(axes)) {}

void KinematicChain::setAngle(int idx, double rad)
{
    m_axes.at(idx).joint.position = rad;
}

double KinematicChain::angle(int idx) const
{
    return m_axes.at(idx).joint.position;
}

KinematicChain KinematicChain::make6dof()
{
    using namespace std::numbers;
    // UR5 DH parameters — standard DH, units: metres, radians.
    // (a, alpha, d, theta_offset)
    const std::vector<ChainAxis> axes = {
        { { 0.000,    pi/2,  0.0892, 0.0 }, { "joint1" } },
        { { -0.4250,  0.0,   0.0000, 0.0 }, { "joint2" } },
        { { -0.3922,  0.0,   0.0000, 0.0 }, { "joint3" } },
        { { 0.000,    pi/2,  0.1093, 0.0 }, { "joint4" } },
        { { 0.000,   -pi/2,  0.0947, 0.0 }, { "joint5" } },
        { { 0.000,    0.0,   0.0823, 0.0 }, { "joint6" } },
    };
    return KinematicChain(axes);
}

} // namespace af
