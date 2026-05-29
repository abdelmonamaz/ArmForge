#pragma once
#include "../model/kinematic_chain.h"
#include "../math/se3.h"
#include <vector>

namespace af {

class ForwardKinematics {
public:
    explicit ForwardKinematics(const KinematicChain& chain);

    // T_0_n : end-effector pose in world frame.
    Pose compute(const std::vector<double>& q) const;

    // One pose per frame, index 0 = world, index n = end-effector.
    std::vector<Pose> allFrames(const std::vector<double>& q) const;

private:
    static Pose dhTransform(const DHParameters& dh, double q);

    const KinematicChain& m_chain;
};

} // namespace af
