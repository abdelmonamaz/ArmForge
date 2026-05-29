#pragma once
#include "dh_parameters.h"
#include "joint.h"
#include <vector>

namespace af {

struct ChainAxis {
    DHParameters dh;
    Joint        joint;
};

class KinematicChain {
public:
    explicit KinematicChain(std::vector<ChainAxis> axes);

    int dof() const { return static_cast<int>(m_axes.size()); }

    const std::vector<ChainAxis>& axes() const { return m_axes; }
    std::vector<ChainAxis>&       axes()       { return m_axes; }

    void   setAngle(int idx, double rad);
    double angle(int idx) const;

    // Preset : UR5-like 6-DOF robot (metres, radians).
    static KinematicChain make6dof();

private:
    std::vector<ChainAxis> m_axes;
};

} // namespace af
