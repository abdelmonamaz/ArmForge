#pragma once
#include <Eigen/Core>

namespace af {

// Spherical obstacle — the simplest collision primitive that still produces
// meaningful "basic obstacle avoidance" demos. Phase 5's CudaRrtPlanner can
// reuse the same primitive for parallel GPU distance batches.
struct SphereObstacle {
    Eigen::Vector3d center;
    double          radius = 0.0;
};

} // namespace af
