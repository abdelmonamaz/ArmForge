#pragma once
#include "i_motion_planner.h"
#include "../model/joint.h"
#include <random>
#include <vector>

namespace af {

// RRT-Connect (Kuffner & LaValle, 2000) — joint-space planner with bidirectional
// trees grown from start and goal, connected via greedy "Connect" extensions.
// CPU reference implementation; `CudaRrtPlanner` (Phase 5) parallelizes the
// collision-checking step on GPU behind the same IMotionPlanner interface.
class RrtConnectPlanner : public IMotionPlanner {
public:
    struct Params {
        double stepSize             = 0.12;   // rad — max extension per tree-growth step
        double collisionResolution  = 0.05;   // rad — segment discretization for collision checks
        int    maxIterations        = 2000;
    };

    explicit RrtConnectPlanner(std::vector<JointLimits> limits, Params params = {});

    PlanResult plan(const PlanRequest& request) const override;

private:
    struct Node {
        std::vector<double> q;
        int                 parent = -1;
    };

    enum class ExtendResult { Reached, Advanced, Trapped };

    ExtendResult extend(std::vector<Node>& tree,
                        const std::vector<double>& target,
                        const StateValidityFn& isValid) const;

    static std::vector<std::vector<double>> extractPath(const std::vector<Node>& tree, int leafIdx);

    std::vector<JointLimits> m_limits;
    Params                   m_params;
};

} // namespace af
