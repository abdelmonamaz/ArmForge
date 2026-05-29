#pragma once
#include <functional>
#include <vector>

namespace af {

// True if the given joint configuration (radians) is collision-free.
// Supplied by the caller so the planner stays decoupled from any specific
// forward-kinematics convention or collision representation.
using StateValidityFn = std::function<bool(const std::vector<double>&)>;

struct PlanRequest {
    std::vector<double> start;          // joint config, radians
    std::vector<double> goal;           // joint config, radians
    StateValidityFn     isStateValid;
};

struct PlanResult {
    std::vector<std::vector<double>> path;        // start ... goal, radians (empty if !success)
    bool                             success    = false;
    int                              iterations = 0;
};

// Abstract motion-planning strategy — the Phase 5 extension point.
// `CudaRrtPlanner` will implement the same interface with GPU-accelerated
// collision batches; the rest of the app never needs to know which one runs.
class IMotionPlanner {
public:
    virtual ~IMotionPlanner() = default;
    virtual PlanResult plan(const PlanRequest& request) const = 0;
};

} // namespace af
