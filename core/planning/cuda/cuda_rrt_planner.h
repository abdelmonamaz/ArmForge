#pragma once
#include "../i_motion_planner.h"
#include "../obstacle.h"
#include "../../model/joint.h"

#include <Eigen/Core>
#include <functional>
#include <random>
#include <vector>

namespace af {

// GPU-accelerated counterpart to RrtConnectPlanner — same bidirectional
// RRT-Connect strategy (Kuffner & LaValle, 2000), intentionally mirrored so
// the two are easy to compare side by side. The only thing that changes is
// *how* a candidate segment is validated:
//   - RrtConnectPlanner  : discretizes the segment and calls isStateValid()
//                          once per point, sequentially, on the CPU.
//   - CudaRrtPlanner     : computes FK for every discretized point on the
//                          host, then validates the *entire batch* of frame-
//                          point clouds against every obstacle in ONE GPU
//                          kernel launch (collision_kernel.cu).
//
// Needs its own obstacle list and forward-kinematics function (rather than
// the single-state isStateValid callback) because batching requires building
// the whole point cloud up front before handing it to the GPU.
class CudaRrtPlanner : public IMotionPlanner {
public:
    // Maps a joint configuration (radians) to the chain's frame origins —
    // the same "stick figure" sample points RrtConnectPlanner's caller uses
    // for collision checks (e.g. RobotController::visualFrames).
    using FkFn = std::function<std::vector<Eigen::Vector3d>(const std::vector<double>&)>;

    struct Params {
        double stepSize             = 0.12;   // rad — max extension per tree-growth step
        double collisionResolution  = 0.05;   // rad — segment discretization for collision checks
        int    maxIterations        = 2000;
    };

    CudaRrtPlanner(std::vector<JointLimits> limits,
                   std::vector<SphereObstacle> obstacles,
                   FkFn fk,
                   double margin,
                   Params params = {});

    PlanResult plan(const PlanRequest& request) const override;

    // True if a CUDA device is present at runtime (checked lazily, cached).
    static bool gpuAvailable();

private:
    struct Node {
        std::vector<double> q;
        int                 parent = -1;
    };

    enum class ExtendResult { Reached, Advanced, Trapped };

    // Discretizes a→b, runs FK for every step, and validates the whole batch
    // of resulting point clouds in a single GPU kernel launch.
    bool segmentValidGpu(const std::vector<double>& a, const std::vector<double>& b) const;

    ExtendResult extend(std::vector<Node>& tree, const std::vector<double>& target) const;

    static std::vector<std::vector<double>> extractPath(const std::vector<Node>& tree, int leafIdx);

    std::vector<JointLimits>    m_limits;
    std::vector<SphereObstacle> m_obstacles;
    FkFn                        m_fk;
    double                      m_margin;
    Params                      m_params;
};

} // namespace af
