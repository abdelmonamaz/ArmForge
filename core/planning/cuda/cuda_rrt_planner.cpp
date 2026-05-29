#include "cuda_rrt_planner.h"
#include "collision_kernel.cuh"

#include <algorithm>
#include <cmath>

namespace af {

namespace {
    double distance(const std::vector<double>& a, const std::vector<double>& b)
    {
        double sum = 0.0;
        for (std::size_t i = 0; i < a.size(); ++i) {
            const double d = a[i] - b[i];
            sum += d * d;
        }
        return std::sqrt(sum);
    }

    // Step at most `stepSize` from `from` toward `to`; returns `to` itself if closer.
    std::vector<double> steer(const std::vector<double>& from, const std::vector<double>& to, double stepSize)
    {
        const double d = distance(from, to);
        if (d <= stepSize) {
            return to;
        }
        std::vector<double> out(from.size());
        const double t = stepSize / d;
        for (std::size_t i = 0; i < from.size(); ++i) {
            out[i] = from[i] + (to[i] - from[i]) * t;
        }
        return out;
    }
}

CudaRrtPlanner::CudaRrtPlanner(std::vector<JointLimits> limits,
                               std::vector<SphereObstacle> obstacles,
                               FkFn fk,
                               double margin,
                               Params params)
    : m_limits(std::move(limits))
    , m_obstacles(std::move(obstacles))
    , m_fk(std::move(fk))
    , m_margin(margin)
    , m_params(params)
{
}

bool CudaRrtPlanner::gpuAvailable()
{
    return af::cuda::isAvailable();
}

// Discretizes a→b at `collisionResolution` (same scheme as
// RrtConnectPlanner::segmentValid), runs FK for every step on the host —
// a handful of 4x4 matrix products, negligible cost — then ships every
// resulting frame-point cloud to the GPU in ONE batchValidate() call instead
// of N sequential isStateValid() round-trips.
bool CudaRrtPlanner::segmentValidGpu(const std::vector<double>& a, const std::vector<double>& b) const
{
    const double d = distance(a, b);
    const int steps = std::max(1, static_cast<int>(std::ceil(d / m_params.collisionResolution)));
    const int numConfigs = steps + 1;

    std::vector<std::vector<Eigen::Vector3d>> frameSets;
    frameSets.reserve(numConfigs);

    int pointsPerConfig = 0;
    for (int i = 0; i <= steps; ++i) {
        const double t = static_cast<double>(i) / steps;
        std::vector<double> q(a.size());
        for (std::size_t j = 0; j < a.size(); ++j) {
            q[j] = a[j] + t * (b[j] - a[j]);
        }
        auto frames = m_fk(q);
        pointsPerConfig = static_cast<int>(frames.size());
        frameSets.push_back(std::move(frames));
    }
    if (pointsPerConfig == 0) {
        return true;   // nothing to check against
    }

    std::vector<float> points(static_cast<std::size_t>(numConfigs) * pointsPerConfig * 3);
    for (int c = 0; c < numConfigs; ++c) {
        for (int p = 0; p < pointsPerConfig; ++p) {
            const auto& v = frameSets[c][p];
            const std::size_t base = (static_cast<std::size_t>(c) * pointsPerConfig + p) * 3;
            points[base + 0] = static_cast<float>(v.x());
            points[base + 1] = static_cast<float>(v.y());
            points[base + 2] = static_cast<float>(v.z());
        }
    }

    std::vector<float> centers(m_obstacles.size() * 3);
    std::vector<float> radii(m_obstacles.size());
    for (std::size_t i = 0; i < m_obstacles.size(); ++i) {
        centers[i * 3 + 0] = static_cast<float>(m_obstacles[i].center.x());
        centers[i * 3 + 1] = static_cast<float>(m_obstacles[i].center.y());
        centers[i * 3 + 2] = static_cast<float>(m_obstacles[i].center.z());
        radii[i]           = static_cast<float>(m_obstacles[i].radius);
    }

    std::vector<unsigned char> valid(numConfigs, 0);
    af::cuda::batchValidate(points.data(), numConfigs, pointsPerConfig,
                            centers.data(), radii.data(), static_cast<int>(m_obstacles.size()),
                            static_cast<float>(m_margin), valid.data());

    return std::ranges::all_of(valid, [](unsigned char v) { return v != 0; });
}

CudaRrtPlanner::ExtendResult CudaRrtPlanner::extend(std::vector<Node>& tree, const std::vector<double>& target) const
{
    int nearest = 0;
    double nearestDist = distance(tree[0].q, target);
    for (std::size_t i = 1; i < tree.size(); ++i) {
        const double dd = distance(tree[i].q, target);
        if (dd < nearestDist) {
            nearestDist = dd;
            nearest = static_cast<int>(i);
        }
    }

    const std::vector<double> q_new = steer(tree[nearest].q, target, m_params.stepSize);
    if (!segmentValidGpu(tree[nearest].q, q_new)) {
        return ExtendResult::Trapped;
    }

    tree.emplace_back(q_new, nearest);
    return (distance(q_new, target) < 1e-9) ? ExtendResult::Reached : ExtendResult::Advanced;
}

std::vector<std::vector<double>> CudaRrtPlanner::extractPath(const std::vector<Node>& tree, int leafIdx)
{
    std::vector<std::vector<double>> path;
    for (int i = leafIdx; i != -1; i = tree[i].parent) {
        path.push_back(tree[i].q);
    }
    std::ranges::reverse(path);   // root → leaf
    return path;
}

// Identical bidirectional-tree strategy to RrtConnectPlanner::plan — only
// extend()'s validity check differs (GPU batch vs. CPU sequential). See that
// function's documentation for the algorithm description.
PlanResult CudaRrtPlanner::plan(const PlanRequest& request) const
{
    PlanResult result;
    const auto dof = request.start.size();
    if (dof == 0 || m_limits.size() != dof || request.goal.size() != dof) {
        return result;
    }
    if (!request.isStateValid(request.start) || !request.isStateValid(request.goal)) {
        return result;   // start or goal already in collision
    }

    std::mt19937 rng{ std::random_device{}() };
    std::vector<std::uniform_real_distribution<double>> sampler;
    sampler.reserve(dof);
    for (const auto& limit : m_limits) {
        sampler.emplace_back(limit.min, limit.max);
    }

    std::vector<Node> treeStart{ Node{ request.start, -1 } };
    std::vector<Node> treeGoal { Node{ request.goal,  -1 } };
    bool growStart = true;

    for (int iter = 0; iter < m_params.maxIterations; ++iter) {
        result.iterations = iter + 1;

        std::vector<double> q_rand(dof);
        for (std::size_t i = 0; i < dof; ++i) {
            q_rand[i] = sampler[i](rng);
        }

        auto& treeFrom = growStart ? treeStart : treeGoal;
        auto& treeTo   = growStart ? treeGoal  : treeStart;

        if (extend(treeFrom, q_rand) == ExtendResult::Trapped) {
            growStart = !growStart;
            continue;
        }

        const std::vector<double> q_new = treeFrom.back().q;

        ExtendResult connectResult;
        do {
            connectResult = extend(treeTo, q_new);
        } while (connectResult == ExtendResult::Advanced);

        if (connectResult == ExtendResult::Reached) {
            auto pathFrom = extractPath(treeFrom, static_cast<int>(treeFrom.size()) - 1);
            auto pathTo   = extractPath(treeTo,   static_cast<int>(treeTo.size())   - 1);

            auto& startSide = growStart ? pathFrom : pathTo;
            auto& goalSide  = growStart ? pathTo   : pathFrom;
            std::ranges::reverse(goalSide);   // q_new → goal

            result.path = std::move(startSide);
            result.path.insert(result.path.end(), goalSide.begin() + 1, goalSide.end());
            result.success = true;
            return result;
        }

        growStart = !growStart;
    }

    return result;
}

} // namespace af
