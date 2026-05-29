#include "rrt_connect_planner.h"
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

    // Discretized collision check along the straight joint-space segment a→b.
    // StateValidityFn is std::function by design — it's the public
    // IMotionPlanner::PlanRequest field type, already type-erased at that
    // boundary. Templating this internal helper would just re-bind to the
    // same std::function, with no erasure to actually remove.
    bool segmentValid(const std::vector<double>& a, const std::vector<double>& b,
                      const StateValidityFn& isValid, double resolution) // NOSONAR (S5213)
    {
        const double d = distance(a, b);
        const int steps = std::max(1, static_cast<int>(std::ceil(d / resolution)));
        for (int i = 0; i <= steps; ++i) {
            const double t = static_cast<double>(i) / steps;
            std::vector<double> q(a.size());
            for (std::size_t j = 0; j < a.size(); ++j) {
                q[j] = a[j] + t * (b[j] - a[j]);
            }
            if (!isValid(q)) {
                return false;
            }
        }
        return true;
    }
}

RrtConnectPlanner::RrtConnectPlanner(std::vector<JointLimits> limits, Params params)
    : m_limits(std::move(limits))
    , m_params(params)
{
}

RrtConnectPlanner::ExtendResult RrtConnectPlanner::extend(std::vector<Node>& tree,
                                                           const std::vector<double>& target,
                                                           const StateValidityFn& isValid) const
{
    int nearest = 0;
    double nearestDist = distance(tree[0].q, target);
    for (std::size_t i = 1; i < tree.size(); ++i) {
        const double d = distance(tree[i].q, target);
        if (d < nearestDist) {
            nearestDist = d;
            nearest = static_cast<int>(i);
        }
    }

    const std::vector<double> q_new = steer(tree[nearest].q, target, m_params.stepSize);
    if (!segmentValid(tree[nearest].q, q_new, isValid, m_params.collisionResolution)) {
        return ExtendResult::Trapped;
    }

    tree.emplace_back(q_new, nearest);
    return (distance(q_new, target) < 1e-9) ? ExtendResult::Reached : ExtendResult::Advanced;
}

std::vector<std::vector<double>> RrtConnectPlanner::extractPath(const std::vector<Node>& tree, int leafIdx)
{
    std::vector<std::vector<double>> path;
    for (int i = leafIdx; i != -1; i = tree[i].parent) {
        path.push_back(tree[i].q);
    }
    std::ranges::reverse(path);   // root → leaf
    return path;
}

// Δq = grow tree A toward a random sample, then greedily "Connect" tree B
// toward the new node. A path is found the moment the two trees touch; the
// trees swap roles every iteration so both grow from start and from goal.
PlanResult RrtConnectPlanner::plan(const PlanRequest& request) const
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

        if (extend(treeFrom, q_rand, request.isStateValid) == ExtendResult::Trapped) {
            growStart = !growStart;
            continue;
        }

        const std::vector<double> q_new = treeFrom.back().q;

        ExtendResult connectResult;
        do {
            connectResult = extend(treeTo, q_new, request.isStateValid);
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
