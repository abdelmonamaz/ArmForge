#include "planning_controller.h"
#include "robot_controller.h"

#include "planning/collision.h"
#include "planning/obstacle.h"
#include "planning/rrt_connect_planner.h"
#include "model/joint.h"

#ifdef ARMFORGE_HAS_CUDA
#include "planning/cuda/collision_kernel.cuh"
#include "planning/cuda/cuda_rrt_planner.h"
#endif

#include <QDebug>
#include <QVariantMap>
#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <memory>
#include <numbers>
#include <random>

namespace {
    constexpr double kDeg2Rad = std::numbers::pi / 180.0;
    constexpr double kRad2Deg = 180.0 / std::numbers::pi;

    // Clearance kept between every chain frame and every obstacle surface.
    constexpr double kCollisionMargin = 25.0;   // scene units

    std::array<double, 6> toRad(const std::array<double, 6>& deg)
    {
        std::array<double, 6> rad{};
        for (int i = 0; i < 6; ++i) {
            rad[i] = deg[i] * kDeg2Rad;
        }
        return rad;
    }

    QVariantList toVariantList(const std::array<double, 6>& v)
    {
        QVariantList out;
        for (double x : v) {
            out.append(x);
        }
        return out;
    }
}

PlanningController::PlanningController(QObject* parent)
    : QObject(parent)
{
}

QVariantList PlanningController::waypoints() const
{
    QVariantList out;
    for (const auto& wp : m_waypoints) {
        QVariantMap m;
        m["ee"] = wp.eePos;
        m["q"]  = toVariantList(wp.q_deg);
        out.append(m);
    }
    return out;
}

QVariantList PlanningController::obstacles() const
{
    QVariantList out;
    for (const auto& obs : m_obstacles) {
        QVariantMap m;
        m["center"] = QVector3D(float(obs.center.x()), float(obs.center.y()), float(obs.center.z()));
        m["radius"] = obs.radius;
        out.append(m);
    }
    return out;
}

void PlanningController::setPlanning(bool v)
{
    if (m_planning == v) {
        return;
    }
    m_planning = v;
    emit planningChanged();
}

bool PlanningController::cudaAvailable() const
{
#ifdef ARMFORGE_HAS_CUDA
    return af::cuda::isAvailable();
#else
    return false;
#endif
}

void PlanningController::setUseGpuPlanner(bool v)
{
    if (m_useGpuPlanner == v) {
        return;
    }
    m_useGpuPlanner = v;
    emit useGpuPlannerChanged();
}

void PlanningController::addWaypoint(QVariantList configDeg)
{
    if (configDeg.size() != 6) {
        return;
    }

    Waypoint wp;
    for (int i = 0; i < 6; ++i) {
        wp.q_deg[i] = configDeg[i].toDouble();
    }
    wp.eePos = RobotController::visualFrames(toRad(wp.q_deg)).back();

    m_waypoints.push_back(wp);
    emit waypointsChanged();
}

void PlanningController::removeWaypoint(int index)
{
    if (index < 0 || std::size_t(index) >= m_waypoints.size()) {
        return;
    }
    m_waypoints.erase(m_waypoints.begin() + index);
    emit waypointsChanged();
}

void PlanningController::clearWaypoints()
{
    if (m_waypoints.empty()) {
        return;
    }
    m_waypoints.clear();
    emit waypointsChanged();
}

// Plans current → waypoint[0] → waypoint[1] → ... as a chain of RRT-Connect
// legs in joint space, concatenates them into one trajectory, and exposes
// both the joint-space path (for replay) and its EE trace (for the 3-D view).
void PlanningController::plan(QVariantList currentConfigDeg)
{
    qDebug() << "[PLAN] plan() called — waypoints=" << int(m_waypoints.size())
             << " currentConfigDeg.size=" << currentConfigDeg.size();

    if (m_waypoints.empty() || currentConfigDeg.size() != 6) {
        qDebug() << "[PLAN] aborted — no waypoints or malformed current config";
        return;
    }

    setPlanning(true);

    std::array<double, 6> currentDeg{};
    for (int i = 0; i < 6; ++i) {
        currentDeg[i] = currentConfigDeg[i].toDouble();
    }
    qDebug() << "[PLAN] start config (deg):" << currentDeg[0] << currentDeg[1] << currentDeg[2]
             << currentDeg[3] << currentDeg[4] << currentDeg[5];

    // Shared by every leg: maps a joint config (radians) to the chain's frame
    // origins — the same "stick figure" sample points used for collision
    // checks, and (in GPU mode) for building the batched point clouds.
    auto toPointCloud = [](const std::vector<double>& q_rad) {
        std::array<double, 6> q{};
        for (int i = 0; i < 6; ++i) {
            q[i] = q_rad[i];
        }
        const auto frames = RobotController::visualFrames(q);

        std::vector<Eigen::Vector3d> points;
        points.reserve(frames.size());
        for (const auto& f : frames) {
            points.emplace_back(f.x(), f.y(), f.z());
        }
        return points;
    };

    const auto obstacles = m_obstacles;
    auto isValid = [obstacles, toPointCloud](const std::vector<double>& q_rad) {
        return af::isClearOfObstacles(toPointCloud(q_rad), obstacles, kCollisionMargin);
    };

    const std::vector<af::JointLimits> limits(6);   // default: ±π, plenty for this demo arm

    // Phase 5 — pick CudaRrtPlanner (GPU-batched segment validation) when the
    // user opted in AND a CUDA device is actually present; CPU otherwise.
    // Both implement IMotionPlanner, so the rest of this function is identical
    // either way — the only thing that changes is *how* segments get validated.
    std::unique_ptr<af::IMotionPlanner> planner;
    bool gpuMode = false;
#ifdef ARMFORGE_HAS_CUDA
    if (m_useGpuPlanner && af::cuda::isAvailable()) {
        planner = std::make_unique<af::CudaRrtPlanner>(limits, obstacles, toPointCloud, kCollisionMargin);
        gpuMode = true;
    }
#endif
    if (!planner) {
        planner = std::make_unique<af::RrtConnectPlanner>(limits);
    }
    qDebug() << "[PLAN] planner ready — mode=" << (gpuMode ? "GPU" : "CPU")
             << " limits=" << int(limits.size()) << " obstacles=" << int(obstacles.size());

    const std::array<double, 6> startRad = toRad(currentDeg);
    std::vector<double> from(startRad.begin(), startRad.end());
    std::vector<std::vector<double>> fullPath{ from };

    bool ok = true;
    int totalIterations = 0;
    int legIdx = 0;

    for (const auto& wp : m_waypoints) {
        const auto goalRad = toRad(wp.q_deg);
        const af::PlanRequest request{ from, std::vector<double>(goalRad.begin(), goalRad.end()), isValid };

        qDebug() << "[PLAN] leg" << legIdx << "— start.size=" << request.start.size()
                 << " goal.size=" << request.goal.size() << " — running RRT-Connect…";

        const af::PlanResult result = planner->plan(request);
        totalIterations += result.iterations;

        qDebug() << "[PLAN] leg" << legIdx << "result — success=" << result.success
                 << " iterations=" << result.iterations << " path.size=" << result.path.size();

        if (!result.success) {
            ok = false;
            break;
        }

        // Skip index 0 — it duplicates the previous leg's final config.
        fullPath.insert(fullPath.end(), result.path.begin() + 1, result.path.end());
        from = request.goal;
        ++legIdx;
    }

    qDebug() << "[PLAN] all legs done — ok=" << ok << " totalIterations=" << totalIterations
             << " fullPath.size=" << fullPath.size();

    m_lastPlanSuccess    = ok;
    m_lastPlanIterations = totalIterations;

    m_pathPoints.clear();
    m_pathConfigsDeg.clear();
    if (ok) {
        m_pathPoints.reserve(int(fullPath.size()));
        m_pathConfigsDeg.reserve(int(fullPath.size()));
        for (std::size_t i = 0; i < fullPath.size(); ++i) {
            const auto& q_rad = fullPath[i];
            if (q_rad.size() != 6) {
                qDebug() << "[PLAN] !! malformed path step" << i << "— size=" << q_rad.size();
                continue;
            }
            std::array<double, 6> q{};
            QVariantList cfgDeg;
            for (int j = 0; j < 6; ++j) {
                q[j] = q_rad[j];
                cfgDeg.append(q_rad[j] * kRad2Deg);
            }
            m_pathConfigsDeg.append(QVariant(cfgDeg));
            m_pathPoints.append(RobotController::visualFrames(q).back());
        }
        qDebug() << "[PLAN] trace built — pathPoints=" << m_pathPoints.size()
                 << " pathConfigsDeg=" << m_pathConfigsDeg.size();
    }

    setPlanning(false);
    qDebug() << "[PLAN] done — emitting pathChanged()";
    emit pathChanged();
}

// Builds `sampleCount` random joint configs and their FK frame-point clouds
// once (shared, untimed setup), then validates that exact same batch through
// two independent code paths — a sequential CPU loop and a single GPU kernel
// launch — timing each so the UI can compare them apples-to-apples.
void PlanningController::runBenchmark(int sampleCount)
{
    sampleCount = std::clamp(sampleCount, 100, 200000);

    std::mt19937 rng{ std::random_device{}() };
    std::uniform_real_distribution angle(-std::numbers::pi, std::numbers::pi);

    std::vector<std::vector<Eigen::Vector3d>> clouds;
    clouds.reserve(sampleCount);
    for (int i = 0; i < sampleCount; ++i) {
        std::array<double, 6> q{};
        for (double& a : q) {
            a = angle(rng);
        }
        const auto frames = RobotController::visualFrames(q);
        std::vector<Eigen::Vector3d> cloud;
        cloud.reserve(frames.size());
        for (const auto& f : frames) {
            cloud.emplace_back(f.x(), f.y(), f.z());
        }
        clouds.push_back(std::move(cloud));
    }

    // ── CPU: sequential af::isClearOfObstacles loop ──────────────
    const auto cpuStart = std::chrono::steady_clock::now();
    for (const auto& cloud : clouds) {
        af::isClearOfObstacles(cloud, m_obstacles, kCollisionMargin);
    }
    const auto cpuEnd = std::chrono::steady_clock::now();
    m_benchmarkCpuMs = std::chrono::duration<double, std::milli>(cpuEnd - cpuStart).count();

    // ── GPU: one batched kernel launch over the whole sample set ─
    m_benchmarkGpuMs = 0.0;
#ifdef ARMFORGE_HAS_CUDA
    if (af::cuda::isAvailable() && !clouds.empty()) {
        const auto pointsPerConfig = static_cast<int>(clouds.front().size());

        std::vector<float> points(static_cast<std::size_t>(sampleCount) * pointsPerConfig * 3);
        for (int c = 0; c < sampleCount; ++c) {
            for (int p = 0; p < pointsPerConfig; ++p) {
                const auto& v = clouds[c][p];
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
        std::vector<unsigned char> valid(sampleCount, 0);

        const auto gpuStart = std::chrono::steady_clock::now();
        af::cuda::batchValidate(points.data(), sampleCount, pointsPerConfig,
                                centers.data(), radii.data(), static_cast<int>(m_obstacles.size()),
                                static_cast<float>(kCollisionMargin), valid.data());
        const auto gpuEnd = std::chrono::steady_clock::now();
        m_benchmarkGpuMs = std::chrono::duration<double, std::milli>(gpuEnd - gpuStart).count();
    }
#endif

    m_benchmarkSampleCount = sampleCount;
    m_benchmarkRan         = true;

    qDebug() << "[BENCH] samples=" << sampleCount
             << " cpuMs=" << m_benchmarkCpuMs
             << " gpuMs=" << m_benchmarkGpuMs;

    emit benchmarkChanged();
}
