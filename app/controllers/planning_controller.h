#pragma once
#include "planning/obstacle.h"

#include <QObject>
#include <QQmlEngine>
#include <QVariantList>
#include <QVector3D>
#include <array>
#include <vector>

// Phase 4 — joint-space motion planning with basic obstacle avoidance.
//
// Workflow:
//   1. Drive the arm (FK sliders or IK gizmo) to a pose, then addWaypoint() —
//      a "teach point" capturing the current joint configuration.
//   2. plan() runs RRT-Connect (CPU, AF_Core) between consecutive configs,
//      checking each candidate against a small set of fixed sphere obstacles.
//   3. RobotController::followPath() replays the resulting trajectory through
//      the existing 50 Hz ControlLoop — same LERP machinery as IK mode.
//
// `pathPoints` / `obstacles` feed PlanningView.qml's 3-D visualization
// (EE trace + obstacle spheres + waypoint markers).
class PlanningController : public QObject
{
    Q_OBJECT
    QML_ELEMENT
    QML_SINGLETON

    Q_PROPERTY(QVariantList waypoints          READ waypoints          NOTIFY waypointsChanged)
    Q_PROPERTY(QVariantList obstacles          READ obstacles          CONSTANT)
    Q_PROPERTY(QVariantList pathPoints         READ pathPoints         NOTIFY pathChanged)
    Q_PROPERTY(QVariantList pathConfigsDeg     READ pathConfigsDeg     NOTIFY pathChanged)
    Q_PROPERTY(bool         planning           READ planning           NOTIFY planningChanged)
    Q_PROPERTY(bool         lastPlanSuccess    READ lastPlanSuccess    NOTIFY pathChanged)
    Q_PROPERTY(int          lastPlanIterations READ lastPlanIterations NOTIFY pathChanged)

    // Phase 5 — when true (and CUDA is available), plan() runs CudaRrtPlanner
    // (GPU-batched segment validation) instead of the CPU RrtConnectPlanner.
    // Falls back to CPU transparently if CUDA isn't available.
    Q_PROPERTY(bool useGpuPlanner READ useGpuPlanner WRITE setUseGpuPlanner NOTIFY useGpuPlannerChanged)

    // ── Phase 5 — CPU vs GPU collision-check benchmark ───────────
    // `cudaAvailable` reflects whether ArmForge was built with CUDA support
    // AND a usable device is present at runtime — the UI hides/disables the
    // GPU side of the benchmark when false (transparent CPU fallback).
    Q_PROPERTY(bool   cudaAvailable        READ cudaAvailable        CONSTANT)
    Q_PROPERTY(bool   benchmarkRan         READ benchmarkRan         NOTIFY benchmarkChanged)
    Q_PROPERTY(int    benchmarkSampleCount READ benchmarkSampleCount NOTIFY benchmarkChanged)
    Q_PROPERTY(double benchmarkCpuMs       READ benchmarkCpuMs       NOTIFY benchmarkChanged)
    Q_PROPERTY(double benchmarkGpuMs       READ benchmarkGpuMs       NOTIFY benchmarkChanged)
    Q_PROPERTY(double benchmarkSpeedup     READ benchmarkSpeedup     NOTIFY benchmarkChanged)

public:
    explicit PlanningController(QObject* parent = nullptr);

    QVariantList waypoints()      const;
    QVariantList obstacles()      const;
    QVariantList pathPoints()     const { return m_pathPoints; }
    QVariantList pathConfigsDeg() const { return m_pathConfigsDeg; }
    bool         planning()       const { return m_planning; }
    bool         lastPlanSuccess()    const { return m_lastPlanSuccess; }
    int          lastPlanIterations() const { return m_lastPlanIterations; }

    bool useGpuPlanner() const { return m_useGpuPlanner; }
    void setUseGpuPlanner(bool v);

    bool   cudaAvailable()        const;
    bool   benchmarkRan()         const { return m_benchmarkRan; }
    int    benchmarkSampleCount() const { return m_benchmarkSampleCount; }
    double benchmarkCpuMs()       const { return m_benchmarkCpuMs; }
    double benchmarkGpuMs()       const { return m_benchmarkGpuMs; }
    double benchmarkSpeedup()     const { return (m_benchmarkGpuMs > 0.0) ? m_benchmarkCpuMs / m_benchmarkGpuMs : 0.0; }

    // Captures the given joint config (degrees, as in RobotController.q0..q5)
    // as a teach-point waypoint, recording its end-effector position too.
    Q_INVOKABLE void addWaypoint(QVariantList configDeg);
    Q_INVOKABLE void removeWaypoint(int index);
    Q_INVOKABLE void clearWaypoints();

    // Plans a path from `currentConfigDeg` through every waypoint in order.
    // Populates pathPoints / pathConfigsDeg; success/iterations report the
    // outcome of the whole multi-leg plan (the worst-case leg, concatenated).
    Q_INVOKABLE void plan(QVariantList currentConfigDeg);

    // Validates `sampleCount` random joint configurations against the fixed
    // obstacle layout twice — once via a sequential CPU loop
    // (af::isClearOfObstacles), once via a single batched GPU kernel launch
    // (af::cuda::batchValidate, when CUDA is available) — and records the
    // elapsed wall-clock time for each so the UI can compare them.
    Q_INVOKABLE void runBenchmark(int sampleCount);

signals:
    void waypointsChanged();
    void pathChanged();
    void planningChanged();
    void benchmarkChanged();
    void useGpuPlannerChanged();

private:
    struct Waypoint {
        std::array<double, 6> q_deg{};
        QVector3D             eePos;
    };

    void setPlanning(bool v);

    std::vector<Waypoint>           m_waypoints;

    // Fixed demo layout — a couple of spheres sitting in the arm's reach
    // (workspace radius ≈ 1000 scene units; see RobotController::visualEE).
    std::vector<af::SphereObstacle> m_obstacles{
        { Eigen::Vector3d(480.0, 220.0,  220.0), 110.0 },
        { Eigen::Vector3d(260.0, 380.0, -200.0),  90.0 },
    };

    QVariantList m_pathPoints;        // EE trace — QVector3D per path step (scene units)
    QVariantList m_pathConfigsDeg;    // joint configs for RobotController::followPath()

    bool m_planning           = false;
    bool m_lastPlanSuccess    = false;
    int  m_lastPlanIterations = 0;
    bool m_useGpuPlanner      = false;

    bool   m_benchmarkRan         = false;
    int    m_benchmarkSampleCount = 0;
    double m_benchmarkCpuMs       = 0.0;
    double m_benchmarkGpuMs       = 0.0;
};
