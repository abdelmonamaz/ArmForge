#include "robot_controller.h"
#include "control_loop.h"
#include <Eigen/Dense>
#include <algorithm>
#include <numbers>
#include <cmath>

namespace {
    constexpr double kDeg2Rad = std::numbers::pi / 180.0;
    constexpr double kRad2Deg = 180.0 / std::numbers::pi;

    // DLS damping: λ² in (scene_units/rad)²
    // λ=50 u/rad ≈ 0.05 m/rad at kScale=1000 — matches standard DLS literature
    constexpr double kLambda2 = 50.0 * 50.0;
    constexpr double kEps     = 1e-3;   // Jacobian finite-difference step (rad)
    constexpr float  kTol     = 0.5f;   // convergence threshold (scene units)
    constexpr int    kMaxIter = 150;

    // S3609: no `static` inside anonymous namespace (already has internal linkage)
    Eigen::Matrix4d Trans(double x, double y, double z)
    {
        Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
        m(0,3)=x; m(1,3)=y; m(2,3)=z;
        return m;
    }
    Eigen::Matrix4d Rotx(double a)
    {
        Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
        // S1659: one declaration per statement
        const double c = std::cos(a);
        const double s = std::sin(a);
        m(1,1)=c; m(1,2)=-s; m(2,1)=s; m(2,2)=c;
        return m;
    }
    Eigen::Matrix4d Roty(double a)
    {
        Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
        const double c = std::cos(a);
        const double s = std::sin(a);
        m(0,0)=c; m(0,2)=s; m(2,0)=-s; m(2,2)=c;
        return m;
    }
    Eigen::Matrix4d Rotz(double a)
    {
        Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
        const double c = std::cos(a);
        const double s = std::sin(a);
        m(0,0)=c; m(0,1)=-s; m(1,0)=s; m(1,1)=c;
        return m;
    }
}

// Visual FK that exactly mirrors the QML Node hierarchy (scene units throughout).
//
// Chain (verified at q=0: EE=(817,119,204) == j6.scenePosition):
//   Trans(0,119,0)·Ry(q0) · Rz(q1) · Trans(425,0,0)·Rz(q2)
//   · Trans(392,0,0)·Rx(q3) · Trans(0,0,109)·Ry(q4) · Trans(0,0,95)·Rx(q5)
//
// Frame origins are read off mid-chain: a node's scenePosition is the
// translation part of (parent transform · local position) — its own rotation
// doesn't move its origin, so j1 and j2 coincide (j2's local position is zero).
std::vector<QVector3D> RobotController::visualFrames(const std::array<double, 6>& q)
{
    std::vector<QVector3D> frames;
    frames.reserve(7);

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    auto pushFrame = [&] {
        frames.push_back({ float(T(0,3)), float(T(1,3)), float(T(2,3)) });
    };

    pushFrame();                                     // base / world origin
    T = T * Trans(0,119,0);   pushFrame();           // j1
    T = T * Roty(q[0]) * Rotz(q[1]);
                              pushFrame();           // j2 (coincides with j1)
    T = T * Trans(425,0,0);   pushFrame();           // j3
    T = T * Rotz(q[2]) * Trans(392,0,0);
                              pushFrame();           // j4
    T = T * Rotx(q[3]) * Trans(0,0,109);
                              pushFrame();           // j5
    T = T * Roty(q[4]) * Trans(0,0, 95);
                              pushFrame();           // j6 = end-effector

    return frames;
}

QVector3D RobotController::visualEE(const std::array<double, 6>& q) const
{
    return visualFrames(q).back();
}

RobotController::RobotController(QObject *parent)
    : QObject(parent)
    // S3230: initialize members in init-list, not constructor body
    // S5025: unique_ptr instead of raw `new`
    , m_loop(std::make_unique<ControlLoop>())
    , m_thread(new QThread(this))
    , m_pathTimer(new QTimer(this))
{
    m_loop->moveToThread(m_thread);

    m_pathTimer->setSingleShot(true);
    connect(m_pathTimer, &QTimer::timeout, this, &RobotController::advancePath);

    // Cross-thread: main → loop (use .get() since connect() needs raw pointer)
    connect(this,         &RobotController::q_sendTarget,  m_loop.get(), &ControlLoop::setTarget,    Qt::QueuedConnection);
    connect(this,         &RobotController::q_seedCurrent, m_loop.get(), &ControlLoop::seedCurrent,  Qt::QueuedConnection);

    // Cross-thread: loop → main
    connect(m_loop.get(), &ControlLoop::stateUpdated, this, &RobotController::onLoopTick, Qt::QueuedConnection);

    connect(m_thread, &QThread::started, m_loop.get(), &ControlLoop::start);
    // No deleteLater connection: unique_ptr deletes m_loop after m_thread->wait() in destructor.

    m_thread->start();

    recomputeFK();
}

RobotController::~RobotController()
{
    m_thread->quit();
    m_thread->wait();
    // unique_ptr<ControlLoop> is destroyed here, after the thread has stopped — safe.
}

void RobotController::setAngle(int idx, qreal deg)
{
    if (m_ikEnabled || qFuzzyCompare(m_q[idx], deg)) {
        return;
    }
    m_q[idx] = deg;
    switch (idx) {
    case 0: emit q0Changed(); break;
    case 1: emit q1Changed(); break;
    case 2: emit q2Changed(); break;
    case 3: emit q3Changed(); break;
    case 4: emit q4Changed(); break;
    case 5: emit q5Changed(); break;
    default: break;  // S3562: switch must have a default case
    }
    recomputeFK();
}

void RobotController::recomputeFK()
{
    // S2681: braces on every loop body
    for (int i = 0; i < 6; ++i) {
        m_j[i] = m_q[i];
    }

    std::array<double, 6> q_rad;
    for (int i = 0; i < 6; ++i) {
        q_rad[i] = m_q[i] * kDeg2Rad;
    }
    m_eePos = visualEE(q_rad);

    emit jointsChanged();
    emit endEffectorPositionChanged();
}

void RobotController::setIkEnabled(bool v)
{
    if (m_ikEnabled == v) {
        return;
    }
    m_ikEnabled = v;

    if (v) {
        // Seed the loop at the current pose so the first trajectory starts correctly.
        QList<double> seed;
        for (int i = 0; i < 6; ++i) {
            seed.append(m_q[i]);
        }
        emit q_seedCurrent(seed);
    }

    emit ikEnabledChanged();
}

void RobotController::setTrajectoryDuration(qreal s)
{
    if (qFuzzyCompare(m_trajectoryDuration, s)) {
        return;
    }
    m_trajectoryDuration = s;
    emit trajectoryDurationChanged();
}

void RobotController::offsetTarget(QVector3D delta)
{
    m_target += delta;
    emit targetPositionChanged();
    if (m_ikEnabled) {
        solveIK();
    }
}

void RobotController::setTargetFromScene(QVector3D scenePos)
{
    m_target = scenePos;
    emit targetPositionChanged();
    if (m_ikEnabled) {
        solveIK();
    }
}

// Receives interpolated joint angles (degrees) from the 50 Hz control loop.
// Drives the visible pose both in IK mode and while replaying a planned path.
void RobotController::onLoopTick(QList<double> q_deg)
{
    if (!m_ikEnabled && !m_pathExecuting) {
        return;
    }
    for (int i = 0; i < 6; ++i) {
        m_q[i] = q_deg[i];
    }
    emit q0Changed(); emit q1Changed(); emit q2Changed();
    emit q3Changed(); emit q4Changed(); emit q5Changed();
    recomputeFK();
}

// Replays a planned trajectory by feeding each joint config to the
// ControlLoop in turn, `segmentDuration` seconds apart — same LERP machinery
// IK uses, just driven from a pre-computed list instead of a live solve.
void RobotController::followPath(QVariantList configsDeg, qreal segmentDuration)
{
    m_pathQueue.clear();
    for (const QVariant& cfg : configsDeg) {
        QList<double> angles;
        const QVariantList row = cfg.toList();
        for (const QVariant& a : row) {
            angles.append(a.toDouble());
        }
        if (angles.size() == 6) {
            m_pathQueue.append(angles);
        }
    }
    if (m_pathQueue.isEmpty()) {
        return;
    }

    m_pathSegmentDuration = std::max(segmentDuration, 0.05);
    m_pathIndex = 0;

    if (!m_pathExecuting) {
        m_pathExecuting = true;
        emit pathExecutingChanged();
    }

    // Snap the loop to wherever the arm is now so the first leg starts cleanly.
    QList<double> seed;
    for (int i = 0; i < 6; ++i) {
        seed.append(m_q[i]);
    }
    emit q_seedCurrent(seed);

    advancePath();
}

void RobotController::advancePath()
{
    if (m_pathIndex >= m_pathQueue.size()) {
        m_pathExecuting = false;
        emit pathExecutingChanged();
        return;
    }

    emit q_sendTarget(m_pathQueue[m_pathIndex], double(m_pathSegmentDuration));
    ++m_pathIndex;
    m_pathTimer->start(int(m_pathSegmentDuration * 1000.0));
}

// DLS inverse kinematics — position-only, numerical Jacobian.
//
// Δq = Jᵀ · (J·Jᵀ + λ²·I)⁻¹ · err
//
// After convergence (or max iterations), the solution is sent to the
// ControlLoop which interpolates joints smoothly over trajectoryDuration seconds.
void RobotController::solveIK()
{
    std::array<double, 6> q;
    for (int i = 0; i < 6; ++i) {
        q[i] = m_q[i] * kDeg2Rad;
    }

    const Eigen::Vector3d tgt(m_target.x(), m_target.y(), m_target.z());

    bool converged = false;
    for (int it = 0; it < kMaxIter; ++it) {
        const QVector3D ee = visualEE(q);
        const Eigen::Vector3d err(tgt.x()-ee.x(), tgt.y()-ee.y(), tgt.z()-ee.z());
        if (err.norm() < kTol) { converged = true; break; }

        // Numerical Jacobian: column i = (EE(q + ε·eᵢ) − EE(q)) / ε
        Eigen::Matrix<double, 3, 6> J;
        for (int i = 0; i < 6; ++i) {
            q[i] += kEps;
            const QVector3D ep = visualEE(q);
            J.col(i) = Eigen::Vector3d(ep.x()-ee.x(), ep.y()-ee.y(), ep.z()-ee.z()) / kEps;
            q[i] -= kEps;
        }

        const auto A  = J * J.transpose() + kLambda2 * Eigen::Matrix3d::Identity();
        const auto dq = J.transpose() * A.ldlt().solve(err);
        for (int i = 0; i < 6; ++i) {
            q[i] = std::clamp(q[i] + dq[i], -std::numbers::pi, std::numbers::pi);
        }
    }

    // Hand the solution to the control loop — joints will travel there smoothly.
    QList<double> target_deg;
    for (int i = 0; i < 6; ++i) {
        target_deg.append(q[i] * kRad2Deg);
    }
    emit q_sendTarget(target_deg, double(m_trajectoryDuration));

    if (m_ikConverged != converged) {
        m_ikConverged = converged;
        emit ikConvergedChanged();
    }
}
