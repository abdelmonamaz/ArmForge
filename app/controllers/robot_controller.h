#pragma once
#include <QObject>
#include <QQmlEngine>
#include <QThread>
#include <QTimer>
#include <QVariantList>
#include <QVector3D>
#include <array>
#include <memory>
#include <vector>

class ControlLoop;

// Exposes the 6 joint angles to QML.
// FK mode:  sliders → setAngle() → recomputeFK() → arm moves instantly.
// IK mode:  gizmo drag → solveIK() → ControlLoop lerps joints at 50 Hz.
class RobotController : public QObject // NOSONAR (S1448: signal count inflates method total)
{
    Q_OBJECT
    QML_ELEMENT
    QML_SINGLETON

    // ── FK-mode joint sliders ────────────────────────────────────
    Q_PROPERTY(qreal q0 READ q0 WRITE setQ0 NOTIFY q0Changed)
    Q_PROPERTY(qreal q1 READ q1 WRITE setQ1 NOTIFY q1Changed)
    Q_PROPERTY(qreal q2 READ q2 WRITE setQ2 NOTIFY q2Changed)
    Q_PROPERTY(qreal q3 READ q3 WRITE setQ3 NOTIFY q3Changed)
    Q_PROPERTY(qreal q4 READ q4 WRITE setQ4 NOTIFY q4Changed)
    Q_PROPERTY(qreal q5 READ q5 WRITE setQ5 NOTIFY q5Changed)

    // Per-joint Euler angles consumed by RobotView.qml.
    Q_PROPERTY(qreal j1Yaw  READ j1Yaw  NOTIFY jointsChanged)
    Q_PROPERTY(qreal j2Roll READ j2Roll NOTIFY jointsChanged)
    Q_PROPERTY(qreal j3Roll READ j3Roll NOTIFY jointsChanged)
    Q_PROPERTY(qreal j4Roll READ j4Roll NOTIFY jointsChanged)
    Q_PROPERTY(qreal j5Roll READ j5Roll NOTIFY jointsChanged)
    Q_PROPERTY(qreal j6Roll READ j6Roll NOTIFY jointsChanged)

    // ── IK mode ──────────────────────────────────────────────────
    Q_PROPERTY(bool      ikEnabled            READ ikEnabled            WRITE setIkEnabled NOTIFY ikEnabledChanged)
    Q_PROPERTY(bool      ikConverged          READ ikConverged          NOTIFY ikConvergedChanged)
    Q_PROPERTY(QVector3D targetPosition       READ targetPosition       NOTIFY targetPositionChanged)
    Q_PROPERTY(QVector3D endEffectorPosition  READ endEffectorPosition  NOTIFY endEffectorPositionChanged)

    // ── Control loop ─────────────────────────────────────────────
    // Trajectory duration in seconds. The arm takes this long to reach the
    // IK target after each gizmo move. Range [0.0, 3.0].
    Q_PROPERTY(qreal trajectoryDuration READ trajectoryDuration WRITE setTrajectoryDuration NOTIFY trajectoryDurationChanged)

    // ── Path execution (Phase 4 — planning) ─────────────────────
    // True while followPath() is replaying a planned trajectory.
    Q_PROPERTY(bool pathExecuting READ pathExecuting NOTIFY pathExecutingChanged)

public:
    explicit RobotController(QObject *parent = nullptr);
    ~RobotController() override;

    // Pure function: joint config (radians) → world-frame positions of every
    // frame along the visual FK chain (base, j1, j2, j3, j4, j5, j6 = EE),
    // in scene units. Single source of truth shared with the rendered scene —
    // PlanningController uses it for collision sampling and the EE path trace.
    static std::vector<QVector3D> visualFrames(const std::array<double, 6>& q_rad);

    qreal q0() const { return m_q[0]; }
    qreal q1() const { return m_q[1]; }
    qreal q2() const { return m_q[2]; }
    qreal q3() const { return m_q[3]; }
    qreal q4() const { return m_q[4]; }
    qreal q5() const { return m_q[5]; }

    void setQ0(qreal v) { setAngle(0, v); }
    void setQ1(qreal v) { setAngle(1, v); }
    void setQ2(qreal v) { setAngle(2, v); }
    void setQ3(qreal v) { setAngle(3, v); }
    void setQ4(qreal v) { setAngle(4, v); }
    void setQ5(qreal v) { setAngle(5, v); }

    qreal j1Yaw()  const { return m_j[0]; }
    qreal j2Roll() const { return m_j[1]; }
    qreal j3Roll() const { return m_j[2]; }
    qreal j4Roll() const { return m_j[3]; }
    qreal j5Roll() const { return m_j[4]; }
    qreal j6Roll() const { return m_j[5]; }

    bool      ikEnabled()           const { return m_ikEnabled; }
    bool      ikConverged()         const { return m_ikConverged; }
    QVector3D targetPosition()      const { return m_target; }
    QVector3D endEffectorPosition() const { return m_eePos; }

    qreal trajectoryDuration()      const { return m_trajectoryDuration; }
    void  setTrajectoryDuration(qreal s);

    void setIkEnabled(bool v);

    bool pathExecuting() const { return m_pathExecuting; }

    Q_INVOKABLE void offsetTarget(QVector3D delta);
    Q_INVOKABLE void setTargetFromScene(QVector3D scenePos);

    // Replays a planned trajectory: a list of 6-angle joint configs (degrees),
    // one ControlLoop LERP segment per step, `segmentDuration` seconds each.
    Q_INVOKABLE void followPath(QVariantList configsDeg, qreal segmentDuration);

signals:
    void q0Changed();
    void q1Changed();
    void q2Changed();
    void q3Changed();
    void q4Changed();
    void q5Changed();
    void jointsChanged();
    void ikEnabledChanged();
    void ikConvergedChanged();
    void targetPositionChanged();
    void endEffectorPositionChanged();
    void trajectoryDurationChanged();
    void pathExecutingChanged();

    // ── Internal: cross-thread invocations on the control loop ───
    void q_sendTarget(QList<double> q_deg, double duration_s);
    void q_seedCurrent(QList<double> q_deg);

private slots:
    // Receives interpolated joint angles from the 50 Hz control loop.
    void onLoopTick(QList<double> q_deg);

    // Sends the next queued config to the ControlLoop, or stops when done.
    void advancePath();

private:
    void      setAngle(int idx, qreal deg);
    void      recomputeFK();
    void      solveIK();

    QVector3D visualEE(const std::array<double, 6>& q_rad) const;

    std::array<qreal, 6> m_q{};   // joint angles in degrees (UI layer)
    std::array<qreal, 6> m_j{};   // per-joint display angles in degrees

    bool      m_ikEnabled         = false;
    bool      m_ikConverged       = false;
    QVector3D m_target;
    QVector3D m_eePos;

    qreal     m_trajectoryDuration = 0.5;

    std::unique_ptr<ControlLoop> m_loop;
    QThread*                     m_thread = nullptr;

    // ── Path execution (Phase 4) ─────────────────────────────────
    QList<QList<double>> m_pathQueue;
    int                  m_pathIndex            = 0;
    qreal                m_pathSegmentDuration  = 0.5;
    bool                 m_pathExecuting        = false;
    QTimer*              m_pathTimer            = nullptr;
};
