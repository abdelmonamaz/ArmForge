#pragma once
#include <QList>
#include <QObject>
#include <QTimer>

// Runs at 50 Hz on its own thread.
// Linearly interpolates 6 joint angles from the current configuration toward
// the last requested target and emits stateUpdated() on every tick during motion.
class ControlLoop : public QObject
{
    Q_OBJECT

    static constexpr double kHz = 50.0;
    static constexpr double kDt = 1.0 / kHz;

public:
    explicit ControlLoop(QObject* parent = nullptr);

public slots:
    void start();
    void stop();

    // Feed a new IK solution (degrees). Restart LERP from the current arm position.
    void setTarget(QList<double> q_deg, double duration_s);

    // Snap internal state to q_deg without launching a trajectory.
    // Call once when IK is first enabled so the LERP starts at the correct pose.
    void seedCurrent(QList<double> q_deg);

signals:
    void stateUpdated(QList<double> q_deg);  // emitted every tick while moving

private slots:
    void tick();

private:
    QTimer*       m_timer;
    // S3230: in-class initializers instead of constructor init-list
    QList<double> m_current = QList<double>(6, 0.0);   // degrees — what the arm shows now
    QList<double> m_start   = QList<double>(6, 0.0);   // snapshot at trajectory start
    QList<double> m_target  = QList<double>(6, 0.0);   // goal angles in degrees
    double        m_duration = 1.0;
    double        m_elapsed  = 0.0;
    bool          m_moving   = false;
};
