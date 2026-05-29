#include "control_loop.h"
#include <algorithm>

ControlLoop::ControlLoop(QObject* parent)
    : QObject(parent)
    , m_timer(new QTimer(this))
{
    m_timer->setInterval(int(1000.0 / kHz));   // 20 ms
    connect(m_timer, &QTimer::timeout, this, &ControlLoop::tick);
}

void ControlLoop::start() { m_timer->start(); }
void ControlLoop::stop()  { m_timer->stop();  }

void ControlLoop::seedCurrent(QList<double> q_deg)
{
    m_current  = q_deg;
    m_start    = q_deg;
    m_target   = q_deg;
    m_moving   = false;
    m_elapsed  = 0.0;
}

void ControlLoop::setTarget(QList<double> q_deg, double duration_s)
{
    m_start    = m_current;
    m_target   = q_deg;
    m_duration = std::max(duration_s, kDt);  // at least one tick
    m_elapsed  = 0.0;
    m_moving   = true;
}

void ControlLoop::tick()
{
    if (!m_moving) return;

    m_elapsed += kDt;
    const double alpha = std::min(m_elapsed / m_duration, 1.0);

    // Linear interpolation: q(t) = q_start + α·(q_target − q_start)
    for (int i = 0; i < 6; ++i)
        m_current[i] = m_start[i] + alpha * (m_target[i] - m_start[i]);

    emit stateUpdated(m_current);

    if (alpha >= 1.0) m_moving = false;
}
