#include "interface.h"

// Tracking Sensor Interface

TrackingSensor::TrackingSensor(std::function<double(void)> getValue, std::function<void(double val)> setValue, std::function<void(void)> reset) {
    m_getValue = getValue;
    m_setValue = setValue;
    m_reset = reset;
}

double TrackingSensor::get() {
    return m_getValue();
}

void TrackingSensor::set(double val) {
    return m_setValue(val);
}

void TrackingSensor::reset(void) {
    return m_reset();
}

// Power Unit Interface

PowerUnit::PowerUnit(std::function<void(double val)> move, std::function<void(void)> stop) {
    m_move = move;
    m_stop = stop;
}

void PowerUnit::move(double val) {
    return m_move(val);
}

void PowerUnit::stop(void) {
    return m_stop();
}