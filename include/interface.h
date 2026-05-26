
#ifndef _TRACKING_H_
#define _TRACKING_H_

#include <functional>

#include "main.h"

// declaration
class TrackingSensor {
    public:
        TrackingSensor(std::function<double(void)> getValue, std::function<void(double val)> setValue = [](double val){}, std::function<void(void)> reset = [](){});
        TrackingSensor() = default;
        double get(void);
        void set(double val);
        void reset(void);

    private:
        std::function<double(void)> m_getValue;
        std::function<void(double)> m_setValue;
        std::function<void(void)> m_reset;

};

class PowerUnit {
    public:
        PowerUnit(std::function<void(double val)> move, std::function<void(void)> stop);
        PowerUnit() = default;

        void move(double val);
        void stop(void);

    private:
        std::function<void(double)> m_move;
        std::function<void(void)> m_stop;

};

#endif