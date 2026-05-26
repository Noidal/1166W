#ifndef _DRIVER_H_
#define _DRIVER_H_

#include "main.h"
#include "interface.h"

class ColorController {
    public:
        ColorController(TrackingSensor colorDetector, PowerUnit outputDevice, TrackingSensor outputTracker, TrackingSensor buttonDetector);
        void activate(double predelay, double postdelay, bool reverse = false, double distance = 0);

    private:
        pros::Mutex m_lock;
        TrackingSensor m_color;
        PowerUnit m_output;
        TrackingSensor m_outTracker;
        TrackingSensor m_button;

};

#endif