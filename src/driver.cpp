#include "driver.h"

ColorController::ColorController(TrackingSensor colorDetector, PowerUnit outputDevice, TrackingSensor outputTracker, TrackingSensor buttonDetector) {
    m_color = colorDetector;
    m_output = outputDevice;
    m_button = buttonDetector;
    m_outTracker = outputTracker;
}

void ColorController::activate(double predelay, double postdelay, bool reverse, double distance) {
    
    bool ejectOn = false;
    double ejectStartPoint = 0;

    if(m_button.get()) {
        if ((ejectOn == true) && (reverse)) {
            // case 1a: if the difference between the starting point and the current point
            // 			is greater than 700 (meaning that it has gone all the way),
            //			turn off the 
            if (abs(m_outTracker.get() - ejectStartPoint) >= 1000) {
                ejectOn = false;
                ejectStartPoint = 0;
                m_output.stop();
            // case 1b: if case 1a is not true, then continue moving the intake down
            } else {
                m_output.move(-128);
            }
        }
        else if (ejectOn == true) {
            pros::delay(postdelay);
            ejectOn = false;
            ejectStartPoint = 0;
        }
        //case 2: eject is not on, but the distance sensor is at the proper distance and the color sensor has found a correct color
        else if (m_color.get())
        {
            pros::delay(predelay);
            m_output.stop();
            if (reverse) {
                m_output.move(-128);
            }
            ejectOn = true;
            ejectStartPoint = m_outTracker.get();
        }
        // case 3: if the redirect is not on and should not be on, 
        //		   then L2 moves the robot forward as normal
        else {
            m_output.move(128);
        }
    } 
    else {
        m_output.stop();
        ejectOn = false;
    }
    pros::delay(10);
}