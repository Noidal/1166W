#ifndef _SUBSYSTEMS_H_
#define _SUBSYSTEMS_H_

#include <vector>

#include "main.h"
#include "interface.h"

class CascadeLift {
    public:
        CascadeLift(PowerUnit output, TrackingSensor speedTracker, TrackingSensor positionTracker, 
                    double incrementUp, double incrementDown,
                    double maxHeight, double minHeight);

        void moveTo(int targetPosition);
        void targetSpeed(int targetSpeed);

        bool autoRun = false;
    
    private:
        void move();
            
        PowerUnit output;
        TrackingSensor speedTracker;
        TrackingSensor positionTracker;

        double incrementUp = 0;
        double incrementDown = 0;

        double maxHeight = 0;
        double minHeight = 0;

        double speed = 0;

        double targetPosition = 0;


        pros::Task* cascadeTask;
};

class Claw {
    public:
        Claw(PowerUnit output, TrackingSensor posTracker);

        void updateState(void);
        void move(void);
        void setState(int newState);

        int state;

    private:
        PowerUnit output;
        TrackingSensor posTracker;
};

#endif