#ifndef _FOLLOW_H_
#define _FOLLOW_H_

#include <vector>
#include <functional>

#include "main.h"
#include "interface.h"
#include "odom.h"
#include "pid.h"
#include "math.h"
#include "profile.h"

struct PIDSet {
    PIDController* x;
    PIDController* y;
    PIDController* thetaL90;
    PIDController* thetaG90;
};

class VelocityController {
    public:
        double linVel;
        double angVel;
        VelocityController(PowerUnit* xOutput, PowerUnit* yOutput, PowerUnit* thetaOutput, PoseTracker* globalPos, PIDSet corrector = {});
        void addAction(std::function<void(void)> action, double time);
        void clearActions(void);
        void startProfile(MotionProfile* profile, bool correct = true);


    private:
        std::vector<double> calculateOutputOfSides(Vector linearVelocityIPS, double angularVelocityRADPS, double profileMaxIPS);
        void followProfile(MotionProfile* profile, bool correct = true);
        
        double timeToRun;
        PowerUnit* xOutput;
        PowerUnit* yOutput;
        PowerUnit* thetaOutput;

        PIDSet corrector;
        PoseTracker* globalPos;
        bool willCorrect;

        std::vector<double> actionTs;
        std::vector<std::function<void(void)>> actions;
        std::vector<bool> actionCompleteds;
};



#endif