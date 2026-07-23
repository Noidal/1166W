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
        VelocityController(PowerUnit* lOutput, PowerUnit* rOutput, PoseTracker* globalPos, PhysicalConstraints robot);
        void addAction(std::function<void(void)> action, double time);
        void clearActions(void);
        void startProfile(MotionProfile* profile, bool correct = true);
        void followProfile(MotionProfile* profile, bool reverse = false, bool RAMSETE = true);


    private:
        std::vector<double> calculateOutputOfSides(double linearVelocityIPS, double angularVelocityRADPS);
        
        double timeToRun;
        PowerUnit* lOutput;
        PowerUnit* rOutput;

        PIDSet corrector;
        PoseTracker* globalPos;
        bool willCorrect;

        PhysicalConstraints robot;

        std::vector<double> actionTs;
        std::vector<std::function<void(void)>> actions;
        std::vector<bool> actionCompleteds;
};



#endif