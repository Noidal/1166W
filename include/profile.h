#ifndef _PROFILE_H_
#define _PROFILE_H_

#include <vector>
#include <functional>

#include "main.h"
#include "math.h"
#include "spline.h"


struct MPPoint {
    double x = 0;
    double y = 0;
    double heading = 0;
    double linVel = 0;
    double angVel = 0;
    double t = 0;
    double timeAtPoint = 0;
};

struct CustomSpeed {
    double t = 0;
    double speed = 1;
};

class PhysicalConstraints {
    public:
        PhysicalConstraints(double rpm, double gearRatio, double wheelDiameter, double distBetweenDTSides, double mass, double dtEfficiency, double fricCoef);
        PhysicalConstraints() = default;
        double maxVelocityAtCurvature(double curvature);
        double maxLinAccelAtAngAccel(double angAccel);
        std::vector<double> outputOfSides(double linearVelocityIPS, double angularVelocityRADPS);
        
        double wheelDiameter;
        double gearRatio;
        double maxAccel;
        double maxDecel;
        double maxSpeed;
        double dtDiff;
        double maxRPM;
        double maxTorque;
        double maxAngAccel;
};

class MotionProfile {
    public:
        // constructors (one with custom zoning, one without)
        MotionProfile(CubicHermiteSpline* path, PhysicalConstraints robot, double distSeg);
        MotionProfile(std::vector<MPPoint>* pregeneratedProfile, PhysicalConstraints robot, double distSeg);

        // instance variables (data about profile)
        std::vector<MPPoint> profile;
        PhysicalConstraints robot;
        double distSeg;

        // public methods (operations on points)
        MPPoint findNearestPoint(double givenT);

    private:
        // private methods (profile generation)
        void generateVelocities(void);
        void constructWithCustomZones(std::vector<std::vector<Point>> zoneLinePoints);
        double customHeading(double t);

        // private instance variables (used in profile generation)
        CubicHermiteSpline* path;
        std::vector<double> headings;
        std::vector<double> headingTs;
};



#endif