#ifndef _PROFILE_H_
#define _PROFILE_H_

#include <vector>
#include <functional>

#include "main.h"
#include "interface.h"
#include "math.h"
#include "spline.h"


struct MPPoint {
    double x;
    double y;
    double heading;
    double linVel;
    double angVel;
    double t;
    double timeAtPoint;
};

struct HoloMPPoint {
    double x;
    double y;
    double heading;
    Vector linVel;
    double angVel;
    double t;
    double timeAtPoint;
};

struct Zone {
    double startT; // start value of t
    double endT; // end value of t
    Line zoneLine; // line that represents the zone on the graph
};

enum Direction {
    LEFT = -1,
    STRAIGHT = 0,
    RIGHT = 1,
};

class MotionProfile {
    public:
        // constructors (one with custom zoning, one without)
        MotionProfile(CubicHermiteSpline* path, double maxSpeed, std::vector<std::vector<Point>> zonePoints = {});
        MotionProfile(CubicHermiteSpline* path, double maxSpeed, std::vector<double> headings, std::vector<double> headingTs, std::vector<std::vector<Point>> zonePoints = {});
        MotionProfile(std::vector<MPPoint>* pregeneratedProfile, double maxSpeed);

        // instance variables (data about profile)
        std::vector<MPPoint> profile;
        std::vector<HoloMPPoint> holoProfile;
        std::vector<Zone> zones;
        double maxSpeed;

        // public methods (operations on points)
        MPPoint findNearestPoint(double givenT);
        HoloMPPoint findNearestHoloPoint(double givenT);

    private:
        // private methods (profile generation)
        void generateVelocities(void);
        void constructWithCustomZones(std::vector<std::vector<Point>> zoneLinePoints);
        double customHeading(double t);

        // private instance variables (used in profile generation)
        CubicHermiteSpline* path;
        std::vector<double> headings;
        std::vector<double> headingTs;
        bool isHolo;
};



#endif