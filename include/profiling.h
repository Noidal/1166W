#ifndef _PROFILINGH_
#define _PROFILINGH_

#include <vector>
#include <functional>

#include "main.h"
#include "interface.h"
#include "odom.h"
#include "pid.h"
#include "math.h"

struct UltraPose {
    double x;
    double y;
    double heading;
    double curvature;
};

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

struct PIDSet {
    PIDController* x;
    PIDController* y;
    PIDController* thetaL90;
    PIDController* thetaG90;
};

class CubicHermiteSpline {
    public:
        CubicHermiteSpline(Point startPos, Point startV, Point endPos, Point endV);
        Point findPoint(double t);
        Pose findPose(double t, double step);
        UltraPose findUltraPose(double t, double step);
        std::vector<UltraPose> entirePath(double numPoints);
        void findFunction(void);
        void findDerivative(void);
        void findSecondDerivative(void);
        double calculateCurvature(double t);
        double findNearestPointOnSpline(Point givenPoint, double excludeBelow);
        double calculateCurveSpeed(double t);
        double findNextT(double currentT, double distanceToMove);

        std::vector<CubicPolyData> functions;
        std::vector<QuadraticPolyData> derivative;
        std::vector<HexicPolyData> secondDerivative;
        std::vector<UltraPose> fullSampleSpline;


    private:
        double evaluateP0(double t);
        double evaluateV0(double t);
        double evaluateP1(double t);
        double evaluateV1(double t);

        Point startPos;
        Point startV;
        Point endPos;
        Point endV;
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
        double calculateSingleDegree(double wheelDiameter);
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