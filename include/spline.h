#ifndef _SPLINE_H_
#define _SPLINE_H_

#include <vector>
#include <functional>

#include "main.h"
#include "math.h"

struct PoseC {
    double x;
    double y;
    double heading;
    double curvature;
};

class CubicHermiteSpline {
    public:
        CubicHermiteSpline(Point startPos, Point startV, Point endPos, Point endV);
        Point findPoint(double t);
        Pose findPose(double t);
        std::vector<PoseC> entirePath(double numPoints);
        void createFunction(void);
        void findDerivative(void);
        void findSecondDerivative(void);
        double calculateCurvature(double t);
        double findNearestPointOnSpline(Point givenPoint, double excludeBelow);
        double calculateCurveSpeed(double t);
        double findNextT(double currentT, double distanceToMove);
        double advanceLength(double currentT, double distance);

        std::vector<CubicPolyData> functions;
        std::vector<QuadraticPolyData> derivative;
        std::vector<HexicPolyData> secondDerivative;
        std::vector<PoseC> fullSampleSpline;


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


#endif