#ifndef _METH_H_
#define _METH_H_

#include <deque>
#include <cmath>

#include "main.h"

struct Point {
    double x;
    double y;
};

struct Pose {
    double x;
    double y;
    double heading;
};

struct Line {
    double slope;
    double yIntercept;
};

struct Inequality {
    double slope;
    double yIntercept;
    int equality;
};

struct Vector {
    double x;
    double y;
    double magnitude;
    double angle;
};

struct HexicPolyData {
    double P6;
    double P5;
    double P4;
    double P3;
    double P2;
    double P1;
    double P0;
};

struct CubicPolyData {
    double a;
    double b;
    double c;
    double d;
};

struct QuadraticPolyData {
    double a;
    double b;
    double c;
};

double calculateDistance(Point point1, Point point2);
double calculateStandardDeviation(std::deque<double> listOfDifferences);
Line calculatePerpendicularNonInequality(Point point1, Point point2);
Inequality calculatePerpendicularInequality(Point point1, Point point2);
int findEquality(Inequality line, Point includedPoint);
Point findIntersection(Line line1, Line line2);
Line findLineWithHeading(Point point1, int heading);
double findHeadingOfLine(Point point1, Point point2);
Line findLineWithPoints(Point point1, Point point2);
QuadraticPolyData derivativeOfCubicPoly(CubicPolyData cubicPoly);
Line derivativeOfQuadratic(QuadraticPolyData quadPoly);
double fixAngle(double originalAngle);
double unfixAngle(double originalAngle);
double RPMtoIPS(double rpm);
double IPStoRPM(double ips);
double bind180(double heading);
double makeRelative(double currentHeading, double goalHeading);



#endif