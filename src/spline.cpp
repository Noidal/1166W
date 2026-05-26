#include "profiling.h"

CubicHermiteSpline::CubicHermiteSpline(Point startPos, Point startV, Point endPos, Point endV) {
    this->startPos = startPos;
    this->startV = {startV.x - startPos.x, startV.y - startPos.y};
    this->endPos = endPos;
    this->endV = {endV.x - endPos.x, endV.y - endPos.y};

    this->findFunction();
    this->findDerivative();
    this->findSecondDerivative();
}

double CubicHermiteSpline::evaluateP0(double t) {
    return (2 * (std::pow(t, 3))) - (3 * (std::pow(t, 2))) + 1;
}

double CubicHermiteSpline::evaluateV0(double t) {
    return (std::pow(t, 3)) - (2 * (std::pow(t, 2))) + t;
}

double CubicHermiteSpline::evaluateP1(double t) {
    return (-2 * (std::pow(t, 3))) + (3 * (std::pow(t, 2)));
}

double CubicHermiteSpline::evaluateV1(double t) {
    return (std::pow(t, 3)) - (std::pow(t, 2));
}






Point CubicHermiteSpline::findPoint(double t) {
    Point result;
    result.x = (evaluateP0(t) * startPos.x) + (evaluateV0(t) * startV.x) + (evaluateP1(t) * endPos.x) + (evaluateV1(t) * endV.x);
    result.y = (evaluateP0(t) * startPos.y) + (evaluateV0(t) * startV.y) + (evaluateP1(t) * endPos.y) + (evaluateV1(t) * endV.y);
    return result;
}

Pose CubicHermiteSpline::findPose(double t, double step) {
    Point current = this->findPoint(t);
    Pose currentP = {current.x, current.y};

    double prevT = t - step;
    Point prev = this->findPoint(prevT);

    currentP.heading = findHeadingOfLine(prev, current);

    return currentP;
}

UltraPose CubicHermiteSpline::findUltraPose(double t, double step) {
    Pose currentP = this->findPose(t, step);
    UltraPose currentUltraP = {currentP.x, currentP.y, currentP.heading, this->calculateCurvature(t)};

    return currentUltraP;
}

std::vector<UltraPose> CubicHermiteSpline::entirePath(double numPoints) {
    UltraPose currentPose;
    std::vector<UltraPose> fullPath;
    
    double step = 1 / numPoints;

    for (double t = 0; t < 1; t += step) {
        currentPose = {this->findPose(t, step).x, this->findPose(t, step).y, this->findPose(t, step).heading, this->calculateCurvature(t)};
        fullPath.push_back(currentPose);
    }

    return fullPath;
}

void CubicHermiteSpline::findFunction(void) {

    // x function
        double coef1 = 2 * startPos.x; // t^3 term
        double coef2 = -3 * startPos.x; // t^2 term
        double coef3 = startPos.x; // t^0 term
        double coef4 = startV.x; // t^3 term
        double coef5 = -2 * startV.x; // t^2 term
        double coef6 = startV.x; // t^1 term
        double coef7 = -2 * endPos.x; // t^3 term
        double coef8 = 3 * endPos.x; // t^2 term
        double coef9 = endV.x; // t^3 term
        double coef10 = -1 * endV.x; // t^2 term;

        CubicPolyData xFuncInfo = {
            coef1 + coef4 + coef7 + coef9, // t^3
            coef2 + coef5 + coef8 + coef10, // t^2
            coef6, // t^1
            coef3 // t^0
        };

    // y function
        coef1 = 2 * startPos.y; // t^3 term
        coef2 = -3 * startPos.y; // t^2 term
        coef3 = startPos.y; // t^0 term
        coef4 = startV.y; // t^3 term
        coef5 = -2 * startV.y; // t^2 term
        coef6 = startV.y; // t^1 term
        coef7 = -2 * endPos.y; // t^3 term
        coef8 = 3 * endPos.y; // t^2 term
        coef9 = endV.y; // t^3 term
        coef10 = -1 * endV.y; // t^2 term;

        CubicPolyData yFuncInfo = {
            coef1 + coef4 + coef7 + coef9, // t^3
            coef2 + coef5 + coef8 + coef10, // t^2
            coef6, // t^1
            coef3 // t^0
        };

    functions = {xFuncInfo, yFuncInfo};
}

void CubicHermiteSpline::findDerivative(void) {
    std::vector<CubicPolyData> functions = this->functions;
    std::vector<QuadraticPolyData> derivatives = {derivativeOfCubicPoly(functions[0]), derivativeOfCubicPoly(functions[1])};

    QuadraticPolyData numerator = derivatives[1];
    QuadraticPolyData denominator = derivatives[0];

    this->derivative = {numerator, denominator};
}

void CubicHermiteSpline::findSecondDerivative(void) {
    QuadraticPolyData derivativeX = this->derivative[1];
    QuadraticPolyData derivativeY = this->derivative[0];

    Line secondDerivativeX = derivativeOfQuadratic(derivativeX);
    Line secondDerivativeY = derivativeOfQuadratic(derivativeY);

    double paraDeriv2NumCoefT3 = (derivativeX.a * secondDerivativeY.slope) - (derivativeY.a * secondDerivativeX.slope);
    double paraDeriv2NumCoefT2 = (((derivativeX.b * secondDerivativeY.slope) + (derivativeX.a * secondDerivativeY.yIntercept)) - ((derivativeY.b * secondDerivativeX.slope) + (derivativeY.a * secondDerivativeX.yIntercept)));
    double paraDeriv2NumCoefT1 = (((derivativeX.c * secondDerivativeY.slope) + (derivativeX.b * secondDerivativeY.yIntercept)) - ((derivativeY.c * secondDerivativeX.slope) + (derivativeY.b * secondDerivativeX.yIntercept)));
    double paraDeriv2NumCoefT0 = ((derivativeX.c * secondDerivativeY.yIntercept) - (derivativeY.c * secondDerivativeX.yIntercept));
    // stored in hexic so it can be returned in a vector with the actually hexic denominator
    HexicPolyData numerator = {0, 0, 0, paraDeriv2NumCoefT3, paraDeriv2NumCoefT2, paraDeriv2NumCoefT1, paraDeriv2NumCoefT0};
    
    double paraDeriv2DenCoefT6 = std::pow(derivativeX.a, 3);
    double paraDeriv2DenCoefT5 = (3 * (std::pow(derivativeX.a, 2) * derivativeX.b));
    double paraDeriv2DenCoefT4 = (3 * (std::pow(derivativeX.b, 2) * derivativeX.a)) + (3 * (std::pow(derivativeX.a, 2) * derivativeX.c));
    double paraDeriv2DenCoefT3 = (6 * (derivativeX.a * derivativeX.b * derivativeX.c)) + (std::pow(derivativeX.b, 3));
    double paraDeriv2DenCoefT2 = (3 * (std::pow(derivativeX.c, 2) * derivativeX.a)) + (3 * (std::pow(derivativeX.b, 2) * derivativeX.c));
    double paraDeriv2DenCoefT1 = (3 * (std::pow(derivativeX.c, 2) * derivativeX.b));
    double paraDeriv2DenCoefT0 = std::pow(derivativeX.c, 3);
    /*
    double paraDeriv2DenCoefT2 = derivativeX.a;
    double paraDeriv2DenCoefT1 = derivativeX.b;
    double paraDeriv2DenCoefT0 = derivativeX.c;
    */

    HexicPolyData denominator = {paraDeriv2DenCoefT6, paraDeriv2DenCoefT5, paraDeriv2DenCoefT4, paraDeriv2DenCoefT3, paraDeriv2DenCoefT2, paraDeriv2DenCoefT1, paraDeriv2DenCoefT0};

    

    this->secondDerivative = {numerator, denominator};

    HexicPolyData secondDerivNum = {0, 0, 0, (-1 * this->secondDerivative[0].P3), (-1 * this->secondDerivative[0].P2), (-1 * this->secondDerivative[0].P1), (-1 * this->secondDerivative[0].P0)};
    HexicPolyData secondDerivDen = {(-1 * this->secondDerivative[1].P6), (-1 * this->secondDerivative[1].P5), (-1 * this->secondDerivative[1].P4), (-1 * this->secondDerivative[1].P3), (-1 * this->secondDerivative[1].P2), (-1 * this->secondDerivative[1].P1), (-1 * this->secondDerivative[1].P0)};

    this->secondDerivative = {secondDerivNum, secondDerivDen};
}

/*
double CubicHermiteSpline::calculateCurvature(double t) {

    double secondDerivativeNum = (this->secondDerivative[0].P3 * std::pow(t, 3)) + (this->secondDerivative[0].P2 * std::pow(t, 2)) + (this->secondDerivative[0].P1 * t) + this->secondDerivative[0].P0;

    double secondDerivativeDen = (this->secondDerivative[1].P6 * std::pow(t, 6)) + (this->secondDerivative[1].P5 * std::pow(t, 5)) + 
                                 (this->secondDerivative[1].P4 * std::pow(t, 4)) + (this->secondDerivative[1].P3 * std::pow(t, 3)) + 
                                 (this->secondDerivative[1].P2 * t) + (this->secondDerivative[1].P1 * t) + 
                                  this->secondDerivative[1].P0;

    double derivativeNum = (this->derivative[0].a * std::pow(t, 2)) + (this->derivative[0].b * t) + this->derivative[0].c;
    double derivativeDen = (this->derivative[1].a * std::pow(t, 2)) + (this->derivative[1].b * t) + this->derivative[1].c;
    
    // double curvature = std::abs((secondDerivativeNum / secondDerivativeDen)) / (std::pow((1 + std::pow((derivativeNum / derivativeDen), 2)), (3 / 2)));
    double curvature = std::abs(((derivativeNum / derivativeDen) * (secondDerivativeNum / secondDerivativeDen))) / std::pow((derivativeNum / derivativeDen), 3);

    return curvature;
} */

double CubicHermiteSpline::calculateCurvature(double t) {

    double derivativeY = (this->derivative[0].a * std::pow(t, 2)) + (this->derivative[0].b * t) + this->derivative[0].c;
    double derivativeX = (this->derivative[1].a * std::pow(t, 2)) + (this->derivative[1].b * t) + this->derivative[1].c;

    Line secondDerivX = derivativeOfQuadratic(this->derivative[1]);
    Line secondDerivY = derivativeOfQuadratic(this->derivative[0]);

    double solvedSDX = (secondDerivX.slope * t) + secondDerivX.yIntercept;
    double solvedSDY = (secondDerivY.slope * t) + secondDerivY.yIntercept;

    double curvature = (std::abs((derivativeX * solvedSDY) - (derivativeY * solvedSDX))) / std::pow(std::sqrt((std::pow(derivativeX, 2) + std::pow(derivativeY, 2))), 3);

    return curvature;
}

// given a point, finds the nearest point to that point on the profile and returns its t
double CubicHermiteSpline::findNearestPointOnSpline(Point givenPoint, double excludeBelow) {
    double closestT = -1;
    double currentT = excludeBelow;
    double closestDistance = 10000;
    double currentDistance = 0;
    std::cout << currentT * fullSampleSpline.size() << "\n";
    for (int i = (currentT * fullSampleSpline.size()); i < fullSampleSpline.size(); i++) {
        currentDistance = std::pow((givenPoint.x - fullSampleSpline[i].x), 2) + std::pow((givenPoint.y - fullSampleSpline[i].y), 2);
        currentT = (i + 1) / (double) fullSampleSpline.size();
        if ((currentDistance < closestDistance) && (currentT > excludeBelow)) {
            closestT = currentT;
            closestDistance = currentDistance;
        }
        if (currentDistance > closestDistance) {
            break;
        }
    }
    return closestT;
}

double CubicHermiteSpline::calculateCurveSpeed(double t) {
    double veloVecX = (this->derivative[1].a * std::pow(t, 2)) + (this->derivative[1].b * t) + (this->derivative[1].c);
    double veloVecY = (this->derivative[0].a * std::pow(t, 2)) + (this->derivative[0].b * t) + (this->derivative[0].c);
    double speed = (std::sqrt(std::pow(veloVecX, 2) + std::pow(veloVecY, 2)));
    return speed;
}

double CubicHermiteSpline::findNextT(double currentT, double distanceToMove) {
    double deltaT = distanceToMove / this->calculateCurveSpeed(currentT);
    double nextT = currentT + deltaT;
    return nextT;
}