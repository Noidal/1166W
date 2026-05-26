#include "math.h"

double findHeadingOfLine(
    Point point1, // the initial point
    Point point2 // the final point
)
{
    // finds the difference between the x- and y- values to get the rise and run of the line
    double xChange = point2.x - point1.x;
    double yChange = point2.y - point1.y;

    bool yIsPositive = yChange > 0;
    bool xIsPositive = xChange > 0;

    // finds the positive position of the angle in relation to the previous multiple of 90 degrees
    double triangleAngle = 0;
    if ((yIsPositive && xIsPositive) || (!yIsPositive && !xIsPositive)) { // quadrants 1 or 3
        triangleAngle = std::atan(std::abs(xChange) / std::abs(yChange)); // tangent is opposite/adjacent, and x is opposite in these cases
    }
    else { // quadrants 2 or 4
        triangleAngle = std::atan(std::abs(yChange) / std::abs(xChange)); // tangent is opposite/adjacent, and y is opposite in these cases
    }
    triangleAngle = (triangleAngle * 180) / M_PI;

    double heading = 0;
    if (xIsPositive && yIsPositive) { // quadrant 1
       heading = triangleAngle;
    } else if (xIsPositive && !yIsPositive) { // quadrant 2
        heading = triangleAngle + 90;
    } else if (!xIsPositive && !yIsPositive) { // quadrant 3
        heading = triangleAngle + 180;
    } else if (!xIsPositive && yIsPositive) { // quadrant 4
        heading = triangleAngle + 270;
    }

    return heading;
}

// distance formula function
double calculateDistance(Point point1, Point point2) {
    return std::sqrt(std::pow((point2.x - point1.x), 2) + std::pow((point2.y - point1.y), 2));
}

Line findLineWithPoints(
    Point point1,
    Point point2
)
{
    Line Line1;
    Line LinePerp;

    if (point2.y - point1.y == 0) {
        Line1.slope = 0;
        Line1.yIntercept = point1.y;
        return Line1;
    }
    else if (point2.x - point1.x == 0) {
        Line1.slope = NAN;
        Line1.yIntercept = point2.y;
        return LinePerp;
    }
    
    // handles all y = mx + b cases
    Line1.slope = (point2.y - point1.y) / (point2.x - point1.x); // calculates the slope of the first line
    Line1.yIntercept = (-1 * (Line1.slope * point1.x)) + point1.y; // formula for y-intercept based on point-slope form
    return Line1;
}

QuadraticPolyData derivativeOfCubicPoly(CubicPolyData cubicPoly) {
    // the power rule is used on each term to find the derivative of the 
    double a = cubicPoly.a * 3; // power rule to find the x^2 coefficient
    double b = cubicPoly.b * 2; // power rule to find the x^1 coefficient
    double c = cubicPoly.c; // power rule to find the x^0 coefficient

    QuadraticPolyData cubicDerivative = {a, b, c};

    return cubicDerivative;
}

Line derivativeOfQuadratic(QuadraticPolyData quadPoly) {
    // the power rule is used on each term to find the derivative of the 
    double a = quadPoly.a * 2; // power rule to find the x^1 coefficient
    double b = quadPoly.b; // power rule to find the x^0 coefficient

    Line quadDerivative = {a, b};

    return quadDerivative;
}

Line findLineWithHeading(
    Point point1, // a point on the line
    int heading // inertial heading
)
{
    // calculates the angle of only the triangle by subtracting from it based on its quadrant
    double triangleAngle = 0;
    if (heading < 90) {
       triangleAngle = heading;
    } else if (heading < 180) {
        triangleAngle = heading - 90;
    } else if (heading < 270) {
        triangleAngle = heading - 180;
    } else if (heading < 360) {
        triangleAngle = heading - 270;
    }
    // treats the distance moved as the hypotenuse of and the heading as the base angle of a triangle
    // and uses them to calculate the value of both legs (the changes in x and y)
    double xChange = 0;
    double yChange = 0;
    double hypotenuse = 1; // because the hypotenuse is an infinite line, the length for this calculation does not matter, so I picked 1 for ease of calculation

    // if the heading is in quadrants 1 or 3, then the x-value is the opposite leg (sine) and the y-value is the adjacent leg (cosine)
    if ((heading < 90) || (heading >= 180 && heading < 270)) {
        xChange = std::sin(((triangleAngle * M_PI) / 180)) * hypotenuse;
        yChange = std::cos(((triangleAngle * M_PI) / 180)) * hypotenuse;
    }
    // otherwise, if the heading is in quadrants 2 or 4, then the x-value is the adjacent leg (cosine) and the y-value is the opposite leg (sine)
    else {
        xChange = std::cos(((triangleAngle * M_PI) / 180)) * hypotenuse;
        yChange = std::sin(((triangleAngle * M_PI) / 180)) * hypotenuse;
    }

    // slope of line = rise / run
    double slope = yChange / xChange;

    // calculates the y-intercept with a derived form of the point-slope form of an equation
    double yIntercept = (-1 * (slope * point1.x)) + point1.y;

    Line line = {slope, yIntercept};

    return line;
}

Line calculatePerpendicularNonInequality(
    Point point1, // has a line between it and point2 that will have a line perpendicular to it
    Point point2 // the point that the perpendicular line will cross
)
{
    Line Line1;
    Line LinePerp;

    if (point2.y - point1.y == 0) { // handles the case if y does not change (line takes the form of x = k)
        LinePerp.slope = NAN;
        LinePerp.yIntercept = point2.x; // yIntercept serves as the value of k (in x = k) for this case
        return LinePerp;
    }
    else if (point2.x - point1.x == 0) { // handles the case if x does not change (line takes the form of y = k)
        LinePerp.slope = 0;
        LinePerp.yIntercept = point2.y; // yIntercept serves as the value of k (in y = k) for this case
        return LinePerp;
    }
    
    // handles all y = mx + b cases
    Line1.slope = (point2.y - point1.y) / (point2.x - point1.x); // calculates the slope of the first line
    Line1.yIntercept = (-1 * (Line1.slope * point1.x)) + point1.y; // formula for y-intercept based on point-slope form
    

    LinePerp.slope = -1 * (1 / Line1.slope); // slope of a perpendicular line is the negative reciprocal of the slope of the original
    LinePerp.yIntercept = (-1 * (LinePerp.slope * point2.x)) + point2.y;

    return LinePerp;
}

// calculates the standard deviation of a set of values, used for the filter (private)
double calculateStandardDeviation(
    std::deque<double> listOfValues // list of values to use
)
{
    // calculates the mean of the values
    double meanOfValues = 0;
    for (int i = 0; i < listOfValues.size(); i++) {
        meanOfValues += listOfValues[i]; // adds each value together to calculate their mean
    }
    meanOfValues = meanOfValues / listOfValues.size();

    // calculates the variance of the values
    std::vector<double> listOfDifferences;
    double variance = 0;
    for (int i = 0; i < listOfValues.size(); i++) {
        variance += std::pow((listOfValues[i] - meanOfValues), 2); // variance is calculated by adding the squares of each value's difference from the mean together
    }
    variance = variance / listOfValues.size();

    double standardDeviation = std::sqrt(variance); // standard deviation = square root of variance

    return standardDeviation;
}

Inequality calculatePerpendicularInequality(
    Point point1, // has a line between it and point2 that will have a line perpendicular to it
    Point point2 // the point that the perpendicular line will cross
)
{
    Inequality Line1;
    Inequality LinePerp;

    if (point2.y - point1.y == 0) { // handles the case if y does not change (line takes the form of x = k)
        LinePerp.slope = NAN;
        LinePerp.yIntercept = point2.x; // yIntercept serves as the value of k (in x = k) for this case
        LinePerp.equality = point1.x > point2.x // a copy of findEquality that simply compares the x-values of both points to determine the equality
                                ? 2 // if the value of the first point is greater than the value of the second point, 
                                     // then the equality is on the opposite side (negative)
                                : -2;
        return LinePerp;
    }
    else if (point2.x - point1.x == 0) { // handles the case if x does not change (line takes the form of y = k)
        LinePerp.slope = 0;
        LinePerp.yIntercept = point2.y; // yIntercept serves as the value of k (in y = k) for this case
        LinePerp.equality = point1.y > point2.y // a copy of findEquality that simply compares the x-values of both points to determine the equality
                                ? 2 // if the value of the first point is greater than the value of the second point, 
                                     // then the equality is on the opposite side (negative)
                                : -2;
        return LinePerp;
    }
    
    // handles all y = mx + b cases
    Line1.slope = (point2.y - point1.y) / (point2.x - point1.x); // calculates the slope of the first line
    Line1.yIntercept = (-1 * (Line1.slope * point1.x)) + point1.y; // formula for y-intercept based on point-slope form
    Line1.equality = 0; // this line is just treated as an equation
    

    LinePerp.slope = -1 * (1 / Line1.slope); // slope of a perpendicular line is the negative reciprocal of the slope of the original
    LinePerp.yIntercept = (-1 * (LinePerp.slope * point2.x)) + point2.y;
    LinePerp.equality = findEquality(LinePerp, point1); // sets the inequality to include the side with point 1 on it

    return LinePerp;
}

int findEquality(
    Inequality line, // the line to find the equality of
    Point includedPoint // a point on the side of the inequality that will be part of the inequality
)
{
    double result = (line.slope * includedPoint.x) + line.yIntercept; // the result of the equation when x from the point is plugged into it

    if (includedPoint.y != result) { // if the actual y-value is not the same as the y-value that the line produces, they are not equal
        if (includedPoint.y > result) { // if the actual y-value is greater than the line's y-value, then the included part is above the line
            return 2; // 2 signifies greater than for the Line structure
        }
        else { // if the actual y-value is less than the line's y-value, then the included part is below the line
            return -2; // -2 signifies less than for the Line structure
        }
    } else { // if the point is on the line, then the line is an equation
            return 0; // 0 signifies equal to on the Line structure
    }
}

Point findIntersection(
    Line line1, // the first line
    Line line2 // the second line
)
{
    if (std::isnan(line1.slope)) {
        return {line1.yIntercept, ((line2.slope * line1.yIntercept) + line2.yIntercept)};
    } else if (std::isnan(line2.slope)) {
        return {line2.yIntercept, ((line1.slope * line2.yIntercept) + line1.yIntercept)};
    }

    Point intersect = {0, 0};
    // first, the x-intersect is found with substitution
    // this starts with y = ax + b and y = cx + d, which can be substituted into ax + b = cx + d
    double mergedSlope = line1.slope - line2.slope;  // ax - cx, which leaves one x term on the left side and none on the right: (a-c)x + b = d
    double mergedYInt = line2.yIntercept - line1.yIntercept; // d - b, which leaves one constant on the right and none on the left: (a-c)x = (d-b)
    intersect.x = mergedYInt / mergedSlope; // (d-b) divided by (a-c), which leaves x isolated on the left side: x = (a-c)/(d-b)

    // next, the y-intersect is found by substituting the x-intersect into one equation
    intersect.y = (line1.slope * intersect.x) + line1.yIntercept;

    return intersect;
}

// fixes angle from a vertical 0, increasing clockwise to the coordinates of the unit circle
double fixAngle(
    double originalAngle // an angle that uses 0 as the top and increases clockwise
)
{
    double fixedAngle = 0;
    if (originalAngle <= 90) {
        fixedAngle = (originalAngle + 90) - (2 * ((originalAngle + 90) - 90));
    } else if (originalAngle <= 180) {
        fixedAngle = 360 - (90 - (360 - (originalAngle + 180)));
    } else if (originalAngle <= 270) {
        fixedAngle = (originalAngle + 90) - (2 * ((originalAngle + 90) - 270));
    } else if (originalAngle < 360) {
        fixedAngle = (originalAngle - 270) + (2 * (90 - (originalAngle - 270)));
    } else {
        fixedAngle = -1;
    }
    return fixedAngle;
}

double unfixAngle(
    double originalAngle
)
{
    double unfixedAngle = 0;
    if (originalAngle <= 90) {
        unfixedAngle = -originalAngle + 90;
    } else if (originalAngle <= 180) {
        unfixedAngle = -originalAngle + 450;
    } else if (originalAngle <= 270) {
        unfixedAngle = -originalAngle + 450;
    } else if (originalAngle < 360) {
        unfixedAngle = -originalAngle + 450;
    } else {
        unfixedAngle = -1;
    }
    return unfixedAngle;
}

double RPMtoIPS(double rpm) {
    // Global Values
    double g_gearRatio6 = 0.8;
    double g_gearRatio2 = 3;
    double g_maxRPM = 480;
    double g_diameter = 4;
    double g_distBetweenWheels = 7.5;
    
    return (rpm * g_gearRatio6 * (M_PI * g_diameter)) / 60;
}

double IPStoRPM(double ips) {
    // Global Values
    double g_gearRatio6 = 0.8;
    double g_gearRatio2 = 3;
    double g_maxRPM = 480;
    double g_diameter = 4;
    double g_distBetweenWheels = 7.5;

    return (ips / (M_PI * g_diameter) / g_gearRatio6) * 60;
}

double bind180(double heading) {
    if (heading < -180) {heading += 360;}
    if (heading > 180) {heading -= 360;}
    return heading;
}

double makeRelative(double currentHeading, double goalHeading) {
    int dir = std::signbit(goalHeading) ? -1 : 1;
    
    goalHeading = std::abs(goalHeading) - currentHeading;
    if (goalHeading < 0) {goalHeading += 360;}
    if (dir < 0) {goalHeading = -1 * (360 - goalHeading);}

    return goalHeading;
}