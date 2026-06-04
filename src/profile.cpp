#include "profile.h"

PhysicalConstraints::PhysicalConstraints(double rpm, double gearRatio, double wheelDiameter, double distBetweenDTSides) {
    this->maxSpeed = (rpm * gearRatio * (M_PI * wheelDiameter)) / 60;
    this->dtDiff = distBetweenDTSides;
    this->maxRPM = rpm;
}

double PhysicalConstraints::maxVelocityAtCurvature(double curvature) {
    double maxSpeed = (2 * this->maxSpeed) / ((std::abs(curvature) * this->dtDiff) + 2);

    return maxSpeed;
}

MotionProfile::MotionProfile(CubicHermiteSpline* path, PhysicalConstraints robot, double distSeg) {
    // assigns the passed-in values to instance variables
    this->path = path;
    this->robot = robot;
    this->distSeg = distSeg;
    // ends by creating the profile
    this->generateVelocities();
}


// puts a previously generated profile into the motion profile class so it can be read as a normal motion profile
MotionProfile::MotionProfile(std::vector<MPPoint>* pregeneratedProfile, PhysicalConstraints robot, double distSeg) {
    this->path = NULL;
    this->robot = robot;
    this->profile = *pregeneratedProfile;
    this->distSeg = distSeg;
}

double MotionProfile::customHeading(double t) {
    int loc = 0;
    for (int i = 0; i < this->headingTs.size(); i++) {
        if (this->headingTs[i] > t) {
            loc = i;
            break;
        }
        return this->headings[headings.size() - 1];
    }
    double newHeading = this->headings[loc - 1] + ((this->headings[loc] - this->headings[loc - 1]) * ((t - this->headingTs[loc - 1]) / (this->headingTs[loc] - this->headingTs[loc - 1])));
    return newHeading;
}

void MotionProfile::generateVelocities() {
    MPPoint currentPoint;
    MPPoint prevPoint;
    double currentT = 0;
    double distanceToNext = 0;

    while (currentT < 1) {
        profile.push_back({path->findPose(currentT).x, path->findPose(currentT).y, path->findPose(currentT).heading});

        currentT = path->advanceLength(currentT, this->distSeg);
    }

    for (int i = 0; currentT < 1; i++) {

        currentPoint = profile[i];
        prevPoint = (i > 0) ? profile[i - 1] : MPPoint{};

        // velocity calculations
        double curvLimitedLinVel = robot.maxVelocityAtCurvature(this->path->calculateCurvature(currentT));
        double accelLimitedLinVel = prevPoint.linVel + (0.005 * robot.maxAccel);
        currentPoint.linVel = std::min(curvLimitedLinVel, accelLimitedLinVel);

        // the angular velocity is the curvature of the current point multiplied by the current linear velocity
        currentPoint.angVel = currentPoint.linVel * this->path->calculateCurvature(currentT);
        
        // adds the new velocities and point as the next profile point
        profile.push_back(currentPoint);


        if (currentPoint.linVel < 1) {currentPoint.linVel = 1;}
        distanceToNext = currentPoint.linVel * 0.005;
        currentT = this->path->findNextT(currentT, distanceToNext);
    }

    for (int i = profile.size() - 1; currentT > 0; i--) {

        currentPoint = {this->path->findPose(currentT).x, this->path->findPose(currentT).y, this->path->findPose(currentT).heading};
        prevPoint = profile[profile.size() - 1];

        // velocity calculations
        double curvLimitedLinVel = robot.maxVelocityAtCurvature(this->path->calculateCurvature(currentT));
        double accelLimitedLinVel = prevPoint.linVel + (0.005 * robot.maxAccel);
        currentPoint.linVel = std::min(curvLimitedLinVel, accelLimitedLinVel);

        // the angular velocity is the curvature of the current point multiplied by the current linear velocity
        currentPoint.angVel = currentPoint.linVel * this->path->calculateCurvature(currentT);
        
        // adds the new velocities and point as the next profile point
        profile.push_back(currentPoint);


        if (currentPoint.linVel < 1) {currentPoint.linVel = 1;}
        distanceToNext = currentPoint.linVel * 0.005;
        currentT = this->path->findNextT(currentT, distanceToNext);
    }
}

// given a value of t on the profile, finds the profile's closest generated point
MPPoint MotionProfile::findNearestPoint(double givenT) {
    MPPoint closestCandidate;
    double closestDifference = 10000;
    for (int i = 0; i < profile.size(); i++) {
        if (std::abs(givenT - this->profile[i].t) < closestDifference) {
            closestCandidate = this->profile[i];
            closestDifference = std::abs(givenT - this->profile[i].t);
        }
    }
    return closestCandidate;
}