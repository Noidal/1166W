#include "profile.h"

PhysicalConstraints::PhysicalConstraints(double rpm, double gearRatio, double wheelDiameter, double distBetweenDTSides, double mass, double dtEfficiency, double fricCoef) {
    // physical properties
    this->gearRatio = gearRatio;
    this->wheelDiameter = wheelDiameter;

    // velocity constraints
    this->maxSpeed = (rpm * gearRatio * (M_PI * wheelDiameter)) / 60;
    this->dtDiff = distBetweenDTSides;
    this->maxRPM = rpm;

    // acceleration constraints
    this->maxTorque = 210 / this->maxRPM;
    double maxMotorAccel = (2 * (((2 * (this->maxTorque * gearRatio * dtEfficiency)) + (0.5 * 3 * dtEfficiency)) / (((wheelDiameter * 0.0254) / 2) * mass))) * 39.37;
    double maxFricAccel = 386 * fricCoef;

    this->maxAccel = std::min(maxMotorAccel, maxFricAccel);
    //this->maxAccel = 30;
    this->maxDecel = maxAccel;
    this->maxAngAccel = (2 * maxAccel) / this->dtDiff;
}

double PhysicalConstraints::maxVelocityAtCurvature(double curvature) {
    double maxSpeed = (2 * this->maxSpeed) / ((std::abs(curvature) * this->dtDiff) + 2);

    return maxSpeed;
}

double PhysicalConstraints::maxLinAccelAtAngAccel(double angAccel) {
    double linAccel = this->maxAccel - std::abs((angAccel * this->dtDiff) / 2.0);

    return linAccel;
}

std::vector<double> PhysicalConstraints::outputOfSides(double linearVelocityIPS, double angularVelocityRADPS) {
    
    double leftVelocityIPS = linearVelocityIPS - ((angularVelocityRADPS * this->dtDiff) / 2.0); // lv = v - ((w * L) / 2)
    double rightVelocityIPS = linearVelocityIPS + ((angularVelocityRADPS * this->dtDiff) / 2.0); // rv = v + ((w * L) / 2)

    // return {leftVelocityIPS, rightVelocityIPS};

    double leftVelocityRPM = (leftVelocityIPS * 60.0 / (M_PI * this->wheelDiameter)) / this->gearRatio; // rpm = m/s * (60 s / min) * (1 rotation / (single degree travel * 360))
    double rightVelocityRPM = (rightVelocityIPS * 60.0 / (M_PI * this->wheelDiameter)) / this->gearRatio; // rpm = m/s * (60 s / min) * (1 rotation / (single degree travel * 360))

    return {leftVelocityRPM, rightVelocityRPM};
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

    // discretize spline parameterized by distance
    while (currentT < 1) {
        profile.push_back({path->findPose(currentT).x, path->findPose(currentT).y, path->findPose(currentT).heading, 0, 0, currentT});

        currentT = path->advanceLength(currentT, this->distSeg);
    }

    // forward pass
    for (int i = 0; i < profile.size(); i++) {

        prevPoint = (i > 0) ? profile[i - 1] : MPPoint{0, 0, 0, 0, 0, -0.001};

        // velocity calculations
        double curvLimitedLinVel = robot.maxVelocityAtCurvature(this->path->calculateCurvature(profile[i].t));
        double accelLimitedLinVel = std::sqrt(std::pow(prevPoint.linVel, 2) + (2.0 * robot.maxAccel * this->distSeg));
        /*
        double dkappads = std::abs(this->path->calculateCurvature(profile[i].t) - this->path->calculateCurvature(prevPoint.t)) / this->distSeg;
        double dkappadsLimitedLinVel = std::sqrt(robot.maxAngAccel / dkappads);
        profile[i].linVel = std::min(std::min(curvLimitedLinVel, accelLimitedLinVel), dkappadsLimitedLinVel);
        */
        profile[i].linVel = std::min(curvLimitedLinVel, accelLimitedLinVel);

        // the angular velocity is the curvature of the current point multiplied by the current linear velocity
        profile[i].angVel = profile[i].linVel * this->path->calculateCurvature(profile[i].t);
    }

    profile[profile.size() - 1].linVel = 0;
    profile[profile.size() - 1].angVel = 0;

    // backward pass
    for (int i = profile.size() - 2; i >= 0; i--) {

        prevPoint = (i < profile.size() - 1) ? profile[i + 1] : MPPoint{0, 0, 0, 0, 0, 1.001};

        // velocity calculations
        double curvLimitedLinVel = robot.maxVelocityAtCurvature(this->path->calculateCurvature(profile[i].t));
        double accelLimitedLinVel = std::sqrt(std::pow(prevPoint.linVel, 2) + (2.0 * robot.maxDecel * this->distSeg));

        double dkappads = std::abs(this->path->calculateCurvature(profile[i].t) - this->path->calculateCurvature(prevPoint.t)) / this->distSeg;
        double dkappadsLimitedLinVel = std::sqrt(robot.maxAngAccel / dkappads);
        profile[i].linVel = std::min(profile[i].linVel, std::min(std::min(curvLimitedLinVel, accelLimitedLinVel), dkappadsLimitedLinVel));

        profile[i].linVel = std::min(profile[i].linVel, std::min(curvLimitedLinVel, accelLimitedLinVel));

        // the angular velocity is the curvature of the current point multiplied by the current linear velocity
        profile[i].angVel = profile[i].linVel * this->path->calculateCurvature(profile[i].t);
    }

    double accumulatedTime = 0;

    for (int i = 0; i < profile.size(); i++) {
        accumulatedTime += (this->distSeg / profile[i].linVel) * 1000;
        profile[i].timeAtPoint = accumulatedTime;
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