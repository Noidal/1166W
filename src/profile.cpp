#include "profiling.h"

MotionProfile::MotionProfile(CubicHermiteSpline* path, double maxSpeed, std::vector<std::vector<Point>> zonePoints) {
    // assigns the passed-in values to instance variables
    this->path = path;
    this->maxSpeed = maxSpeed;
    this->isHolo = false;
    // default velocity zoning
    // (0-10% goes from x0.1-1, 10-90% remains at x1, 90-100% goes from x1-0)
    this->zones = {
        {0, 0.1, {9, 0.1}},
        {0.1, 0.9, {0, 1}},
        {0.9, 1, {-10, 10}}
    };
    // custom velocity zoning
    if (zonePoints.size() > 0) {
        this->constructWithCustomZones(zonePoints);
    }
    // ends by creating the profile
    this->generateVelocities();
}

MotionProfile::MotionProfile(CubicHermiteSpline* path, double maxSpeed, std::vector<double> headings, std::vector<double> headingTs, std::vector<std::vector<Point>> zonePoints) {
    // assigns the passed-in values to instance variables
    this->path = path;
    this->maxSpeed = maxSpeed;
    this->headings = headings;
    headings.insert(headings.begin(), 0);
    this->headingTs = headingTs;
    headingTs.insert(headingTs.begin(), headingTs[0]);
    this->isHolo = true;
    // default velocity zoning
    // (0-10% goes from x0.1-1, 10-90% remains at x1, 90-100% goes from x1-0)
    this->zones = {
        {0, 0.1, {9, 0.1}},
        {0.1, 0.9, {0, 1}},
        {0.9, 1, {-10, 10}}
    };
    // custom velocity zoning
    if (zonePoints.size() > 0) {
        this->constructWithCustomZones(zonePoints);
    }
    // ends by creating the profile
    this->generateVelocities();
}


// puts a previously generated profile into the motion profile class so it can be read as a normal motion profile
MotionProfile::MotionProfile(std::vector<MPPoint>* pregeneratedProfile, double maxSpeed) {
    this->path = NULL;
    this->zones = {};
    this->maxSpeed = maxSpeed;
    this->profile = *pregeneratedProfile;
}


void MotionProfile::constructWithCustomZones(std::vector<std::vector<Point>> zoneLinePoints) {
    this->zones = {};
    Line currentZoneLine;
    // generates the velocity lines
    for (int i = 0; i < zoneLinePoints.size(); i++) {
        currentZoneLine = findLineWithPoints(zoneLinePoints[i][0], zoneLinePoints[i][1]);
        this->zones.push_back({zoneLinePoints[i][0].x, zoneLinePoints[i][1].x, currentZoneLine});
    }
    // tests to make sure each point has a valid velocity, with a sample size of 1000
    for (int i = 0; i < 1000; i++) {

        double t = i / (double) 1000;
        Zone assignedZone = {NAN, NAN, {NAN, NAN}};

        for (int j = 0; j < this->zones.size(); j++) {
            if ((this->zones[j].startT <= t) && (this->zones[j].endT >= t)) {
                assignedZone = this->zones[j];
            }
        }
        // if a point doesn't have a valid zone, the default zoning is used instead
        if (std::isnan(assignedZone.startT)) {
            // pros::lcd::print(0, "Invalid velocity zoning! Using default as substitute.");
            this->zones = {
                {0, 0.1, {9, 0.1}},
                {0.1, 0.9, {0, 1}},
                {0.9, 1, {-10, 10}}
            };
            return;
        }
    }
    return;
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
    Pose currentPoint = this->path->findPose(0, 0.0001);
    double currentT = 0;
    double prevT = -0.0001;
    double distanceToNext = 0;
    while (true) {

        currentPoint = this->path->findPose(currentT, 0.0001);

        double newHeading = 0;
        Pose nextPoint = {};
        if (this->isHolo) {
            newHeading = this->customHeading(currentT);
            nextPoint = this->path->findPose(currentT + 0.0001, 0.0001);
        }

        Zone assignedZone = {NAN, NAN, {NAN, NAN}};

        // iterates through the zones until the one that contains the current point's t value is found
        for (int j = 0; j < this->zones.size(); j++) {
            if ((this->zones[j].startT <= currentT) && (this->zones[j].endT >= currentT)) {
                assignedZone = this->zones[j];
            }
        }

        // determines the direction of the curve by examining whether the heading is changing to the right or to the left
        Pose prevPoint = this->path->findPose(prevT, 0.0001);
        double dir;
        if (prevPoint.heading > currentPoint.heading) {dir = 1;}
        else if (currentPoint.heading > prevPoint.heading) {dir = -1;}
        else {dir = 0;}

        // velocity calculations
        // the zone line is solved for at the point t to find the velocity multiplier
        double linearVelocityMultiplier = (assignedZone.zoneLine.slope * currentT) + assignedZone.zoneLine.yIntercept;
        // if (pointList.size() != 1000) {std::cout << linearVelocityMultiplier << ", t = " << t << "\n";}
        // the linear velocity is simply the multiplier (which is a percentage) of the maximum speed allowed by the profile
        double currentMaxSpeed = this->maxSpeed;
        double movementAngle = (std::atan2(nextPoint.y - currentPoint.y, nextPoint.x - currentPoint.x));
        if (isHolo) {currentMaxSpeed = this->maxSpeed * (1 / std::abs(std::sin(nextPoint.x - currentPoint.x) + std::cos(nextPoint.y - currentPoint.y)));}
        currentMaxSpeed = currentMaxSpeed;
        double linearVelocity = linearVelocityMultiplier * this->maxSpeed;
        // the angular velocity is the curvature of the current point multiplied by the current linear velocity
        double angularVelocity = linearVelocity * this->path->calculateCurvature(currentT) * dir; //* (double) curveDirection;
        
        // adds the new velocities and point as the next profile point
        profile.push_back({currentPoint.x, currentPoint.y, currentPoint.heading, linearVelocity, angularVelocity, currentT});
        if (isHolo) {
            holoProfile.push_back({currentPoint.x, currentPoint.y, newHeading, {nextPoint.x - currentPoint.x, nextPoint.y - currentPoint.y, linearVelocity, movementAngle}, 0, currentT});
        }
/*
        // redefines the current point to the next point for the next loop
        currentPoint.x = currentPoint.x + ((linearVelocity * 0.005) * std::cos(fixAngle(currentPoint.heading) * (M_PI / 180)));
        currentPoint.y = currentPoint.y + ((linearVelocity * 0.005) * std::sin(fixAngle(currentPoint.heading) * (M_PI / 180)));
        currentT = this->path->findNearestPointOnSpline({currentPoint.x, currentPoint.y}, prevT);

        // std::cout << "cx = " << currentT << ", cy = " << currentT << "\n";

        // std::cout << "Working... t=" << currentT << "\n";

        //std::cout << "x = " << currentPoint.x << ", y = " << currentPoint.y << "\n";
        // std::cout << "r = " << currentT - prevT << "\n";
*/
        if (linearVelocity < 1) {linearVelocity = 1;}
        distanceToNext = linearVelocity * 0.005;
        prevT = currentT;
        currentT = this->path->findNextT(currentT, distanceToNext);
        if (currentT >= 1) {
            break;
        }
    }
}

// given a value of t on the profile, finds the profile's closest generated point
MPPoint MotionProfile::findNearestPoint(double givenT) {
    if (this->isHolo) {return {};}
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

// given a value of t on the profile, finds the profile's closest generated point
HoloMPPoint MotionProfile::findNearestHoloPoint(double givenT) {
    if (!this->isHolo) {return {};}
    HoloMPPoint closestCandidate;
    double closestDifference = 10000;
    for (int i = 0; i < holoProfile.size(); i++) {
        if (std::abs(givenT - this->holoProfile[i].t) < closestDifference) {
            closestCandidate = this->holoProfile[i];
            closestDifference = std::abs(givenT - this->profile[i].t);
        }
    }
    return closestCandidate;
}