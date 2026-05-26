#ifndef _PROS_CONFIG_H_
#define _PROS_CONFIG_H_

#include "main.h"
#include "chassis.h"
#include "interface.h"
#include "odom.h"
#include "pid.h"
#include "profiling.h"
#include "kalman.h"
#include "math.h"
#include "mcl.h"

// Controllers
    pros::Controller master(pros::E_CONTROLLER_MASTER);

// Motors
    // Mecanum Drivetrain
        
        pros::Motor topLeft6(-6, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor topLeft2(-7, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);

        pros::Motor bottomLeft6(-8,pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor bottomLeft2(-9, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);

        pros::Motor topRight6(1, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor topRight2(2, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);

        pros::Motor bottomRight6(3, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor bottomRight2(4, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);
        
    // Intake
        pros::Motor input(18, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor storage(17, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor output(19, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);

    // Sensors
        /* pros::Rotation parallelLeftOdom(12);
        pros::Rotation parallelRightOdom(13); */
        // pros::Rotation perpOdom(11);

        pros::Distance front(12);
        pros::Distance back(11);
        pros::Distance right(13);

        pros::IMU inertial1(14);
        pros::IMU inertial2(15);

        pros::Optical color(20);

    // Three-Wire Devices
        pros::adi::DigitalOut unloader(8);
        pros::adi::DigitalOut aligner(1);
        pros::adi::DigitalOut descorer(2);

// Program Module Initialization

    // DiffChassis chassis = DiffChassis({&topLeft6, &topLeft2, &bottomLeft6, &bottomLeft2}, {&topRight6, &topRight2, &bottomRight6, &bottomRight2});

    /* OdomPod leftOdom(&parallelLeftOdom, 2);
    OdomPod rightOdom(&parallelRightOdom, 2);
    TrackingSensor fbTrack(
        []() -> double {
            return ((leftOdom.measure() + rightOdom.measure()) / 2);
        },
        [](double val) {
            parallelLeftOdom.set_position(val);
            parallelRightOdom.set_position(val);
            return;
        },
        []() {
            parallelLeftOdom.reset_position();
            parallelRightOdom.reset_position();
            return;
        }
    );
    /* 
    TrackingSensor angVelTracker(
        []() -> double {
            return ((leftOdom.measureVelocity() - rightOdom.measureVelocity()) / 5);
        }
    ); */
    TrackingSensor angVelTracker(
        []() -> double {
            return 0.0001;
        }
    );

    /*OdomPod perpendicularOdom = OdomPod(&perpOdom, 2);
    TrackingSensor lrTrack(
        []() -> double {
            return perpendicularOdom.measure();
        },
        [](double val) {
            perpOdom.set_position(val);
            return;
        },
        []() {
            perpOdom.reset_position();
            return;
        }
    );*/

    KalmanFilter Kalman1 = KalmanFilter(&inertial1, angVelTracker);
    KalmanFilter Kalman2 = KalmanFilter(&inertial2, angVelTracker);

    TrackingSensor headingTracker(
        []() -> double {
            return getAggregatedHeading(Kalman1, Kalman2);
        }
    );

    double distFromLastReset = 0;
    double lastHeading = 90;
    TrackingSensor RelativePIDHeadingTracker(
        []() -> double {
            double changeInHeading = getAggregatedHeading(Kalman1, Kalman2) - lastHeading;
            if (changeInHeading > 315) {
                changeInHeading -= 360;
            } else if (changeInHeading < -315) {
                changeInHeading += 360;
            }
            distFromLastReset += changeInHeading;
            lastHeading = getAggregatedHeading(Kalman1, Kalman2);
            std::cout << "pidhead: " << distFromLastReset << "\n\n";
            return distFromLastReset;
        },
        [](double val) {
            return;
        },
        []() {
            distFromLastReset = 0;
            return;
        }
    );

    // Pose startPose = {0, 0, 0};

    // Odometry odom(fbTrack, headingTracker, startPose);

    ConstantContainer fbConstants = {4, 0.1, 2.7};
    ConstantContainer thetaConstantsSub90 = {3, 0.2, 26};
    ConstantContainer thetaConstantsAbove90 = {2.3, 0.24, 32};

    double fbTol = 1;
    double thetaTolSub90 = 2.5;
    double thetaTolAbove90 = 3.5;

    // PIDSet robotPIDs(&xPID, &yPID, &thetaPIDSub90, &thetaPIDAbove90);
    // PoseTracker currentPose(&odom);

    /*
    TrackingSensor frontDistance(
        []() -> double {
            double val = front.get();
            return val == 9999 ? -1 : (val / 10) / 2.54;
        }
    );

    TrackingSensor leftDistance(
        []() -> double {
            double val = left.get();
            return val == 9999 ? -1 : (val / 10) / 2.54;
        }
    );

    TrackingSensor rightDistance(
        []() -> double {
            double val = right.get();
            return val == 9999 ? -1 : (val / 10) / 2.54;
        }
    );*/

    TrackingSensor linAngleTracker(
        []() -> double {
            double offsetFromHeading = chassis.m_lAng.get() < 180 ? chassis.m_lAng.get() : chassis.m_lAng.get() - 360;
            return headingTracker.get() + offsetFromHeading;
        }
    );

    TrackingSensor linAngleTrackerV2(
        []() -> double {
            double offsetFromHeading = chassis.m_lAng.get() < 180 ? chassis.m_lAng.get() : chassis.m_lAng.get() - 360;
            return offsetFromHeading;
        }
    );

    // VelocityController follower(&chassis.m_xOutput, &chassis.m_yOutput, &chassis.m_thetaOutput, &currentPose, robotPIDs);

    /* ParticleFilter mcl(frontDistance, leftDistance, rightDistance, headingTracker, 
                       chassis.m_lVel, linAngleTrackerV2, chassis.m_aVel, 
                       {{-4, 7.5}, {-7.5, 0}, {7.5, 0}}, {0, -90, 90},
                       {40, -24, 0});

    Pose mclResetPose = {0, 0, 0};
    TrackingSensor mclFBTrack(
        []() -> double {
            Point change = {mcl.getBestParticle().x - mclResetPose.x, mcl.getBestParticle().y - mclResetPose.y};
            double linChange = (std::cos((M_PI / 180) * mclResetPose.heading) * change.x) + (std::sin((M_PI / 180) * mclResetPose.heading) * change.y);
            return linChange;
        },
        [](double val) {},
        []() {
            mclResetPose = {mcl.getBestParticle().x, mcl.getBestParticle().y, headingTracker.get()};
        }
    ); */
    double fVal = 0;
    double bVal = 0;

    TrackingSensor fSensor(
        []() -> double {
            return (front.get() * 0.03937008) - fVal;
        },
        [](double val) {},
        []() {
            fVal = front.get() * 0.03937008;
        }
    );
    TrackingSensor bSensor(
        []() -> double {
            return (back.get()  * 0.03937008) - bVal;
        },
        [](double val) {},
        []() {
            bVal = back.get() * 0.03937008;
        }
    );

    PowerUnit flipper(
        [](double in) {
            chassis.m_fbOutputCorrect.move(-in);
        },
        []() {
            chassis.m_fbOutputCorrect.stop();
        }
    );


    PIDController fPID(fSensor, fbConstants, flipper, fbTol);
    PIDController bPID(bSensor, fbConstants, chassis.m_fbOutputCorrect, fbTol);

    PIDController thetaPIDSub90(RelativePIDHeadingTracker, thetaConstantsSub90, chassis.m_thetaOutputCorrect, thetaTolSub90);
    PIDController thetaPIDAbove90(RelativePIDHeadingTracker, thetaConstantsAbove90, chassis.m_thetaOutputCorrect, thetaTolAbove90);

    struct WhichPID {
        PIDController* operator()(double heading) {
            if (std::abs(heading) < cutoff) {
                return below;
            } else {
                return above;
            }
        }
        PIDController* below;
        PIDController* above;
        int cutoff;
    };

    WhichPID thetaPID = {&thetaPIDSub90, &thetaPIDAbove90, 90};

    double makeRelative(double heading) {
        int dir = std::signbit(heading) ? -1 : 1;
        
        heading = std::abs(heading) - headingTracker.get();
        if (heading < 0) {heading += 360;}
        if (dir < 0) {heading = -1 * (360 - heading);}

        return heading;
    }

#endif