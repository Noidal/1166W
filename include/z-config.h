#ifndef _PROS_CONFIG_H_
#define _PROS_CONFIG_H_

#include "main.h"
#include "chassis.h"
#include "interface.h"
#include "odom.h"
#include "pid.h"
#include "spline.h"
#include "follow.h"
#include "profile.h"
#include "kalman.h"
#include "math.h"
#include "mcl.h"

// MASON SCHEME
#define FB_INPUT pros::E_CONTROLLER_ANALOG_RIGHT_Y
#define ROT_INPUT pros::E_CONTROLLER_ANALOG_LEFT_X

#define INTAKE_ALL_IN pros::E_CONTROLLER_DIGITAL_R2
#define INTAKE_ALL_OUT pros::E_CONTROLLER_DIGITAL_L1
#define INTAKE_HOLD pros::E_CONTROLLER_DIGITAL_L2
#define INTAKE_DROP pros::E_CONTROLLER_DIGITAL_R1

#define LOADER_TOGGLE pros::E_CONTROLLER_DIGITAL_Y
#define RAMP_TOGGLE pros::E_CONTROLLER_DIGITAL_RIGHT
#define COLOR_TOGGLE pros::E_CONTROLLER_DIGITAL_A
#define FINGER_TOGGLE pros::E_CONTROLLER_DIGITAL_B


// autonnumber key goes here
int autonnumber = 0;

// variant key goes here
int variant = 0;

// all possible start positions go here
Pose startPose = {0, 0, 0};

ConstantContainer fbConstants = {4, 0.1, 2.7};
ConstantContainer thetaConstantsSub90 = {1.05, 0.15, 8};
ConstantContainer thetaConstantsAbove90 = {0.7, 0.1, 32};

double fbTol = 1;
double thetaTolSub90 = 2.5;
double thetaTolAbove90 = 3.5;

#define FRONT_LEFT -18
#define BACK_LEFT -17
#define HALF_LEFT -20

#define FRONT_RIGHT 8
#define BACK_RIGHT 9
#define HALF_RIGHT 19

// #define intake

#define PARA_ODOM 10
#define PERP_ODOM 5

#define FRONT_DIST 99
#define RIGHT_DIST 99
#define LEFT_DIST 99

#define INERTIAL_A 16
#define INERTIAL_B 7

#define COLOR 10

// #define pistons

#define ODOM_DIAMETER 2

// Controllers
    pros::Controller master(pros::E_CONTROLLER_MASTER);

// Motors
    // Differential Drivetrain
        pros::Motor frontLeft(FRONT_LEFT, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor backLeft(BACK_LEFT, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor halfLeft(HALF_LEFT, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);

        pros::Motor frontRight(FRONT_RIGHT, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor backRight(BACK_RIGHT, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor halfRight(HALF_RIGHT, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);
        
    // Intake Motors

    // Sensors
        pros::Rotation parallelTrack(PARA_ODOM);
        pros::Rotation perpTrack(PERP_ODOM);

        pros::Distance front(FRONT_DIST);
        pros::Distance left(LEFT_DIST);
        pros::Distance right(RIGHT_DIST);

        pros::IMU inertial1(INERTIAL_A);
        pros::IMU inertial2(INERTIAL_B);

        pros::Optical color(COLOR);

    // Three-Wire Devices
        

// Program Module Initialization

    DiffChassis chassis = DiffChassis({&frontLeft, &backLeft, &halfLeft}, {&frontRight, &backRight, &halfRight}, FB_INPUT, ROT_INPUT);

    OdomPod mainOdom(&parallelTrack, ODOM_DIAMETER);
    TrackingSensor fbTrack(
        []() -> double {
            return mainOdom.measure();
        },
        [](double val) {
            parallelTrack.set_position(val);
            return;
        },
        []() {
            parallelTrack.reset_position();
            return;
        }
    );
    double distAtLastReset = 0;
    TrackingSensor PIDfbTrack(
        []() -> double {
            return mainOdom.measure() - distAtLastReset;
        },
        [](double val) {
            return;
        },
        []() {
            distAtLastReset = mainOdom.measure() / 2;
            return;
        }
    );
    TrackingSensor angVelTracker(
        []() -> double {
            return (mainOdom.measureVelocity() / 5);
        }
    );

    OdomPod perpendicularOdom = OdomPod(&perpTrack, ODOM_DIAMETER);
    TrackingSensor lrTrack(
        []() -> double {
            return perpendicularOdom.measure();
        },
        [](double val) {
            perpTrack.set_position(val);
            return;
        },
        []() {
            perpTrack.reset_position();
            return;
        }
    );

    KalmanFilter Kalman1 = KalmanFilter(&inertial1, angVelTracker);
    KalmanFilter Kalman2 = KalmanFilter(&inertial2, angVelTracker);

    TrackingSensor headingTracker(
        []() -> double {
            return inertial1.get_heading();
        }
    );

    double distFromLastReset = 0;
    double lastResetHead = 0;
    double lastHeading = startPose.heading;
    TrackingSensor PIDHeadingTracker(
        []() -> double {
            double changeInHeading = inertial1.get_heading() - lastHeading;
            if (changeInHeading > 315) {
                changeInHeading -= 360;
            } else if (changeInHeading < -315) {
                changeInHeading += 360;
            }
            distFromLastReset += changeInHeading;
            lastHeading = inertial1.get_heading();

            return distFromLastReset;
        },
        [](double val) {
            return;
        },
        []() {
            distFromLastReset = 0;
            lastResetHead = inertial1.get_heading();
            lastHeading = inertial1.get_heading();
            return;
        }
    );

    Odometry odom(fbTrack, headingTracker, startPose);

    PIDController fbPID(PIDfbTrack, fbConstants, chassis.m_fbOutputCorrect, fbTol);
    PIDController thetaPIDSub90(PIDHeadingTracker, thetaConstantsSub90, chassis.m_thetaOutputCorrect, thetaTolSub90);
    PIDController thetaPIDAbove90(PIDHeadingTracker, thetaConstantsAbove90, chassis.m_thetaOutputCorrect, thetaTolAbove90);

    // PIDSet robotPIDs(&xPID, &yPID, &thetaPIDSub90, &thetaPIDAbove90);
    PoseTracker currentPose(&odom);

    TrackingSensor linAngleTracker(
        []() -> double {
            double offsetFromHeading = chassis.m_lAng.get() < 180 ? chassis.m_lAng.get() : chassis.m_lAng.get() - 360;
            return headingTracker.get() + offsetFromHeading;
        }
    );

    HeadingPIDSelector thetaPID = {&thetaPIDSub90, &thetaPIDSub90, 100};

    void manualTurn(double heading, double range) {
        waitUntil((inertial2.get_heading() > heading - (range / 2)) && (inertial2.get_heading() < heading + (range / 2)));
    }


    // VelocityController follower(&chassis.m_xOutput, &chassis.m_yOutput, &chassis.m_thetaOutput, &currentPose, robotPIDs);

#endif