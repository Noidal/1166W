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

// MASON SCHEME
/*
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
*/

// DANE SCHEME
#define FB_INPUT pros::E_CONTROLLER_ANALOG_RIGHT_Y
#define ROT_INPUT pros::E_CONTROLLER_ANALOG_LEFT_X

#define INTAKE_ALL_IN pros::E_CONTROLLER_DIGITAL_R1
#define INTAKE_ALL_OUT pros::E_CONTROLLER_DIGITAL_L1
#define INTAKE_HOLD pros::E_CONTROLLER_DIGITAL_R2
#define INTAKE_DROP pros::E_CONTROLLER_DIGITAL_A

#define LOADER_TOGGLE pros::E_CONTROLLER_DIGITAL_L2
#define RAMP_TOGGLE pros::E_CONTROLLER_DIGITAL_DOWN
#define COLOR_TOGGLE pros::E_CONTROLLER_DIGITAL_Y
#define FINGER_TOGGLE pros::E_CONTROLLER_DIGITAL_RIGHT


// 1 = low goal
// 2 = high goal
// 3 = Solo AWP
// 4 = 7 long, low side
// 5 = low elim
// 666 = auto skills
int autonnumber = 5;

// 0 = standard
// 1 = replace autos 1 and 2 with finger push end
int variant = 0;

// low goal
// Pose startPose = {-47, -17, 180};
// high goal
// Pose startPose = {-47, 17, 0};
// sawp high->low
// Pose startPose = {-45.5, 1.5, 180};
// 7 long, low side
// Pose startPose = {-46, -15, 90};
// low elim
Pose startPose = {-44, -7, 131};
// auto skills
// Pose startPose = {-47, -17, 180};
// Pose startPose = {29, -47.5, 90};

ConstantContainer fbConstants = {4, 0.1, 2.7};
ConstantContainer thetaConstantsSub90 = {1.05, 0.15, 8};
ConstantContainer thetaConstantsAbove90 = {0.7, 0.1, 32};

double fbTol = 1;
double thetaTolSub90 = 2.5;
double thetaTolAbove90 = 3.5;

#define FRONT_LEFT -13
#define MID_LEFT -12
#define BACK_LEFT 11

#define FRONT_RIGHT 18
#define MID_RIGHT 19
#define BACK_RIGHT -20

#define R_INPUT 3
#define STORAGE 2
#define R_OUTPUT 1

#define LEFT_ODOM 4
#define RIGHT_ODOM 5
#define PERP_ODOM 99

#define FRONT_DIST 99
#define RIGHT_DIST 99
#define LEFT_DIST 99

#define INERTIAL_A 14
#define INERTIAL_B 16

#define COLOR 10

#define RAMP 1
#define LOADER 2
#define SEXJOKE 3

#define ODOM_DIAMETER 2

// Controllers
    pros::Controller master(pros::E_CONTROLLER_MASTER);

// Motors
    // Differential Drivetrain
        
        pros::Motor topLeft(FRONT_LEFT, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor midLeft(MID_LEFT, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor backLeft(BACK_LEFT, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);

        pros::Motor topRight(FRONT_RIGHT, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor midRight(MID_RIGHT, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor backRight(BACK_RIGHT, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        
    // Intake
        pros::Motor stage1(R_INPUT, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor stage2(STORAGE, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor stage3(R_OUTPUT, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);

    // Sensors
        pros::Rotation parallelLeftOdom(LEFT_ODOM);
        pros::Rotation parallelRightOdom(RIGHT_ODOM);
        pros::Rotation perpOdom(PERP_ODOM);

        pros::Distance front(FRONT_DIST);
        pros::Distance left(LEFT_DIST);
        pros::Distance right(RIGHT_DIST);

        pros::IMU inertial1(INERTIAL_A);
        pros::IMU inertial2(INERTIAL_B);

        pros::Optical color(COLOR);

    // Three-Wire Devices
        pros::adi::DigitalOut ramp(RAMP);
        pros::adi::DigitalOut loader(LOADER);
        pros::adi::DigitalOut sexjoke(SEXJOKE);

// Program Module Initialization

    DiffChassis chassis = DiffChassis({&topLeft, &midLeft, &backLeft}, {&topRight, &midRight, &backRight}, FB_INPUT, ROT_INPUT);

    OdomPod leftOdom(&parallelLeftOdom, ODOM_DIAMETER);
    OdomPod rightOdom(&parallelRightOdom, ODOM_DIAMETER);
    TrackingSensor fbTrack(
        []() -> double {
            return ((-leftOdom.measure() + rightOdom.measure()) / 2);
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
    double distAtLastReset = 0;
    TrackingSensor PIDfbTrack(
        []() -> double {
            return ((-leftOdom.measure() + rightOdom.measure()) / 2) - distAtLastReset;
        },
        [](double val) {
            return;
        },
        []() {
            distAtLastReset = ((-leftOdom.measure() + rightOdom.measure()) / 2);
            return;
        }
    );
    TrackingSensor angVelTracker(
        []() -> double {
            return ((leftOdom.measureVelocity() - rightOdom.measureVelocity()) / 5);
        }
    );

    OdomPod perpendicularOdom = OdomPod(&perpOdom, ODOM_DIAMETER);
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