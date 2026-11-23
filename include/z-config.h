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
#include "subsystems.h"

// MASON SCHEME
#define FB_INPUT pros::E_CONTROLLER_ANALOG_RIGHT_Y
#define ROT_INPUT pros::E_CONTROLLER_ANALOG_LEFT_X

#define INTAKE_IN pros::E_CONTROLLER_DIGITAL_
#define TOGGLE_ON pros::E_CONTROLLER_DIGITAL_L1
#define TOGGLE_SLOW_ON pros::E_CONTROLLER_DIGITAL_DOWN

#define INTAKE_HOLD pros::E_CONTROLLER_DIGITAL_
#define INTAKE_DROP pros::E_CONTROLLER_DIGITAL_
#define TOGGLE_WATCH pros::E_CONTROLLER_DIGITAL_UP

#define CASCADE_UP pros::E_CONTROLLER_DIGITAL_R2
#define CASCADE_DOWN pros::E_CONTROLLER_DIGITAL_R1

#define CLAW_ROTATE_UP pros::E_CONTROLLER_DIGITAL_Y
#define CLAW_ROTATE_DOWN pros::E_CONTROLLER_DIGITAL_B

#define CLAW_CLOSE_TOGGLE pros::E_CONTROLLER_DIGITAL_L2
#define CLAW_FLIP_TOGGLE pros::E_CONTROLLER_DIGITAL_RIGHT


// autonnumber key goes here
int autonnumber = 1;

// variant key goes here
int variant = 0;

// all possible start positions go here
// Pose startPose = {-58.5, 9.5, 45};
Pose startPose = {-62, -1, 28};

ConstantContainer fbConstants = {4, 0.1, 2.7};
ConstantContainer thetaConstantsSub90 = {2.5, 0.1, 11};
ConstantContainer thetaConstantsAbove90 = {1.6, 0.1, 9};

double fbTol = 1;
double thetaTolSub90 = 2.5;
double thetaTolAbove90 = 3.5;

#define FRONT_LEFT -19 // -5
#define BACK_LEFT -20 // -17
#define HALF_LEFT -99 // -20

#define FRONT_RIGHT 18 // 8
#define BACK_RIGHT 17 // 9
#define HALF_RIGHT 99 // 13

#define INTAKE_TOGGLE 8

#define CASCADE_LEFT -9
#define CASCADE_RIGHT 10

#define CLAW_ROTATE 4

#define PARA_ODOM -13
#define PERP_ODOM 14

#define FRONT_DIST 99
#define RIGHT_DIST 99
#define LEFT_DIST 99

#define INERTIAL_A 15 // closer to brain
#define INERTIAL_B 7

#define COLOR 6
#define ROTATION 16
#define PIN_SENSOR 3

#define CLAW_CLOSE 2
#define CLAW_FLIP 3

#define ODOM_DIAMETER 2

// Controllers
    pros::Controller master(pros::E_CONTROLLER_MASTER);

// Motors
    // Differential Drivetrain
        pros::Motor frontLeft(FRONT_LEFT, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor backLeft(BACK_LEFT, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);

        pros::Motor frontRight(FRONT_RIGHT, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor backRight(BACK_RIGHT, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        
    // Manipulation Motors
        pros::Motor intakeToggle(INTAKE_TOGGLE, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);

        pros::Motor leftCascade(CASCADE_LEFT, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor rightCascade(CASCADE_RIGHT, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);

        pros::Motor clawRotate(CLAW_ROTATE, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);
        int clawState = 0;

    // Sensors
        pros::Rotation parallelTrack(PARA_ODOM);
        pros::Rotation perpTrack(PERP_ODOM);

        /* pros::Distance front(FRONT_DIST);
        pros::Distance left(LEFT_DIST);
        pros::Distance right(RIGHT_DIST); */

        pros::IMU inertial1(INERTIAL_A);
        pros::IMU inertial2(INERTIAL_B);

        pros::Optical toggleSensor(COLOR);
        pros::Rotation cascadeSensor(ROTATION);
        pros::Distance pinSensor(PIN_SENSOR);

    // Three-Wire Devices
        pros::adi::DigitalOut clawClose(CLAW_CLOSE, false);
        pros::adi::DigitalOut clawFlip(CLAW_FLIP, false);
        

// Program Module Initialization

    DiffChassis chassis = DiffChassis({&frontLeft, &backLeft}, {&frontRight, &backRight}, FB_INPUT, ROT_INPUT);

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
            distAtLastReset = mainOdom.measure();
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
            return inertial2.get_heading();
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

    Odometry odom(fbTrack, headingTracker, startPose, lrTrack);
    
    PIDController fbPID(PIDfbTrack, fbConstants, chassis.m_fbAutoIn, fbTol);
    PIDController thetaPIDSub90(PIDHeadingTracker, thetaConstantsSub90, chassis.m_thetaAutoIn, thetaTolSub90);
    PIDController thetaPIDAbove90(PIDHeadingTracker, thetaConstantsAbove90, chassis.m_thetaAutoIn, thetaTolAbove90);

    // PIDSet robotPIDs(&xPID, &yPID, &thetaPIDSub90, &thetaPIDAbove90);
    PoseTracker currentPose(&odom);

    TrackingSensor linAngleTracker(
        []() -> double {
            double offsetFromHeading = chassis.m_lAng.get() < 180 ? chassis.m_lAng.get() : chassis.m_lAng.get() - 360;
            return headingTracker.get() + offsetFromHeading;
        }
    );

    HeadingPIDSelector thetaPID = {&thetaPIDSub90, &thetaPIDAbove90, 90};

    void manualTurn(double heading, double range) {
        waitUntil((inertial2.get_heading() > heading - (range / 2)) && (inertial2.get_heading() < heading + (range / 2)));
    }

    PhysicalConstraints robot(600, 0.75, 3.25, 13, 3.100, 0.9, 0.075);
    VelocityController follower(&chassis.m_directLeftIn, &chassis.m_directRightIn, &currentPose, robot);

    PowerUnit cascadePower(
        [](double power) {leftCascade.move(power); rightCascade.move(power);},
        []() {leftCascade.brake(); rightCascade.brake();}
    );
    TrackingSensor cascadePosition(
        []() -> double {
            return cascadeSensor.get_position() / 100;
        },
        [](double val) {},
        []() {}
    );
    TrackingSensor cascadeSpeed(
        []() -> double {
            return cascadeSensor.get_velocity();
        },
        [](double val) {},
        []() {}
    );

    CascadeLift cascade(cascadePower, cascadeSpeed, cascadePosition, 60, 20, 1000, 0);


    PowerUnit clawPower(
        [](double power) {clawRotate.move(power);},
        []() {clawRotate.brake();}
    );
    TrackingSensor clawPosition(
        []() -> double {
            return clawRotate.get_position() / 100;
        },
        [](double val) {},
        []() {}
    );

    Claw claw(clawPower, clawPosition);

#endif