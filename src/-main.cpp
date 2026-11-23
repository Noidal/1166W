#include "z-config.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::delay(3000);
	pros::lcd::initialize();
	inertial1.set_heading(startPose.heading);
	inertial2.set_heading(startPose.heading);
    // Kalman1.startFilter();
    // Kalman2.startFilter();
	
	parallelTrack.reset_position();
	perpTrack.reset_position();
	odom.updateLoop();
	// mcl.start();
	toggleSensor.set_led_pwm(100);
	master.print(0, 0, "Initialized!");
	cascadeSensor.reset_position();

	chassis.addPID(&fbPID, &thetaPID);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	clawClose.set_value(true);
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	
	chassis.powerAccess(false, true, true);
	chassis.brakeMode(pros::v5::MotorBrake::hold);

//hif
	// autonomous setup

	int time = 0;
	int startTime = pros::millis();

	/* pros::Task item([] () {while (true) {		pros::lcd::print(0, "x = %f", currentPose.get().x);
		pros::lcd::print(1, "y = %f", currentPose.get().y);
		pros::lcd::print(2, "h = %f", currentPose.get().heading);  pros::delay(50);}}); */

	// CubicHermiteSpline test = CubicHermiteSpline({0, 0}, {0, 114}, {30.5, 37.5}, {77, 37.5});
	// CubicHermiteSpline test = CubicHermiteSpline({0, 0}, {0, 114}, {0, 37.5}, {0, 114});
	// MotionProfile testP = MotionProfile(&test, robot, 0.1);
	// follower.followProfile(&testP, false, false);
	// chassis.powerAccess(false, false, false);
	/*
	if (std::abs(autonnumber) == 1) {

				clawRotate.move_relative(50, 200);
				fbPID.movement(15);
				chassis.m_directLeftIn.move(128);
				chassis.m_directRightIn.move(128);
				pros::delay(300);
				chassis.powerAccess(false, true, false);
				clawClose.set_value(true);

				fbPID.movement(-8);
				clawRotate.move_relative(-50, 200);
				thetaPIDAbove90.movement(90);
				fbPID.movement(10);
				clawClose.set_value(false);
				rightCascade.move_relative(3000, 600);
				leftCascade.move_relative(3000, 600);
				fbPID.movement(-8);
				thetaPIDAbove90.movement(-90);
				clawRotate.move_relative(40, 200);

				fbPID.movement(9);
				chassis.powerAccess(false, true, true);
				chassis.m_directLeftIn.move(128);
				chassis.m_directRightIn.move(128);
				pros::delay(500);
				chassis.powerAccess(false, true, false);
				pros::delay(200);
				clawClose.set_value(true);

				fbPID.movement(-9);
				thetaPIDSub90.movement(-30);
				fbPID.movement(25);
				thetaPIDSub90.movement(-50);
				chassis.powerAccess(false, true, true);
				chassis.m_directLeftIn.move(600);
				chassis.m_directRightIn.move(600);
				pros::delay(2000);
				chassis.powerAccess(false, true, false);
				fbPID.movement(-10);

				} else if (std::abs(autonnumber) == 2) {

				clawRotate.move_relative(50, 200);
				fbPID.movement(15);
				chassis.m_directLeftIn.move(128);
				chassis.m_directRightIn.move(128);
				pros::delay(300);
				chassis.powerAccess(false, true, false);
				clawClose.set_value(true);

				fbPID.movement(-8);
				clawRotate.move_relative(-50, 200);
				thetaPIDAbove90.movement(-90);
				fbPID.movement(10);
				clawClose.set_value(false);
				rightCascade.move_relative(3000, 600);
				leftCascade.move_relative(3000, 600);
				fbPID.movement(-8);
				thetaPIDAbove90.movement(90);
				clawRotate.move_relative(40, 200);

				fbPID.movement(9);
				chassis.powerAccess(false, true, true);
				chassis.m_directLeftIn.move(128);
				chassis.m_directRightIn.move(128);
				pros::delay(500);
				chassis.powerAccess(false, true, false);
				pros::delay(200);
				clawClose.set_value(true);

				fbPID.movement(-9);
				thetaPIDSub90.movement(30);
				fbPID.movement(25);
				thetaPIDSub90.movement(50);
				chassis.powerAccess(false, true, true);
				chassis.m_directLeftIn.move(600);
				chassis.m_directRightIn.move(600);
				pros::delay(2000);
				chassis.powerAccess(false, true, false);
				fbPID.movement(-10);
	}
	*/
	if (std::abs(autonnumber) == 1) {
		intakeToggle.move(-128);
		pros::delay(250);
		chassis.moveToPoint(currentPose.get(), {-52.5, 18}, false);
		chassis.powerAccess(false, false, true);
		chassis.m_directLeftIn.move(300);
		chassis.m_directRightIn.move(300);
		pros::delay(300);
		chassis.powerAccess(false, true, false);
		pros::delay(300);
		clawClose.set_value(true);
		chassis.moveToPoint(currentPose.get(), {-57, 11}, false, false, true);

		thetaPIDSub90.movement(makeRelative(currentPose.get().heading, -335));
		chassis.powerAccess(false, false, true);
		chassis.m_directLeftIn.move(150);
		chassis.m_directRightIn.move(400);
		pros::delay(1000);
		chassis.powerAccess(false, true, false);
		pros::delay(300);
		clawClose.set_value(false);
		pros::delay(200);

		rightCascade.move_relative(2500, 600);
		leftCascade.move_relative(2500, 600);
		thetaPIDSub90.movement(makeRelative(currentPose.get().heading, 70));
		pros::delay(200);
		chassis.m_directLeftIn.move(200);
		chassis.m_directRightIn.move(200);
		chassis.powerAccess(false, false, true);
		clawRotate.move_relative(80, 200);
		pros::delay(700);
		chassis.powerAccess(false, true, false);
		pros::delay(200);
		clawClose.set_value(true);

		chassis.moveToPoint(currentPose.get(), {-54, 11}, false, false, true);
		rightCascade.move_relative(-2500, 600);
		leftCascade.move_relative(-2500, 600);
		chassis.moveToPoint(currentPose.get(), {-57, 35});
		clawRotate.move_relative(-80, 200);
		thetaPIDSub90.movement(makeRelative(currentPose.get().heading, 35));

		pros::delay(200);
		chassis.powerAccess(false, false, true);
		chassis.m_directLeftIn.move(200);
		chassis.m_directRightIn.move(200);
		PIDfbTrack.reset();
		waitUntil(pinSensor.get() < 140 || PIDfbTrack.get() > 10);
		chassis.powerAccess(false, true, false);
		clawClose.set_value(false);
		pros::delay(300);


		rightCascade.move_relative(4000, 600);
		leftCascade.move_relative(4000, 600);
		chassis.moveToPoint(currentPose.get(), {-55, 35});
		clawRotate.move_relative(100, 200);
		chassis.moveToPoint(currentPose.get(), {-50, 26});
		chassis.powerAccess(false, false, true);
		chassis.m_directLeftIn.move(300);
		chassis.m_directRightIn.move(300);
		clawClose.set_value(true);

	}


	while (true) {
		pros::delay(100000);
	}
	switch (autonnumber) {

	}
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	master.rumble("-.-");
	clawClose.set_value(true);
	// switch drivetrain into driver control configuration
	/*
	chassis.powerAccess(true, false, false);
	chassis.brake();
	chassis.brakeMode(pros::v5::MotorBrake::coast); */

	int deadzone = 15;
	int startTime = pros::millis();

	double cascadeSpeed = 0;
	double cascadeIncrement = 0;

	bool flipStatus = false;
	bool closeStatus = false;
	bool autoClaw = false;
	bool autoCascade = false;

	bool wrongToggleColor = false;

	leftCascade.set_brake_mode(pros::v5::MotorBrake::hold);
	rightCascade.set_brake_mode(pros::v5::MotorBrake::hold);

	clawRotate.set_brake_mode(pros::v5::MotorBrake::hold);

	chassis.powerAccess(true, false, false);

	while (true) {

		autoCascade = cascade.autoRun;
		
	// Odometry Diagnostics
		pros::lcd::print(0, "x = %f", currentPose.get().x);
		pros::lcd::print(1, "y = %f", currentPose.get().y);
		pros::lcd::print(2, "h1 = %f", inertial1.get_heading());
		pros::lcd::print(3, "h2 = %f", inertial2.get_heading()); 
		// std::cout << "{" << currentPose.get().x << ", " << currentPose.get().y << "}\n";

	// Differential Drive Control
		chassis.driverControl(master, deadzone);

	// Intake/Toggle Watcher
		if (master.get_digital(TOGGLE_WATCH) && 
			((((autonnumber < 0) && (toggleSensor.get_hue() > 100)) || 
			((autonnumber > 0) && (toggleSensor.get_hue() < 30))))) {
				wrongToggleColor = true;
		} else {
				wrongToggleColor = false;
		}

	// Intake/Toggle Control
		if (master.get_digital(TOGGLE_ON) || (wrongToggleColor)) {
			intakeToggle.move(-128);
		} /* else if (master.get_digital(INTAKE_IN)) {
			intakeToggle.move(128);
		} */ else if (master.get_digital(TOGGLE_SLOW_ON)) {
			intakeToggle.move(-15);
		} else {
			intakeToggle.brake();
		}

	// Cascade Control
	if (!autoCascade) {
		if (master.get_digital(CASCADE_UP)) {
			// std::cout << clawRotate.get_position() << "\n";
			autoClaw = true;
			if ((cascadePosition.get() > 800) && (clawRotate.get_position() < 180)) {
				clawRotate.move(50);
			} else {
				autoClaw = false;
			}
			cascade.targetSpeed(128);
		} else if (master.get_digital(CASCADE_DOWN) && ((cascadeSensor.get_position()) && ((clawState > -1) || (cascadeSensor.get_position() > -10)))) {
			autoClaw = true;
			if ((cascadePosition.get() < 50) && (clawRotate.get_position() > 15)) {
				clawRotate.move(-50);
			} else {
				autoClaw = false;
			}
			cascade.targetSpeed(-128);
		} else {
			cascade.targetSpeed(0);
		}
	}
	
	// Claw Rotation Control
		/* if (master.get_digital_new_press(CLAW_ROTATE_UP)) {
			claw.setState(claw.state >= 2 ? claw.state : claw.state + 1);
		} else if (master.get_digital_new_press(CLAW_ROTATE_DOWN)) {
			claw.setState(claw.state <= 1 ? claw.state : claw.state - 1);
		}

		if (cascadeSensor.get_position() > 600) {
			claw.setState(2);
		}

		claw.move(); */

		if (master.get_digital(CLAW_ROTATE_UP)) {
			clawRotate.move(50);
		} else if (master.get_digital(CLAW_ROTATE_DOWN)) {
			clawRotate.move(-50);
		} else if (!autoClaw) {
			clawRotate.brake();
		}

	// Claw Piston Close
		if (master.get_digital_new_press(CLAW_CLOSE_TOGGLE)) {
			std::cout << closeStatus << "\n";
			clawClose.set_value(!closeStatus);
			closeStatus = !closeStatus;
		}
	
	// Claw Piston Flip
		if (master.get_digital_new_press(CLAW_FLIP_TOGGLE)) {
			clawFlip.set_value(!flipStatus);
			flipStatus = !flipStatus;
		}

	pros::delay(50);
	}
}