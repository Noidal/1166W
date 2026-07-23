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
	
	odom.updateLoop();
	// mcl.start();
	color.set_led_pwm(100);
	master.print(0, 0, "Initialized!");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

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
	
	chassis.powerAccess(false, false, true);
	chassis.brakeMode(pros::v5::MotorBrake::hold);

//hif
	// autonomous setup

	int time = 0;
	int startTime = pros::millis();

	/* pros::Task item([] () {while (true) {		pros::lcd::print(0, "x = %f", currentPose.get().x);
		pros::lcd::print(1, "y = %f", currentPose.get().y);
		pros::lcd::print(2, "h = %f", currentPose.get().heading);  pros::delay(50);}}); */

	CubicHermiteSpline test = CubicHermiteSpline({0, 0}, {1.5, 114}, {30.5, 0}, {77, 37.5});
	MotionProfile testP = MotionProfile(&test, robot, 0.5);
	follower.followProfile(&testP, false, true);

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

	/*
	chassis.powerAccess(true, false, false);
	chassis.brake();
	chassis.brakeMode(pros::v5::MotorBrake::coast); */

	int deadzone = 15;
	int startTime = pros::millis();
	double pvel = 0;
	double cvel = 0;

	while (true) {
		
		pros::lcd::print(0, "x = %f", currentPose.get().x);
		pros::lcd::print(1, "y = %f", currentPose.get().y);
		pros::lcd::print(2, "h1 = %f", inertial1.get_heading());
		pros::lcd::print(3, "h2 = %f", inertial2.get_heading()); 
		
		std::cout << "{" << currentPose.get().x << ", " << currentPose.get().y << "}\n";
	// Differential Drive Control
		chassis.driverControl(master, deadzone);

	pros::delay(50);
	}
}