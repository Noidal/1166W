#include "z-config-r2.h"

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
	parallelLeftOdom.set_position(0);
	parallelRightOdom.set_position(0);
	perpOdom.set_position(0);
	odom.updateLoop();
	chassis.addPID(&fbPID, &thetaPID);
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
	chassis.continuousPower(true, false);
	chassis.brakeMode(pros::v5::MotorBrake::hold);

//hif
	// autonomous setup

	stage1.tare_position();
	stage2.tare_position();
	stage3.tare_position();

	int time = 0;
	int startTime = pros::millis();
	int adjDir = 1;
	pros::Task stopOut([] () {
		if (autonnumber > 1) {
			waitUntil(color.get_hue() > 100);
		} else {
			waitUntil(color.get_hue() < 20);
		}
		pros::delay(50);
		stage3.brake();
	});
	pros::Task* delayedLoaderDown = NULL;
	stopOut.suspend();

	pros::Task item([] () {while (true) {		pros::lcd::print(0, "x = %f", currentPose.get().x);
		pros::lcd::print(1, "y = %f", currentPose.get().y);
		pros::lcd::print(2, "h = %f", currentPose.get().heading);  pros::delay(50);}});

	switch (autonnumber) {
		case 1:
		case -1:
			chassis.moveToPoint(odom.m_robotPose, {-47, -43}, false);
			thetaPID.operator()(makeRelative(odom.m_robotPose.heading, 270))->movement(makeRelative(odom.m_robotPose.heading, 270));
			
			// move to Loader and intake from Loader
			loader.set_value(true);
			pros::delay(500);
			stage1.move(-128);
			stage2.move(-128);
			chassis.move_relative(14, 200, false);
			pros::delay(1000);
			
			// move to Long Goal and score all prior blocks in goal
			chassis.moveToPoint(odom.m_robotPose, {-35, -43}, false, false, true);
			chassis.setFB(-50);
			stage3.move(-128);
			time = pros::millis();
			if (autonnumber < 1) {
				waitUntil((color.get_hue() > 100) || (pros::millis() - time > 2000));
			} else {
				waitUntil((color.get_hue() < 20) || (pros::millis() - time > 2000));
			}
			pros::delay(100);
			stage3.brake();
			chassis.moveToPoint(odom.m_robotPose, {-36, -47}, false);
			if (autonnumber < 1) {
				if (color.get_hue() > 100) {stage3.move(-128);}
			} else {
				if (color.get_hue() < 20) {stage3.move(-128);}
			}
			stopOut.resume();

			// wing push variant
			if (variant == 1) {
				loader.set_value(false);
				sexjoke.set_value(true);
				chassis.moveToPoint(odom.m_robotPose, {-36, -60}, true, false, true);
				sexjoke.set_value(false);
				chassis.moveToPoint(odom.m_robotPose, {-12, -62}, true, false, true);
				break;
			}

			// grab mid-Blocks across field, then score into low goal
			loader.set_value(false);
			chassis.moveToPoint(odom.m_robotPose, {-22, -22});
			loader.set_value(true);
			chassis.move_relative(5, 200, false);
			stage3.brake();
			loader.set_value(false);
			pros::delay(300);
			chassis.moveToPoint(odom.m_robotPose, {-9, -9}, true);
			stage1.move(64);
			stage2.move(64);
			stage3.move(64);
			pros::delay(3000);
			chassis.move_relative(-5, 200, false);
			break;



		case 2:
		case -2:

			chassis.moveToPoint(odom.m_robotPose, {-47, 43}, false);
			thetaPID.operator()(makeRelative(odom.m_robotPose.heading, -275))->movement(makeRelative(odom.m_robotPose.heading, -275));
			
			// move to Loader and intake from Loader
			loader.set_value(true);
			pros::delay(700);
			stage1.move(-128);
			stage2.move(-128);
			chassis.move_relative(14, 200, false);
			pros::delay(1000);
			
			// move to Long Goal and score all prior blocks in goal
			chassis.moveToPoint(odom.m_robotPose, {-35, 43}, false, false, true);
			chassis.setFB(-50);
			stage3.move(-128);
			time = pros::millis();
			if (autonnumber < 1) {
				waitUntil((color.get_hue() > 100) || (pros::millis() - time > 2000));
			} else {
				waitUntil((color.get_hue() < 20) || (pros::millis() - time > 2000));
			}
			pros::delay(100);
			stage3.brake();
			chassis.moveToPoint(odom.m_robotPose, {-38, 47}, false);
			if (autonnumber < 1) {
				if (color.get_hue() > 100) {stage3.move(-128);}
			} else {
				if (color.get_hue() < 20) {stage3.move(-128);}
			}
			stopOut.resume();
			
			if (variant == 1) {
				loader.set_value(false);
				sexjoke.set_value(true);
				chassis.moveToPoint(odom.m_robotPose, {-36, 34}, true, false, true);
				sexjoke.set_value(false);
				chassis.moveToPoint(odom.m_robotPose, {-12, 32}, true, false, true);
			}
			
			// grab mid-Blocks across field, then score into low goal
			loader.set_value(false);
			chassis.moveToPoint(odom.m_robotPose, {-22, 22});
			loader.set_value(true);
			chassis.move_relative(5, 200, false);
			pros::delay(300);
			stage3.brake();
			chassis.moveToPoint(odom.m_robotPose, {-11, 9}, true, false, true);
			ramp.set_value(true);
			stage1.move(-128);
			stage2.move(-128);
			stage3.move(-128);
			pros::delay(1500);
			chassis.move_relative(5, 200, false);
			break;



		case 3:
		case -3:
			stage1.move(-128);
			stage2.move(-128);
			chassis.move_relative(7, 100, false);
			chassis.moveToPoint(odom.m_robotPose, {-47, 43}, false, false, true);
			thetaPID.operator()(makeRelative(odom.m_robotPose.heading, 270))->movement(makeRelative(odom.m_robotPose.heading, 270));
			
			// move to Loader and intake from Loader
			loader.set_value(true);
			pros::delay(500);
			stage1.move(-128);
			stage2.move(-128);
			chassis.move_relative(15.5, 200, false);
			chassis.setFB(50);
			pros::delay(550);
			
			// move to Long Goal and score all prior blocks in goal
			chassis.moveToPoint(odom.m_robotPose, {-31, 43}, false, false, true);
			chassis.setFB(-50);
			stage3.move(-128);
			time = pros::millis();
			if (autonnumber < 1) {
				waitUntil((color.get_hue() > 100) || (pros::millis() - time > 1050));
			} else {
				waitUntil((color.get_hue() < 20) || (pros::millis() - time > 1050));
			}
			pros::delay(100);
			stage3.brake();
			chassis.moveToPoint(odom.m_robotPose, {-36, 47}, false);
			if (autonnumber < 1) {
				if (color.get_hue() > 100) {stage3.move(-128);}
			} else {
				if (color.get_hue() < 20) {stage3.move(-128);}
			}
			stopOut.resume();
			
			
			// grab mid-Blocks across field, then score into low goal
			loader.set_value(false);
			chassis.moveToPoint(odom.m_robotPose, {-20, 22});
			stage3.brake();
			loader.set_value(true);
			chassis.move_relative(5, 200, false);
			pros::delay(200);
			chassis.moveToPoint(odom.m_robotPose, {-10, 11}, true, false, true);
			stage1.move(-128);
			stage2.move(-128);
			// stage3.move(-128);
			ramp.set_value(true);
			pros::delay(100);
			// stage3.brake();
			chassis.move_relative(10, 200, false);

			
			// back up, get other blocks
			stage1.move(-128);
			stage2.move(-128);
			stage3.brake();
			loader.set_value(false);
			ramp.set_value(false);
			chassis.moveToPoint(odom.m_robotPose, {-32, -48}, true, false, false);
			thetaPID.operator()(makeRelative(odom.m_robotPose.heading, 280))->movement(makeRelative(odom.m_robotPose.heading, 280));
			chassis.move_relative(-30, 300);
			// chassis.moveToPoint(odom.m_robotPose, {-30, -47}, true, false, true);
			pros::delay(300);
			stage3.move(-128);
			chassis.setFB(-50);
			pros::delay(3000);
			
			break;
		case 4:
		case -4:
			// intake center blocks
			stage1.move(-128);
			stage2.move(-128);
			chassis.moveToPoint(odom.m_robotPose, {-22.5, -21}, true, false, false);
			loader.set_value(true);
			pros::delay(100);

			// get blocks from loader
			chassis.moveToPoint(odom.m_robotPose, {-47, -47.5}, true);
			thetaPID.operator()(makeRelative(odom.m_robotPose.heading, 280))->movement(makeRelative(odom.m_robotPose.heading, 280));
			
			// move to Loader and intake from Loader
			loader.set_value(true);
			pros::delay(500);
			stage1.move(-128);
			stage2.move(-128);
			chassis.move_relative(15, 200, false);
			pros::delay(1000);
			
			// move to Long Goal and score all prior blocks in goal
			chassis.moveToPoint(odom.m_robotPose, {-33, -46}, false, false, true);
			chassis.setFB(-50);
			stage3.move(-128);
			time = pros::millis();
			if (autonnumber < 1) {
				waitUntil((color.get_hue() > 100) || (pros::millis() - time > 2000));
			} else {
				waitUntil((color.get_hue() < 20) || (pros::millis() - time > 2000));
			}
			pros::delay(100);
			stage3.brake();
			chassis.moveToPoint(odom.m_robotPose, {-36, -47}, false);
			if (autonnumber < 1) {
				if (color.get_hue() > 100) {stage3.move(-128);}
			} else {
				if (color.get_hue() < 20) {stage3.move(-128);}
			}
			stopOut.resume();

			// finger push into control zone
			loader.set_value(false);
			sexjoke.set_value(true);
			chassis.moveToPoint(odom.m_robotPose, {-36, -58.5}, true, false, true);
			sexjoke.set_value(false);
			chassis.moveToPoint(odom.m_robotPose, {-15, -60.5}, true, false, true);
			// waitUntil(pros::millis() - startTime > 14500);
			// sexjoke.set_value(true);
			break;

		case 5:
		case -5:
			// grab mid-Blocks across field, then score into low goal
			stage1.move(-128);
			stage2.move(-128);
			loader.set_value(false);
			delayedLoaderDown = new pros::Task([]() {
				pros::delay(1000);
				loader.set_value(true);
			});
			chassis.moveToPoint(odom.m_robotPose, {-28, -27}, false);
			loader.set_value(true);
			chassis.move_relative(1.5, 200, false);
			pros::delay(300);
			stage1.brake();
			stage2.brake();
			stage3.brake();
			loader.set_value(false);
			chassis.moveToPoint(odom.m_robotPose, {-14, -15}, true);
			adjDir = (makeRelative(odom.m_robotPose.heading, adjDir * 45) < 180) ? adjDir : -adjDir;
			thetaPID.operator()(makeRelative(odom.m_robotPose.heading, adjDir * 45))->movement(makeRelative(odom.m_robotPose.heading, adjDir * 45));
			stage1.move(80);
			stage2.move(80);
			stage3.move(80);
			pros::delay(1000);
			// chassis.move_relative(-5, 200, false);
			stage1.move(-128);
			stage2.move(-128);
			stage3.brake();

			// back into Loader territory
			chassis.moveToPoint(odom.m_robotPose, {-40, -43}, true, false, true);
			thetaPID.operator()(makeRelative(odom.m_robotPose.heading, -275))->movement(makeRelative(odom.m_robotPose.heading, -275));

			// move to Loader and intake from Loader
			loader.set_value(true);
			pros::delay(500);
			stage1.move(-128);
			stage2.move(-128);
			chassis.move_relative(30, 200, false);
			pros::delay(700);
			
			// move to Long Goal and score all prior blocks in goal
			chassis.moveToPoint(odom.m_robotPose, {-34, -43}, false, false, true);
			chassis.setFB(-50);
			stage3.move(-128);
			time = pros::millis();
			if (autonnumber < 1) {
				waitUntil((color.get_hue() > 100) || (pros::millis() - time > 1200));
			} else {
				waitUntil((color.get_hue() < 20) || (pros::millis() - time > 1200));
			}
			pros::delay(100);
			stage3.brake();
			chassis.moveToPoint(odom.m_robotPose, {-40, -47}, false);
			if (autonnumber < 1) {
				if (color.get_hue() > 100) {stage3.move(-128);}
			} else {
				if (color.get_hue() < 20) {stage3.move(-128);}
			}
			stopOut.resume();

			// wing push
			loader.set_value(false);
			sexjoke.set_value(true);
			chassis.moveToPoint(odom.m_robotPose, {-31, -58}, true, false, true);
			adjDir = (makeRelative(odom.m_robotPose.heading, adjDir * (180 + findHeadingOfLine({odom.m_robotPose.x, odom.m_robotPose.y}, {-15, -61}))) < 180) ? adjDir : -adjDir;
			thetaPID.operator()(makeRelative(odom.m_robotPose.heading, adjDir * (180 + findHeadingOfLine({odom.m_robotPose.x, odom.m_robotPose.y}, {-15, -61}))))->movement(makeRelative(odom.m_robotPose.heading, adjDir * (180 + findHeadingOfLine({odom.m_robotPose.x, odom.m_robotPose.y}, {-15, -61}))));
			sexjoke.set_value(false);
			chassis.moveToPoint(odom.m_robotPose, {-17, -61}, false, false, true);
			break;

		case 666:
			
			sexjoke.set_value(true);
			chassis.moveToPoint(odom.m_robotPose, {-47, -43}, false);
			thetaPID.operator()(makeRelative(odom.m_robotPose.heading, 270))->movement(makeRelative(odom.m_robotPose.heading, 270));
		
			// move to Loader and intake from Loader
			loader.set_value(true);
			pros::delay(500);
			stage1.move(-128);
			stage2.move(-128);
			chassis.move_relative(14, 200, false);
			pros::delay(1500);
			
			// move to Long Goal and score all prior blocks in goal
			chassis.moveToPoint(odom.m_robotPose, {-45, -43}, false, false, true);
			loader.set_value(false);
			chassis.moveToPoint(odom.m_robotPose, {-36, -30}, true, false, true);
			chassis.moveToPoint(odom.m_robotPose, {26, -17}, true, false, true);

			chassis.moveToPoint(odom.m_robotPose, {27, -48}, true);
			thetaPID.operator()(makeRelative(odom.m_robotPose.heading, -85))->movement(makeRelative(odom.m_robotPose.heading, -85));

			// move to Loader and intake from Loader
			chassis.setFB(-80);
			pros::delay(500);
			stage3.move(-128);
			time = pros::millis();
			waitUntil((pros::millis() - time > 1000));
			stage3.move(128);
			pros::delay(150);
			stage3.move(-128);
			waitUntil((pros::millis() - time > 2000));
			pros::delay(100);
			stage3.brake();

			odom.m_robotPose = {29, -47.5, 90};
			
			// score 2 electric boogaloo
			loader.set_value(true);
			stage1.move(-128);
			stage2.move(-128);
			chassis.setFB(0);
			chassis.move_relative(10, 300, false);
			chassis.moveToPoint(odom.m_robotPose, {45, -48}, false);
			pros::delay(150);
			chassis.move_relative(14, 200, false);
			pros::delay(2000);
			chassis.move_relative(-37, 300, false);
			chassis.setFB(-80);
			stage3.move(-128);
			pros::delay(3000);
			loader.set_value(false);

			// cross field
			chassis.moveToPoint(odom.m_robotPose, {40, -48}, false);
			chassis.moveToPoint(odom.m_robotPose, {40, 90}, true);
			chassis.moveToPoint(odom.m_robotPose, {40, 0}, false);
			chassis.moveToPoint(odom.m_robotPose, {40, 47}, false);
			chassis.move_relative(17, 200, false);
			chassis.enablePID();
			chassis.setFB(50);
			pros::delay(500);
			chassis.setFB(0);
			pros::delay(200);
			chassis.move_relative(-19, 200, false);
			chassis.enablePID();
			loader.set_value(true);
			thetaPID.operator()(80)->movement(80);
			loader.set_value(true);
			stage1.move(-128);
			stage2.move(-128);
			chassis.move_relative(27.5, 300, false);
			pros::delay(1500);
			
			// move to Long Goal and score all prior blocks in goal
			chassis.continuousPower(true, false);
			chassis.setFB(-80);
			pros::delay(1000);
			stage3.move(-128);
			time = pros::millis();
			waitUntil((pros::millis() - time > 1000));
			stage3.move(128);
			pros::delay(150);
			stage3.move(-128);
			waitUntil((pros::millis() - time > 2000));
			pros::delay(100);
			
			break;



		case 10:
			stage1.move(-128);
			stage2.move(-128);
			stage3.move(-128);
			chassis.move_relative(-60, 450, false);

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

	chassis.continuousPower(false, true);
	chassis.brake();
	chassis.brakeMode(pros::v5::MotorBrake::coast);

	int deadzone = 15;

	int colorDelay = 0;
	int colorEnabled = 0;

	master.print(0, 0, "color = %d", colorEnabled);

	while (true) {
		pros::lcd::print(0, "x = %f", currentPose.get().x);
		pros::lcd::print(1, "y = %f", currentPose.get().y);
		pros::lcd::print(2, "h1 = %f", inertial1.get_heading());
		pros::lcd::print(3, "h2 = %f", inertial2.get_heading()); /*
		std::cout << "{" << currentPose.get().x << ", " << currentPose.get().y << "}\n"; */
	// Differential Drive Control
		chassis.driverControl(master, deadzone);


		// std::cout << currentPose.get().heading << "\n";
	// Intake Control
	// Long Goal
	if (master.get_digital(INTAKE_ALL_IN)) {
		stage1.move(-128);
		stage2.move(-128);
		stage3.move(-128);
		// ramp.set_value(true);
	} 
	// High Center Goal
	else if (master.get_digital(INTAKE_DROP)) {
		stage1.move(-128);
		stage2.move(-128);
		stage3.move(-128);
		ramp.set_value(false);
	}
	// stage2
	else if (master.get_digital(INTAKE_HOLD)) {
		stage1.move(-128);
		stage2.move(-128);
		stage3.brake();
	}
	// Low Center Goal
	else if (master.get_digital(INTAKE_ALL_OUT)) {
		stage1.move(128);
		stage2.move(128);
		stage3.move(128);
	}
	else {
		stage1.brake();
		stage2.brake();
		stage3.brake();
	}
	// color sort
	if (((master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) || (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) 
		|| (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) || (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)))
		&&
		(((colorEnabled == 1) && (color.get_hue() > 60)) // remove blue
		|| ((colorEnabled == 2) && (color.get_hue() < 20)))) // remove red
	{
		colorDelay = 10;
	}
	if (colorDelay > 0) {
		colorDelay--;
		stage1.move(128);
		if (colorDelay > 5) {stage2.move(-128);}
		stage3.move(-128);
	}


	// color sort toggle
	if (master.get_digital_new_press(COLOR_TOGGLE)) {
		colorEnabled++;
		if (colorEnabled > 2) {colorEnabled = 0;}

		switch (colorEnabled) {
			case 0:
				master.print(0, 0, "remove none");
			case 1:
				master.print(0, 0, "remove blue");
			case 2:
				master.print(0, 0, "remove rhed");

		}
	}

	// Ramp Mech
	if (master.get_digital_new_press(RAMP_TOGGLE)) {
		ramp.set_value(!ramp.get_value());
	}

	// Loader Mech
	if (master.get_digital_new_press(LOADER_TOGGLE)) {
		loader.set_value(!loader.get_value());
	}

	// Finger Mech
	if (master.get_digital_new_press(FINGER_TOGGLE)) {
		sexjoke.set_value(!sexjoke.get_value());
	}

	pros::delay(20);
	}
}