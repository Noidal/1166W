#include "pid.h"

PIDController::PIDController(TrackingSensor pidSensor, ConstantContainer constants, PowerUnit output, double tolerance) {
    m_sensor = pidSensor;
    m_constants = constants;
    m_output = output;
    m_tolerance = tolerance;
    m_movementTask = NULL;
}

PIDController::~PIDController() {
    if (m_movementTask != NULL) {
        m_movementTask->remove();
    }
}

void PIDController::movement(
    double setPoint, // goal coordinate position
    bool nonblocking, // defaults to false- explicitly set to true to make controller run as task

    std::vector<std::function<void(void)>> customs, // a lambda function that will execute during the PID (optional)
    std::vector<double> executeAts // the distance point (in inches) that you want to trigger the custom lambda function at (optional)
    )
{
    bool canRun = movementLock.try_lock();
    if (!canRun) {
        return;
    }
    if (nonblocking) {
        movementLock.unlock();
        m_movementTask = new pros::Task([this, setPoint, customs, executeAts](){this->movement(setPoint, false, customs, executeAts);});
        return;
    }

    m_sensor.reset();

    // PID Calculation Variables
    // General Variables
    double power = 0;

    std::vector<bool> customsCompleted(customs.size(), false);
    bool actionCompleted = false;

    double remainingDistance = setPoint;

    // Odometry Pre-Measurement

    // used to measure the rotational sensor values of all the motors (this comes in degrees)
    double currentDistanceMovedByWheel = m_sensor.get();

    // this initializes variables that are used to measure values from previous cycles
    p_error = setPoint - currentDistanceMovedByWheel;
    p_integral = 0;



    while (actionCompleted != true) {

        // gets the power for the current cycle
        power = this->calculateOutput(currentDistanceMovedByWheel, setPoint);

        // reverses the direction if the robot has passed the set point
        /* if (std::abs(currentDistanceMovedByWheel) > std::abs(setPoint)) {
            power *= -1;
        } */

        // moves the wheels at the desired power, ending the cycle
        m_output.move(power);



    // Custom lambda function that will execute if given and the robot has reached the point given by executeAt
            
        for (int i = 0; i < customs.size(); i++) {
            // ensures that the code will only run if the function has been provided and if executeAt has been reached
            if (customs[i] != 0 && (currentDistanceMovedByWheel >= executeAts[i]) && !customsCompleted[i]) {
                // runs the function
                customs[i]();
                // prevents the function from running again
                customsCompleted[i] = true;
            }
        }

        // fifteen millisecond delay between cycles
        pros::delay(15);

        // find new sensor location
        currentDistanceMovedByWheel = m_sensor.get();

        // applies any new mods
        if (m_modFresh) {
            setPoint = m_setPointMod;
            m_modFresh = false;
        }

        // calculates the distance moved as the difference between the distance left to move
        // and the total distance to move
        remainingDistance = setPoint - currentDistanceMovedByWheel;

        // checks to see if the robot has completed the movement by checking if it is within a range of the perpendicular line of its goal point
        if ((remainingDistance <= 0 + m_tolerance) &&
           (remainingDistance >= 0 - m_tolerance)) {
            actionCompleted = true;
            m_output.stop();
            movementLock.unlock();
        }
        
}

}

void PIDController::modMovement(double setPoint) {
    m_setPointMod = setPoint;
    m_modFresh = true;
}

void PIDController::stopMovement(void) {
    if (m_movementTask != NULL) {
        m_movementTask->remove();
    }
}

double PIDController::calculateOutput(
	double distanceMoved, // current distance moved (in odometry units)
	double setPoint // goal distance to move (in odometry units)
	)
{
	// P: Proportional -- slows down as we reach our target for more accuracy

		// error = goal reading - current reading
		double error = setPoint - distanceMoved;
		// kP (proportional constant) determines how fast we want to go overall while still keeping accuracy
		double proportionalOut = error * m_constants.kP;

//        std::cout << "error: " << error << "\n";


	// I: Integral -- starts slow and speeds up as time goes on to prevent undershooting

		// starts the integral at the error, then compounds it with the new current error every loop
		double integral = p_integral + error;
		// prevents the integral variable from causing the robot to overshoot
		if ((((error <= 0)) && (p_error > 0)) || 
			(((error >= 0)) && (p_error < 0))) {
			integral = 0;
		}
		// prevents the integral from winding up too much, causing the number to be beyond the control of
        // even kI
		// if we want to make this better, see Solution #3 for 3.3.2 in the packet
		/* if (((setPoint > 0) && (error >= 100)) || ((setPoint < 0) && (error <= -100))) {
			integral = 0;
			}
		*/

		// kI (integral constant) brings integral down to a reasonable/useful output number
		double integralOut = integral * m_constants.kI;

        if ((integral > 100) || (integral < -100)) {
			integral = integral > 100
				? 100
				: -100;
		}

		// adds integral to return structure for compounding
		p_integral = integral;

//        std::cout << "integral: " << integralOut << "\n";



	// D: Derivative -- slows the robot more and more as it goes faster

        // starts the derivative by making it the rate of change from the previous cycle to this one
        double derivative = error - p_error;

        // kD (derivative constant) prevents derivative from over- or under-scaling
        double derivativeOut = (derivative * m_constants.kD) / ((pros::millis() - p_time) / 15);

		// sets the previous error to the current error for use in the next derivative
		p_error = error;

        // tracks the time of this check so that the next check will not overdo the derivative 
        // due to a large change over a large amount of time
        p_time = pros::millis();

//        std::cout << "derivative: " << derivativeOut << "\n\n";

	// Adds the results of each of the calculations together to get the desired power
		double power = proportionalOut + integralOut + derivativeOut;
        // std::cout << "p = " << proportionalOut << ", i = " << integralOut << ", d = " << derivativeOut << "\n";

	// returns a PIDReturn structure
		return power;
}