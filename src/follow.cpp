#include "follow.h"

VelocityController::VelocityController(PowerUnit* lOutput, PowerUnit* rOutput, PoseTracker* globalPos, PhysicalConstraints robot) {
    this->lOutput = lOutput;
    this->rOutput = rOutput;
    this->globalPos = globalPos;

    this->robot = robot;
}

// (private)
void VelocityController::followProfile(MotionProfile* currentlyFollowing, bool reverse, bool RAMSETE)
{
    // step-related variables
    double currentStep = 0;
    double step = 1 / (double) currentlyFollowing->profile.size();
    double pointID = 0;
    // point variables
    MPPoint currentPoint = {0, 0, 0, 0, 0, 0};
    MPPoint nextPoint = {0, 0, 0, 0, 0, 0};
    // speed variables
    std::vector<double> velocitiesRPM = {0, 0};
    // clock variables
    double startTime = pros::millis();
    // action variables
    std::vector<bool> actionCompleteds = {false, false, false, false, false, false};


    // control loop
    while (true) {

        // calculates the current point as the nearest point to the current step
        double elapsedTime = pros::millis() - startTime;
        for (; pointID < currentlyFollowing->profile.size(); pointID++) {
            if (!(elapsedTime > currentlyFollowing->profile[pointID].timeAtPoint)) {
                break;
            }
        }
        currentPoint = currentlyFollowing->profile[pointID];

        // sets linear and angular velocities to that of the current point - these are changed by RAMSETE if it is on
        double linVel = currentPoint.linVel;
        double angVel = currentPoint.angVel;


        // calculation of output of each side with error corrections from RAMSETE
        if (RAMSETE) {
            Pose location = {this->globalPos->get().x * 0.0254, this->globalPos->get().y * 0.0254, this->globalPos->get().heading};
            nextPoint = currentlyFollowing->profile[pointID + 1];
            nextPoint = {nextPoint.x * 0.0254, nextPoint.y * 0.0254, nextPoint.heading};

            if (reverse) {
                linVel *= -1;
                if (location.heading > 180) {
                    location.heading -= 180;
                } else {
                    location.heading += 180;
                }
            }

            double fixedOdomAngle = fixAngle(location.heading) * (M_PI / 180);
            double fixedNextAngle = fixAngle(nextPoint.heading) * (M_PI / 180);

            std::cout << "head = " << location.heading << "fixed = " << fixedNextAngle << "\n";



        //textToWrite.push_back("odom = " + std::to_string(location.heading) + ", fodom = " + std::to_string(fixedOdomAngle) + "\n");
        //textToWrite.push_back("next = " + std::to_string(nextPoint.heading) + ", fnext = " + std::to_string(fixedNextAngle) + "\n\n");
        // rotation of the x, y, and heading errors to fit the local frame
            Pose error;
            error.x = (std::cos(fixedOdomAngle) * (nextPoint.x - location.x)) + (std::sin(fixedOdomAngle) * (nextPoint.y - location.y));
            if (reverse) {error.x *= -1;}
            error.y = (std::cos(fixedOdomAngle) * (nextPoint.y - location.y)) - (std::sin(fixedOdomAngle) * (nextPoint.x - location.x));
            error.heading = fixedNextAngle - fixedOdomAngle;
            // std::cout << error.heading << "\n";

            linVel *= 0.0254;

            // bounds the error from 0-180 to prevent the correction from being an un-optimal turn direction (where going the other way would be faster)
            if (error.heading < -M_PI) {
                error.heading = error.heading + (2 * M_PI);
            } else if (error.heading > M_PI) {
                error.heading = error.heading - (2 * M_PI);
            }

            //std::cout << "prp = " << fixedNextAngle << ", ap = " << fixedOdomAngle << ", c = " << error.heading << "\n";
            //std::cout << "ucl = " << location.heading << ", actual = " << getAggregatedHeading(Kalman1, Kalman2) << ", used = " << fixedOdomAngle << "\n";

        // tuning constants (current values are from the widely accepted defaults from FTCLib)
            double b = 2.0; // this is a proportional gain for each of the different error elements of the controller (put into the gain value calculations)
            double zeta = 0.7; // this is a dampener for the direct movements (k1 and k3/x-value and heading-value)

        // gain values that serve as scaling multipliers for the outputs based on the profile's velocity and pre-set constants
            // k1/k3: the proportional gain value for both the local frame of x and heading
            double k = 2 * zeta * std::sqrt(std::pow(angVel, 2) + (b * std::pow(linVel, 2)));
            // k2: the gain value for the y-value, which the robot cannot move directly on and is thus handled differently 
            double k2 = b * linVel;

        // control inputs that determine the controller's influence on the velocity based on the error
            // simply the normal gain value multiplied by the x-error because that can be directly moved upon (input for linear velocity)
            double u1 = k * error.x;
            // the special gain value is used for the y-value, and it is also scaled with the part in purple parenthesis to let it switch directions smoothly if it is past
            // its goal point; the second part is the same simple direct angular movement for the error in heading that is used in the same way with the x-value for linear
            // movement
            double u2 = (k2 * (std::sin(error.heading) / error.heading) * error.y) + (k * error.heading);

        // actual calculations of modified linear and angular velocities as additions/subtractions to the profile's original values
            // the original linear velocity is also transformed to follow the robot's error in heading before having the control input subtracted from it
            linVel = (linVel * std::cos(error.heading)) + u1;
            // the angular velocity does not need to be transformed in the same way that the linear velocity needs to because it is already angular in reference to the robot
            angVel = angVel + u2;

            linVel *= 39.37008;
        }

        // standard calculation of output of each side based on specifications of the motion profile
        velocitiesRPM = robot.outputOfSides(linVel, angVel);

        // executes custom actions if the profile has reached or passed their t-point and have not yet been activated
        for (int i = 0; i < actions.size(); i++) {
            if (((currentPoint.t >= actionTs[i])) && !actionCompleteds[i]) {
                actions[i]();
                actionCompleteds[i] = true;
            }
        }
        
        // converts the velocity in rpm to velocity in millivolts
        int maxVoltage = 12000; // innate max voltage of motors (in mV)
        double rpmToV = maxVoltage / 600; // multiplier to convert rpm to voltage (in units of millivoltage / rpm so multiplying it by rpm cancels to millivoltage)

        // sends the output voltage to the motors
        if (reverse && !RAMSETE) {
            lOutput->move(127 * (-velocitiesRPM[1] / 600));
            rOutput->move(127 * (-velocitiesRPM[0] / 600));
        } else {
            lOutput->move(127 * (velocitiesRPM[0] / 600));
            rOutput->move(127 * (velocitiesRPM[1] / 600));
        }
        
        // 5 ms delay (- the time taken to calculate)
        pros::delay(5);

        // if the current step is the final point (t = 1 - step), 
        // then the drivetrain is stopped and the function ends
        if (currentPoint.t == currentlyFollowing->profile[currentlyFollowing->profile.size() - 1].t) {
            return;
        }
    }
}

// starts the filter loop if it is not already active (public)
void VelocityController::startProfile(MotionProfile* profile, bool correct) {

    // auto controlLoopFunction = [this, path, RAMSETE] () {return this->followProfile(*this->queuedProfile, path, RAMSETE);};

    // if (controlLoop_task_ptr == NULL) {
        // pros::Task* controlLoop_task_ptr = new pros::Task(controlLoopFunction);
        this->followProfile(profile, correct);
    // }
}

void VelocityController::addAction(std::function<void(void)> action, double time) {
    double actionT = time;
    this->actions.push_back(action);
    this->actionTs.push_back(actionT);
}

void VelocityController::clearActions(void) {
    this->actions.clear();
    this->actionTs.clear();
    this->actionCompleteds = {false, false, false, false, false, false};
}
