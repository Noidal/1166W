#include "chassis.h"

HoloChassis::HoloChassis(std::vector<pros::Motor*> FL, std::vector<pros::Motor*> FR, std::vector<pros::Motor*> BL, std::vector<pros::Motor*> BR) {
    m_FL = FL;
    m_FR = FR;
    m_BL = BL;
    m_BR = BR;

    m_xOutput = PowerUnit(
        [this](double power) {m_xPower = power;},
        [this]() {m_xPower = 0;}
    );
    m_yOutput = PowerUnit(
        [this](double power) {m_yPower = power;},
        [this]() {m_yPower = 0;}
    );
    m_thetaOutput = PowerUnit(
        [this](double power) {m_thetaPower = power;},
        [this]() {m_thetaPower = 0;}
    );



    m_xOutputCorrect = PowerUnit(
        [this](double power) {m_xCorrect = power;},
        [this]() {m_xCorrect = 0;}
    );
    m_yOutputCorrect = PowerUnit(
        [this](double power) {m_yCorrect = power;},
        [this]() {m_yCorrect = 0;}
    );
    m_thetaOutputCorrect = PowerUnit(
        [this](double power) {m_thetaCorrect = power;},
        [this]() {m_thetaCorrect = 0;}
    );

    m_lVel = TrackingSensor(
        [this]() -> double {
            double xrpmSpeed = (480 / 128) * (m_xPower + m_xCorrect);
            double xipsSpeed = RPMtoIPS(xrpmSpeed);

            double yrpmSpeed = (480 / 128) * (m_yPower + m_yCorrect);
            double yipsSpeed = RPMtoIPS(yrpmSpeed);

            //std::cout << "lv: " << std::sqrt(std::pow(xipsSpeed, 2) + std::pow(yipsSpeed, 2)) << "\n";

            return std::sqrt(std::pow(xipsSpeed, 2) + std::pow(yipsSpeed, 2));
        },
        [this](double val) {
            return;
        },
        [this]() {
            return;
        }
    );
    m_lAng = TrackingSensor(
        [this]() -> double {
            double xrpmSpeed = (480 / 128) * (m_xPower + m_xCorrect);
            double xipsSpeed = RPMtoIPS(xrpmSpeed);

            double yrpmSpeed = (480 / 128) * (m_yPower + m_yCorrect);
            double yipsSpeed = RPMtoIPS(yrpmSpeed);

            double angle = (180 / M_PI) * std::atan2(xipsSpeed, yipsSpeed);
            if (angle < 0) {angle += 360;}

            return angle;
        },
        [this](double val) {
            return;
        },
        [this]() {
            return;
        }
    );
    m_aVel = TrackingSensor(
        [this]() -> double {
            double g_distBetweenWheels = 10.5;
            double maxAngVel = (RPMtoIPS(480) / (g_distBetweenWheels / 2));
            double angRadSpeed = (maxAngVel / 128) * (m_thetaPower + m_thetaCorrect);
            return angRadSpeed;
        },
        [this](double val) {
            return;
        },
        [this]() {
            return;
        }
    );

    m_xPower = 0;
    m_yPower = 0;
    m_thetaPower = 0;

    m_xCorrect = 0;
    m_yCorrect = 0;
    m_thetaCorrect = 0;

    m_xPID = NULL;
    m_yPID = NULL;
    m_hasPID = false;

    chassisTask = NULL;
}

HoloChassis::HoloChassis(std::vector<pros::Motor*> FL, std::vector<pros::Motor*> FR, std::vector<pros::Motor*> BL, std::vector<pros::Motor*> BR,
                         PIDController* xPID, PIDController* yPID) 
            : HoloChassis(FL, FR, BL, BR)
{
    m_xPID = xPID;
    m_yPID = yPID;
    m_hasPID = true;
}

void HoloChassis::addPID(PIDController* xPID, PIDController* yPID) {
    m_xPID = xPID;
    m_yPID = yPID;
    m_hasPID = true;
}

HoloChassis::~HoloChassis() {
    if (chassisTask == NULL) {
        chassisTask->remove();
    }
}

void HoloChassis::move() {
    m_xCorrect = 0;
    m_yCorrect = 0;
    m_thetaCorrect = 0;
    for (int i = 0; i < m_FL.size(); i++) {
        m_FL[i]->move(((m_xPower + m_xCorrect) + (m_yPower + m_yCorrect)) + (m_thetaPower + m_thetaCorrect));
    }
    for (int i = 0; i < m_FR.size(); i++) {
        m_FR[i]->move(((m_xPower + m_xCorrect) - (m_yPower + m_yCorrect)) - (m_thetaPower + m_thetaCorrect));
    }
    for (int i = 0; i < m_BL.size(); i++) {
        m_BL[i]->move(((m_xPower + m_xCorrect) - (m_yPower + m_yCorrect)) + (m_thetaPower + m_thetaCorrect));
    }
    for (int i = 0; i < m_BR.size(); i++) {
        m_BR[i]->move(((m_xPower + m_xCorrect) + (m_yPower + m_yCorrect)) - (m_thetaPower + m_thetaCorrect));
    }
}

void HoloChassis::brake() {
    for (int i = 0; i < m_FL.size(); i++) {
        m_FL[i]->brake();
    }
    for (int i = 0; i < m_FR.size(); i++) {
        m_FR[i]->brake();
    }
    for (int i = 0; i < m_BL.size(); i++) {
        m_BL[i]->brake();
    }
    for (int i = 0; i < m_BR.size(); i++) {
        m_BR[i]->brake();
    }
}

void HoloChassis::driverControl(pros::Controller controller, double dz) {
    m_xPower = controller.get_analog(ANALOG_RIGHT_Y);
    m_yPower = controller.get_analog(ANALOG_RIGHT_X);
    m_thetaPower = controller.get_analog(ANALOG_LEFT_X);

    if ((m_xPower < dz) && (m_xPower > -dz)) {
        m_xPower = 0;
    }
    if ((m_yPower < dz) && (m_yPower > -dz)) {
        m_yPower = 0;
    }
}

void HoloChassis::setX(double power) {
    m_xPower = power;
}

void HoloChassis::setY(double power) {
    m_yPower = power;
}

void HoloChassis::setTheta(double power) {
    m_thetaPower = power;
}

void HoloChassis::brakeMode(pros::MotorBrake type) {
    for (int i = 0; i < m_FL.size(); i++) {
        m_FL[i]->set_brake_mode(type);
    }
    for (int i = 0; i < m_FR.size(); i++) {
        m_FR[i]->set_brake_mode(type);
    }
    for (int i = 0; i < m_BL.size(); i++) {
        m_BL[i]->set_brake_mode(type);
    }
    for (int i = 0; i < m_BR.size(); i++) {
        m_BR[i]->set_brake_mode(type);
    }
}

void HoloChassis::continuousPower(void) {
    bool hasNotStarted = chassisLock.try_lock();
    if (hasNotStarted) {
        chassisTask = new pros::Task([this](){
                while (true) {
                this->move();
                pros::delay(5);
            }
        });
    }
}

void HoloChassis::moveToPoint(Point localPoint, bool nonblocking) {
    if (!m_hasPID) {
        return;
    }
    bool wasCont = !chassisLock.try_lock();
    if (!wasCont) {
        chassisLock.unlock();
        this->continuousPower();
    }
    m_xPID->movement(localPoint.x, true);
    m_yPID->movement(localPoint.y, nonblocking);
    if (!wasCont) {
        // remove chassis task
    }
    return;
}






DiffChassis::DiffChassis(std::vector<pros::Motor*> left, std::vector<pros::Motor*> right, pros::controller_analog_e_t fbInput, pros::controller_analog_e_t rotInput) {
    m_left = left;
    m_right = right;

    m_fbIn = fbInput;
    m_rotIn = rotInput;

    m_fbOutput = PowerUnit(
        [this](double power) {m_fbPower = power;},
        [this]() {m_fbPower = 0;}
    );
    m_thetaOutput = PowerUnit(
        [this](double power) {m_thetaPower = power;},
        [this]() {m_thetaPower = 0;}
    );



    m_fbOutputCorrect = PowerUnit(
        [this](double power) {m_fbCorrect = power;},
        [this]() {m_fbCorrect = 0;}
    );
    m_thetaOutputCorrect = PowerUnit(
        [this](double power) {m_thetaCorrect = power;},
        [this]() {m_thetaCorrect = 0;}
    );

    m_lVel = TrackingSensor(
        [this]() -> double {
            double xrpmSpeed = (480 / 128) * (m_fbPower + m_fbCorrect);
            double xipsSpeed = RPMtoIPS(xrpmSpeed);

            //std::cout << "lv: " << std::sqrt(std::pow(xipsSpeed, 2) + std::pow(yipsSpeed, 2)) << "\n";

            return xipsSpeed;
        },
        [this](double val) {
            return;
        },
        [this]() {
            return;
        }
    );
    m_lAng = TrackingSensor(
        [this]() -> double {
            return 0;
        },
        [this](double val) {
            return;
        },
        [this]() {
            return;
        }
    );
    m_aVel = TrackingSensor(
        [this]() -> double {
            double g_distBetweenWheels = 10.5;
            double maxAngVel = ((2 * RPMtoIPS(450)) / g_distBetweenWheels);
            double angRadSpeed = (maxAngVel / 128) * (m_thetaPower + m_thetaCorrect);
            return angRadSpeed;
        },
        [this](double val) {
            return;
        },
        [this]() {
            return;
        }
    );

    m_fbPower = 0;
    m_thetaPower = 0;

    m_fbCorrect = 0;
    m_thetaCorrect = 0;

    m_fbPID = NULL;
    m_thetaPID = NULL;
    m_hasPID = false;

    chassisTask = new pros::Task([this](){
        while (true) {
            if (this->pidStatus || this->manStatus) {
                this->move();
                pros::delay(5);
            }
        }
    });
}

DiffChassis::DiffChassis(std::vector<pros::Motor*> left, std::vector<pros::Motor*> right,
                         pros::controller_analog_e_t fbInput, pros::controller_analog_e_t rotInput,
                         PIDController* fbPID, HeadingPIDSelector* thetaPID) 
            : DiffChassis(left, right, fbInput, rotInput)
{
    m_fbPID = fbPID;
    m_thetaPID = thetaPID;
    m_hasPID = true;
}

void DiffChassis::addPID(PIDController* fbPID, HeadingPIDSelector* thetaPID) {
    m_fbPID = fbPID;
    m_thetaPID = thetaPID;
    m_hasPID = true;
}

DiffChassis::~DiffChassis() {
    if (chassisTask == NULL) {
        chassisTask->remove();
    }
}

void DiffChassis::move() {
    if (!this->pidStatus) {
        m_fbCorrect = 0;
        m_thetaCorrect = 0;
    }
    if (!this->manStatus) {
        m_fbPower = 0;
        m_thetaPower = 0;
    }
    for (int i = 0; i < m_left.size(); i++) {
        m_left[i]->move((m_fbPower + m_fbCorrect) + (m_thetaPower + m_thetaCorrect));
    }
    for (int i = 0; i < m_right.size(); i++) {
        m_right[i]->move((m_fbPower + m_fbCorrect) - (m_thetaPower + m_thetaCorrect));
    }

    std::cout << "left: " << (m_fbPower + m_fbCorrect) + (m_thetaPower + m_thetaCorrect) << "\n";
    std::cout << "right: " << (m_fbPower + m_fbCorrect) - (m_thetaPower + m_thetaCorrect) << "\n\n";
}

void DiffChassis::move_relative(double distance, int speed, bool nonblocking) {
    this->continuousPower(false, false);

    distance = (distance / (M_PI * 3.25)) * 360;

    int initialLDist = m_left[0]->get_position();
    int initialRDist = m_right[0]->get_position();

    for (int i = 0; i < m_left.size(); i++) {
        m_left[i]->move_relative(distance, speed);
    }
    for (int i = 0; i < m_right.size(); i++) {
        m_right[i]->move_relative(distance, speed);
    }
    waitUntil(((std::abs(m_left[0]->get_position() - initialLDist) >= std::abs(distance)) && (std::abs(m_left[0]->get_position() - initialLDist) >= std::abs(distance))) || nonblocking);
}

void DiffChassis::brake() {
    m_fbCorrect = 0;
    m_thetaCorrect = 0;
    for (int i = 0; i < m_left.size(); i++) {
        m_left[i]->brake();
    }
    for (int i = 0; i < m_right.size(); i++) {
        m_right[i]->brake();
    }
}

void DiffChassis::driverControl(pros::Controller controller, double dz) {
    this->continuousPower(false, true);
    m_fbPower = controller.get_analog(m_fbIn);
    m_thetaPower = controller.get_analog(m_rotIn);

    if ((m_fbPower < dz) && (m_fbPower > -dz)) {
        m_fbPower = 0;
    } else {
        (m_fbPower > 0) ? m_fbPower -= 15 : m_fbPower += 15;
        m_fbPower *= 1.13;
    }
    if ((m_thetaPower < dz) && (m_thetaPower > -dz)) {
        m_thetaPower = 0;
    } else {
        (m_fbPower > 0) ? m_fbPower -= 15 : m_fbPower += 15;
        m_thetaPower *= 1.13;
    }
}

void DiffChassis::setFB(double power) {
    m_fbCorrect = power;
}

void DiffChassis::setTheta(double power) {
    m_thetaCorrect = power;
}

void DiffChassis::brakeMode(pros::MotorBrake type) {
    for (int i = 0; i < m_left.size(); i++) {
        m_left[i]->set_brake_mode(type);
    }
    for (int i = 0; i < m_right.size(); i++) {
        m_right[i]->set_brake_mode(type);
    }
}

void DiffChassis::continuousPower(bool pidStatus, bool manStatus) {this->pidStatus = pidStatus; this->manStatus = manStatus;}

void DiffChassis::moveToPoint(Pose current, Point goal, bool turn, bool nonblocking, bool reverse) {
    int rev = reverse ? -1 : 1;
    int dir = 1;
    if (!m_hasPID) {
        return;
    }
    this->continuousPower(true, false);
    double goalHead = findHeadingOfLine({current.x, current.y}, goal);
    if (reverse) {
        goalHead += 180;
        if (goalHead > 360) {goalHead -= 360;}
    }
    if (makeRelative(current.heading, dir * goalHead) > 180) {
        dir = -1;
    }
    std::cout << makeRelative(current.heading, dir * goalHead) << "\n";
    if (turn) {
        m_thetaPID->operator()(makeRelative(current.heading, dir * goalHead))->movement(makeRelative(current.heading, dir * goalHead));
    }
    if ((goal.x < 80) && (goal.y < 80)) {
        m_fbPID->movement(rev * calculateDistance({current.x, current.y}, goal), nonblocking);
    }
    return;
}

void DiffChassis::enablePID(void) {this->continuousPower(true, false);}