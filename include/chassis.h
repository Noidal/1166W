#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include <vector>

#include "main.h"
#include "interface.h"
#include "pid.h"
#include "math.h"

class HoloChassis {
    public:
        HoloChassis(std::vector<pros::Motor*> FL, std::vector<pros::Motor*> FR, std::vector<pros::Motor*> BL, std::vector<pros::Motor*> BR);
        HoloChassis(std::vector<pros::Motor*> FL, std::vector<pros::Motor*> FR, std::vector<pros::Motor*> BL, std::vector<pros::Motor*> BR,
                    PIDController* xPID, PIDController* yPID);
        void addPID(PIDController* xPID, PIDController* yPID);
        ~HoloChassis();

        void move(void);
        void brake(void);
        void brakeMode(pros::MotorBrake type);
        void driverControl(pros::Controller controller, double dz);
        void continuousPower(void);
        void moveToPoint(Point localPoint, bool nonblocking = false);

        void setX(double power);
        void setY(double power);
        void setTheta(double power);


        PowerUnit m_xOutput;
        PowerUnit m_xOutputCorrect;

        PowerUnit m_yOutput;
        PowerUnit m_yOutputCorrect;

        PowerUnit m_thetaOutput;
        PowerUnit m_thetaOutputCorrect;

        TrackingSensor m_lVel;
        TrackingSensor m_lAng;
        TrackingSensor m_aVel;

    private:
        std::vector<pros::Motor*> m_FL;
        std::vector<pros::Motor*> m_FR;
        std::vector<pros::Motor*> m_BL;
        std::vector<pros::Motor*> m_BR;

        double m_xPower;
        double m_xCorrect;

        double m_yPower;
        double m_yCorrect;

        double m_thetaPower;
        double m_thetaCorrect;

        PIDController* m_xPID;
        PIDController* m_yPID;
        bool m_hasPID;

        pros::Mutex chassisLock;
        pros::Task* chassisTask;
};

class DiffChassis {
    public:
        DiffChassis(std::vector<pros::Motor*> left, std::vector<pros::Motor*> right, 
                    pros::controller_analog_e_t fbInput, pros::controller_analog_e_t rotInput);
        DiffChassis(std::vector<pros::Motor*> left, std::vector<pros::Motor*> right,
                    pros::controller_analog_e_t fbInput, pros::controller_analog_e_t rotInput,
                    PIDController* fbPID, HeadingPIDSelector* thetaPID);
        void addPID(PIDController* fbPID, HeadingPIDSelector* thetaPID);
        ~DiffChassis();

        void move(void);
        void move_relative(double distance, int speed, bool nonblocking = true);
        void brake(void);
        void brakeMode(pros::MotorBrake type);
        void driverControl(pros::Controller controller, double dz);
        void powerAccess(bool driver, bool auton, bool lr);
        void moveToPoint(Pose current, Point goal, bool turn = true, bool nonblocking = false, bool reverse = false);
        double feedforward(double velocity, int side);
        void enablePID(void);

        void setFB(double power);
        void setTheta(double power);

        pros::controller_analog_e_t m_fbIn;
        pros::controller_analog_e_t m_rotIn;

        PowerUnit m_fbDriverIn;
        PowerUnit m_fbAutoIn;

        PowerUnit m_thetaDriverIn;
        PowerUnit m_thetaAutoIn;

        PowerUnit m_directLeftIn;
        PowerUnit m_directRightIn;

        TrackingSensor m_lVel;
        TrackingSensor m_lAng;
        TrackingSensor m_aVel;

    private:
        std::vector<pros::Motor*> m_left;
        std::vector<pros::Motor*> m_right;

        double m_fbDriver;
        double m_fbAuto;

        double m_thetaDriver;
        double m_thetaAuto;

        double m_directLeft;
        double m_directRight;

        PIDController* m_fbPID;
        HeadingPIDSelector* m_thetaPID;
        bool m_hasPID;

        pros::Mutex chassisLock;
        bool driverEnabled;
        bool autoEnabled;
        bool lrEnabled;
        pros::Task* chassisTask;
};

#endif