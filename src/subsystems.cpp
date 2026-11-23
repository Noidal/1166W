#include "subsystems.h"

CascadeLift::CascadeLift(PowerUnit output, TrackingSensor speedTracker, TrackingSensor positionTracker,
                        double incrementUp, double incrementDown, 
                        double maxHeight, double minHeight)
    : output(output), speedTracker(speedTracker), positionTracker(positionTracker), incrementUp(incrementUp), incrementDown(incrementDown), maxHeight(maxHeight), minHeight(minHeight) 
{
    targetPosition = minHeight;
    cascadeTask = new pros::Task([this]() {
        while (true) {
            if (autoRun) {this->move();}
            pros::delay(15);
        }
    });
}

void CascadeLift::targetSpeed(int targetSpeed) {
    if (targetSpeed > this->speed) {
	    speed += incrementUp;
    } else if (targetSpeed < this->speed) {
        speed -= incrementDown;
    }
    std::cout << targetSpeed << "\n";

    if ((std::abs(targetSpeed - speed) <= std::abs(incrementUp)) || (std::abs(targetSpeed - speed) <= std::abs(incrementDown))) {
        speed = targetSpeed;
    }

    if (!(((speed < 0) && (positionTracker.get() < minHeight)) || ((speed > 0) && (positionTracker.get() > maxHeight)))) {
        output.move(speed);
    } else {
        output.move(0);
    }
}

void CascadeLift::moveTo(int targetPosition) {
    this->targetPosition = targetPosition;
    autoRun = true;
}

void CascadeLift::move() {
    int position = positionTracker.get();
    int direction = 0;

    if (targetPosition > this->maxHeight) {targetPosition = this->maxHeight;}
    if (targetPosition < this->minHeight) {targetPosition = this->minHeight;}

    std::cout << "p = " << position << ", tp = " << targetPosition << "\n";
    if (std::abs(targetPosition - position) > 10) {
        direction = (targetPosition - position) > 0 ? 1 : -1;
        this->targetSpeed(direction * 128);
        } else {
        this->targetSpeed(0);
        autoRun = false;
    }
}

Claw::Claw(PowerUnit output, TrackingSensor posTracker)
    : output(output), posTracker(posTracker) {}

void Claw::setState(int newState) {this->state = newState;}

void Claw::move() {
    if (std::abs((state * 90) - posTracker.get()) > 3) {
		output.move(((((state * 90) - posTracker.get()) > 0) ? 1 : -1) * 5);
	} else {
        output.stop();
    }
}

void Claw::updateState() {
    while (std::abs((state * 90) - posTracker.get()) > 3) {
		output.move(((((state * 90) - posTracker.get()) > 0) ? 1 : -1) * 5);
        pros::delay(15);
	}
    output.stop();
}