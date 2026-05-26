#ifndef _KALMAN_H_
#define _KALMAN_H_

#include <deque>

#include "main.h"
#include "interface.h"
#include "math.h"

class KalmanFilter {
    private:
        // instance variables
        pros::IMU* inertial; // defined in constructor
        TrackingSensor angVelReader; // defined in constructor
        pros::Task* filterLoop_ptr; // is initialized when the Kalman filter turns on

        double filteredHeading; // updates as Kalman filter runs
        double filterUncertainty; // updates as Kalman filter runs

        int delay; // time between cycles

        std::deque<double> measurementVariances; // list of all measurement variances from the estimate
        std::deque<double> predictionVariances; // list of all prediction variances from the estimate


        // internal methods
        void KalmanFilterLoop(void); // actual filter


    public:
        // public methods
        KalmanFilter(pros::IMU* inertial, TrackingSensor angVelReader); // constructor

        // return methods for the filter, updated constantly as the filter runs
        double getFilteredHeading(void);
        double getFilterUncertainty(void);

        // start and stop methods for the filter
        void startFilter(void);
        void endFilter(void);
};

double getAggregatedHeading(KalmanFilter inertial1, KalmanFilter inertial2);

#endif