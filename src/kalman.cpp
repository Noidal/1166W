#include "kalman.h"

double getAggregatedHeading(KalmanFilter inertial1, KalmanFilter inertial2) {
    // gets heading from each of the sensors
    double I1Heading = inertial1.getFilteredHeading();
    double I2Heading = inertial2.getFilteredHeading();

    // gets the uncertainty from each of the IMUs
    double I1Uncertainty = inertial1.getFilterUncertainty();
    double I2Uncertainty = inertial2.getFilterUncertainty();

    // weights each of the IMUs
    double I1Weight = I2Uncertainty / (I2Uncertainty + I1Uncertainty); // weight of I1
    double I2Weight = 1 - I1Weight; // weight of I2

    // aggregates the heading by applying the weights to all the headings
    double aggregatedHeading = (I1Weight * I1Heading) + (I2Weight * I2Heading);


    if (abs(I1Heading - I2Heading) > 15) {
        if (I1Uncertainty < I2Uncertainty) {
            aggregatedHeading = I1Heading;
        } else {
            aggregatedHeading = I2Heading;
        }
    }
    return aggregatedHeading;
}

// Kalman Filter class methods
// (private)
void KalmanFilter::KalmanFilterLoop()
{

    // current measurement value
    double currentMeasurement = 1;
    double previousMeasurement = 1;

    // current actual values
    double currentHeading = 1;
    double currentCovariance = 1;

    // predicted measurement values for later in the cycle
    double statePrediction = 1;
    double estimateVariancePrediction = 1;

    // variable for the Kalman gain
    double kalmanGain = 1;

    // standard deviation of the variances
    double measurementDeviation = 1;
    double predictionDeviation = 1;

    // velocity values
    double velocity = 1;
    double msAcceleration = 1;
    // direction of accelerometer values for velocity
    bool isPositive = true;
    bool wasPositive = true;
            
    // initial loop

    // measurement guess phase
    currentHeading = inertial->get_heading(); // initial measurement, treated as "filtered" for starting cycle
    currentCovariance = 0.01; // initial guess

    // prediction phase
    statePrediction = currentHeading + (velocity * (this->delay / 1000));; // State Extrapolation Equation
    estimateVariancePrediction = currentCovariance + ((std::pow(this->delay, 2) / 1000) * predictionDeviation); // Covariance Extrapolation Equation

    // looping filter
    while (true) {
                // MEASUREMENT PHASE
                currentMeasurement = inertial->get_heading(); // unit is degrees

                // KALMAN GAIN CALCULATION
                kalmanGain = estimateVariancePrediction / (estimateVariancePrediction + measurementDeviation); // Kalman Gain Equation (value = deg / (deg + deg))

                // UPDATE PHASE
                currentHeading = statePrediction + (kalmanGain * (currentMeasurement - statePrediction)); // State Update Equation (deg = deg + (value * (deg - deg)))
                currentCovariance = (1 - kalmanGain) * estimateVariancePrediction; // Covariance Update Equation

                // reset of filter if it passes 0
                if (((previousMeasurement > 355) && (currentMeasurement < 5)) ||
                    ((previousMeasurement < 5) && (currentMeasurement > 355))) 
                    {
                        currentHeading = currentMeasurement;
                    }

                // VELOCITY UPDATE
                velocity = angVelReader.get();

                // VARIANCE/DEVIATION CALCULATION

                // measurement variance
                measurementVariances.push_back(currentMeasurement);
                if (measurementVariances.size() > 50) {measurementVariances.pop_front();}
                measurementDeviation = calculateStandardDeviation(measurementVariances); // unit is degrees
                if (measurementDeviation == 0) {measurementDeviation = 0.00001;}
                // prediction variance
                predictionVariances.push_back(statePrediction);
                if (predictionVariances.size() > 50) {predictionVariances.pop_front();}
                predictionDeviation = calculateStandardDeviation(predictionVariances); // unit is degrees
                if (predictionDeviation == 0) {predictionDeviation = 0.00001;}

                // PREDICTION PHASE
                statePrediction = currentHeading + (velocity * (this->delay / 1000));; // State Extrapolation Equation (deg = deg + (dps * (ms / 1000)))
                estimateVariancePrediction = currentCovariance + ((std::pow(this->delay, 2) / 1000) * predictionDeviation); // Covariance Extrapolation Equation (deg = deg + (sec^2 * deg))

                // ENDING DELAY AND OUTPUT UPDATE
                previousMeasurement = currentMeasurement;

                this->filteredHeading = currentHeading;
                this->filterUncertainty = currentCovariance;
                pros::delay(this->delay);
    }
}


// constructor (public)
KalmanFilter::KalmanFilter(pros::IMU* inertial, TrackingSensor angVelReader) {

    // instance variable initializations
    filterLoop_ptr = NULL; // sets the filter loop pointer to null so it can be defined later
    filteredHeading = -1; // filtered heading is set to -1 as an unset value
    filterUncertainty = -1; // filtered uncertainty is set to -1 as an unset value

    delay = 5; // delay (in ms) between cycles

    this->inertial = inertial; // takes the IMU for its heading
    this->angVelReader = angVelReader; // takes the turning rotation
}

/* 
* returns the filtered heading that the filter provides,
* but will return -1 if the filter is not active
* (public)
*/
double KalmanFilter::getFilteredHeading()
    {return this->filteredHeading;}

// (public)
double KalmanFilter::getFilterUncertainty()
    {return this->filterUncertainty;}

// starts the filter loop if it is not already active (public)
void KalmanFilter::startFilter() {

    auto filterLoopFunction = [this]() {return this->KalmanFilterLoop();};

    if (filterLoop_ptr == NULL) {
        filterLoop_ptr = new pros::Task(filterLoopFunction);
    }
}

// completely ends the filter loop if it is active (public)
void KalmanFilter::endFilter() {
    if (filterLoop_ptr != NULL) {

        filterLoop_ptr->remove();
        filterLoop_ptr = NULL;

        filteredHeading = -1;

        measurementVariances.clear();
        predictionVariances.clear();
    }
}
//hif