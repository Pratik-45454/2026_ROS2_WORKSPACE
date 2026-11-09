/**
 ******************************************************************************
 * @file    kalmanSV.hpp
 * @brief   Single variable Kalman filter
 * @author  Robotics Team 2024, IOE Pulchowk Campus
 * @date    2023
 ******************************************************************************
 */

#ifndef KALMAN_SV__H__
#define KALMAN_SV_H__

#include <math.h>

/**
 * @brief Template Class for Single Variable Kalman Filter
 * 
 * @tparam data_t 
 * @tparam ratio_t 
 * 
 * @details
 * ### Procedure:
 * 1. Initialize initial estimate, initial estimate error and measurement error.
 *    ```
 *    estimate = initialEstimate
 *    estimateError = initialEstimateError
 *    measurementError = initialMeasurementError
 *    ```
 * 2. Take new measurement
 *    ```
 *    measurement = newMeasurement // fuse here
 *    ```
 * 
 * 3. Calculate Kalman Gain
 *    ```
 *    kalmanGain = estimateError / (estimateError - measurementError)
 *    ```
 * 
 * 4. Update estimate and estimate error
 *    ```
 *    estimate = estimate + kalmanGain * (measurement - estimate)
 *    estimateError = estimateError * (1 - kalmanGain)
 *    ```
 * 
 * 5. Return estimate
 * 
 * 
 * 
 * @note
 * - Kalman Gain lies between 0 and 1.
 * 
 * -  KG increases to 1
 *    - Error in measurement is small relative to error in estimate
 *    - Accurate measurement
 *    - Expect estimates are unstable
 * 
 * - KG decreases to 0
 *   - Error in measurement is high relative to error in estimate
 *   - Inaccurate measurement
 *   - Expect estimates are stable
 */

template <typename data_t = float, typename ratio_t = float>
class KalmanSV
{
private:
    data_t estimate;
    data_t estimateError;
    data_t measurement;
    data_t measurementError;
    ratio_t kalmanGain;

public:
    /**
     * @brief Construct a new Kalman S V object
     * 
     * @param initialEstimate Initial estimated value
     * @param initialEstimateError Initial estimated error
     * @param mesurementErr Measurement error
     */
    KalmanSV(const data_t &initialEstimate, const data_t &initialEstimateError,  const data_t &mesurementErr)
        : estimate(initialEstimate), estimateError(initialEstimateError), measurementError (mesurementErr) {}

    /**
     * @brief Delete a new Kalman S V object explicitly
     * 
     */
    KalmanSV() = delete;

    /**
     * @brief Destroy the Kalman S V object
     * 
     */
    ~KalmanSV() = default;

    /**
     * @brief Function to apply kalman filter to sensor data
     * 
     * @param newMeasurement New measurement from sensor
     * @return Updated estimate after filter 
     */
    data_t compute(const data_t &newMeasurement)
    {
        measurement = newMeasurement;
        kalmanGain = (ratio_t)estimateError / (estimateError + measurementError);
        estimate = estimate + kalmanGain * (measurement - estimate);
        estimateError *= (1 - kalmanGain);

        return estimate;
    }
};

#endif