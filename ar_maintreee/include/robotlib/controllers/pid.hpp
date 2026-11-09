
/**********************************************************************************************
 * @file pid.hpp
 * @brief PID library header modified from Arduino PID Library - Version 1.2.1
 * @author Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 * @date 2023
 *
 * @section license License
 * This PID Library is licensed under the MIT License
 * 
 * - Modified by Robotics club, Pulchowk Campus
 *********************************************************************************************/

#ifndef _PID_HPP
#define _PID_HPP

#include <cstdint>

/** @enum Mode
 *  @brief Enum for Auto or Manual Tuning modes.
 */
enum Mode
{
    MANUAL,    /**< Manual Tuning Mode */
    AUTOMATIC /**< Automatic Tuning Mode */
};

/** @enum ControlDirection
 *  @brief Enum for direct or reverse control direction.
 * 
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.
 */
enum ControlDirection
{
    DIRECT,  /**< Direct Control Direction */
    REVERSE /**< Reverse Control Direction */
};

/** @enum ProportionalType
 *  @brief Enum for proportional output based on error or measurement.
 */
enum ProportionalType
{
    PROPORTIONAL_ON_ERROR,        /**< Proportional on Error */
    PROPORTIONAL_ON_MEASUREMENT   /**< Proportional on Measurement */
};

/** @class PID
 *  @brief PID Controller class.
 */
class PID
{
private:
    float kp_, ki_, kd_;
    float outMin_, outMax_;
    int16_t sampleTime_;
    ProportionalType pType_;
    ControlDirection controlDirection_;
    Mode mode_;

    float kpScaled_, kiScaled_, kdScaled_;
    float lastInput_ = 0.0f;
    uint32_t lastTime_;

public:
    float input = 0.0f;
    float setpoint = 0.0f;
    float outputSum = 0.0f;
    float output = 0.0f;

    /** @brief Default constructor for PID class. */
    PID() = default;

    /** @brief Copy constructor for PID class. */
    PID(const PID &) = default;

    /** @brief Constructor for PID class.
     *  @param kp Proportional gain.
     *  @param ki Integral gain.
     *  @param kd Derivative gain.
     *  @param outMin Minimum output limit.
     *  @param outMax Maximum output limit.
     *  @param sampleTime Time between PID calculations.
     *  @param pType Type of proportional output (Error/Measurement).
     *  @param controlDirection Control direction (Direct/Reverse).
     *  @param mode Tuning mode (Auto/Manual).
     */
    PID(float kp, float ki, float kd,
        float outMin, float outMax, int16_t sampleTime,
        ProportionalType pType = PROPORTIONAL_ON_ERROR,
        ControlDirection controlDirection = DIRECT,
        Mode mode = AUTOMATIC);

    /** @brief Initialization method for PID class. */
    void init();

    /** @brief Performs the PID calculation.
     *  @return True if successful PID computation.
     *  @details It should be called every time loop() cycles. ON/OFF and
     *  calculation frequency can be set using SetMode and SetSampleTime respectively.
     */
    bool compute();

    /** @brief Sets new tuning parameters.
     *  @param newKp New proportional gain.
     *  @param newKi New integral gain.
     *  @param newKd New derivative gain.
     */
    void setTunings(float newKp, float newKi, float newKd);

    /** @brief Sets new output limits.
     *  @param newOutMin New minimum output limit.
     *  @param newOutMax New maximum output limit.
     */
    void setOutputLimits(float newOutMin, float newOutMax);

    /** @brief Sets the tuning mode (Auto/Manual).
     *  @param newMode New tuning mode.
     */
    void setMode(Mode newMode);

    /** @brief Sets the sample time for PID calculation.
     *  @param newSampleTime New sample time.
     */
    void setSampleTime(uint16_t newSampleTime);

    /** @brief Resets the PID internal state */
    void reset();

    float getOutMax();
    float getOutMin();

    /** @brief Destructor for PID class. */
    ~PID() = default;
};

#endif