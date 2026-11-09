/**********************************************************************************************
 * @file discrete_pid.hpp
 * @brief A PID library for STM32 formulated in a mathematically correct way.
 * @author Álvaro Villoslada <https://github.com/Alvipe/Arduino-discrete-time-PID>
 * @date 2025
 *
 * - Modified by Robotics club, Pulchowk Campus
 ************************************************************************

 * @class Discrete_PID
 * A Proportional Integral Derivative (PID) controller is the most commonly used
 * method to control a process.
 *
 * A PID controller calculates an "error" value as the difference between the
 * measured output (the process variable) and the reference input (the setpoint).
 * The controller attempts to minimize the error by adjusting the process control
 * inputs. The proportional value determines the reaction to the current error,
 * the integral value determines the reaction based on the sum of recent errors,
 * and the derivative value determines the reaction based on the rate at which
 * the error has been changing.
 *
 * Algorithm
 * The PID algorithm has been implemented by computing the Laplace transform of
 * the PID expression in the parallel form, and then by computing the Z transform
 * of the laplace expression using the backward Euler method. The resulting
 * implementation of the discretized PID is known as the "velocity algorithm".
 * This implementation has some advantages over simpler discrete PID algorithms,
 * such as the elimination of the "reset windup" phenomenon or bumpless parameter
 * changes. However, it is more affected by "derivative kicks": great changes of
 * the control signal caused by sudden variations of the error (due to noise or
 * steep changes of the setpoint). To avoid this problem, the derivative term of
 * the implemented PID controller is modified to a LPF filter to make it less
 * noisy, avoiding the "derivative kick".
 *
 *
 * PID implementation:
 *    u[k] = -A1/A0*u[k-1] - A2/A0*u[k-2] + B0/A0*e[k] + B1/A0*e[k-1] + B2/A0*e[k-2]
 *    B0 = kp*(1 + N*Ts) + ki*Ts*(1 + N*Ts) + kd*N
 *    B1 = -(kp*(2 + N*Ts) + ki*Ts + 2*kd*N)
 *    B2 = kp + kd*N
 *    A0 = 1 + N*Ts
 *    A1 = -(2 + N*Ts)
 *    A2 = 1
 *
 * where u is the controller output, e is the error, kp is the proportional gain,
 * ki is the integral gain, kd the derivative gain, N is the filtered derivative
 * constant and Ts is the sample time.
 */

#ifndef __DISCRETEPID_H
#define __DISCRETEPID_H

#include "../common/processor_type.h"
#include "pid.hpp"
#include "../maths/math.hpp"
#include "string.h"

#define N_def 1000 // The filter coefficient of the filtered derivative term
class Discrete_PID : public PID
{
public:
	float input = 0.0f, setpoint = 0.0f, output = 0.0f;
	bool is_allowable_error = false;

	Discrete_PID() = default;

	/**
	 * \brief        Class constructor, initializes the PID parameters.
	 * \param[in]    kp          The proportional gain.
	 * \param[in]    ki          The integral gain.
	 * \param[in]    kd          The derivative gain.
	 * \param[in]    sampleTime  The execution period of the controller (in ms).
	 * @param[in]    upperLimit  The maximum output value of the controller.
	 * @param[in]    lowerLimit  The minimum output value of the controller.
	 * @param[in]    tol         The tolerance value for permissible error.
	 * \Details
	 * The constructor first calls setSampleTime function to set the sample
	 * time to the passed value. Then calls setGains to compute the derived
	 * gains B0, B1, B2, A0, A1, A2 from the passed proportional, integral
	 * and derivative gains and the sample time. Finally sets the controller
	 * output limits to a pair of predefined values and sets the state
	 * variables to all zeros.
	 */
	Discrete_PID(float _kp, float _ki, float _kd, float _lowerLimit, float _upperLimit, uint16_t _sampleTime, float _tol = 1e-6f, float _N = N_def);

	void init();

	/**
	 * \brief        Executes the PID algorithm.
	 * \param[in]    input       The measured value of the variable to control.
	 * \param[in]    setpoint    The target value of the variable to control.
	 * \return       output      The control signal.
	 * \Details
	 * The PID algorithm executes only when the elapsedTime variable is equal
	 * or greater than the sample time value defined by the user. The PID
	 * is implemented using the velocity form with a filtered derivative term.
	 * The output computed by the PID algorithm is limited to a maximum and
	 * minimum value, which is very useful to generate a PWM control signal.
	 */
	bool compute();

	/**
	 * \brief    Resets the state buffer to zeros.
	 * \return   True value to indicate that the state has been reset.
	 */
	bool reset();

	/**
	 * \brief        Computes the derived gains B0, B1, B2, A0, A1, A2.
	 * \param[in]    kp      The proportional gain.
	 * \param[in]    ki      The integral gain.
	 * \param[in]    kd      The derivative gain.
	 * \return       None.
	 */
	void setGains(float _kp, float _ki, float _kd);

	/**
	 * \brief        Sets the value of the sample time variable Ts.
	 * \param[in]    sampleTime  User-defined sample time (in ms).
	 * \return       None.
	 */
	void setSampleTime(uint16_t _sampleTime);

	/**
	 * \brief        Sets the maxium and minimum values of controller output.
	 * \param[in]    upperLimit  User-defined maximum output value.
	 * \param[in]    lowerLimit  User-defined minimum output value.
	 * \return       None.
	 */
	void setOutputLimits(float _upperLimit, float _lowerLimit);

	void setTunings(float _kp, float _ki, float _kd);
	void setTempGains(float _kp, float _ki, float _kd);

	void suppress_integral();
	void unsuppress_integral();

private:
	float kp, prev_kp; // Proportional gain
	float ki, prev_ki; // Integral gain
	float kd, prev_kd; // Derivative gain
	float B0;					 // Derived gain B0 = kp*(1 + N*Ts) + ki*Ts*(1 + N*Ts) + kd*N
	float B1;					 // Derived gain B1 = -(kp*(2 + N*Ts) + ki*Ts + 2*kd*N)
	float B2;					 // Derived gain B2 = kp + kd*N
	float A0;					 // Derived gain A0 = 1 + N*Ts
	float A1;					 // Derived gain A1 = -(2 + N*Ts)
	float A2;					 // Derived gain A2 = 1
	float C0;
	float C1;
	float C2;
	float N;
	float state[6];		// State array of length 4 to store e[k-1], e[k-2], u[k-1] and u[k-2]
	float Ts;					// Sample time
	float upperLimit; // Maximum output value of the controller
	float lowerLimit; // Minimum output value of the controller
	float tol = 1e-6; // Tolerance value for permissible error
	float previous_time_sec = 0.0f;
	float prev_input = 0.0f;

	bool is_i_suppressed = false;
	uint8_t allow_error_count = 0;
};

#endif
