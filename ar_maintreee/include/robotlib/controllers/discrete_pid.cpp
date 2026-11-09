#include "discrete_pid.hpp"

Discrete_PID::Discrete_PID(float _kp, float _ki, float _kd, float _lowerLimit, float _upperLimit, uint16_t _sampleTime, float _tol, float _N)
{
	N = _N;
	tol = _tol;
	Discrete_PID::setSampleTime(_sampleTime);
	Discrete_PID::setGains(_kp, _ki, _kd);
	Discrete_PID::setOutputLimits(_lowerLimit, _upperLimit);
	memset(state, 0, 6u * sizeof(float));
}

void Discrete_PID::init()
{
	reset();
}

bool Discrete_PID::compute()
{
	float current_time_sec = uwTick / 1000.0f;
	float elapsed_time_sec = current_time_sec - previous_time_sec;

	// This ensures that the gains are updated when the k valued are changed while using the stm_cube_monitor
	if (prev_kp != kp || prev_ki != ki || prev_kd != kd)
	{
		setGains(kp, ki, kd);
		reset();
	}

	float error = setpoint - input;
	float _dinput = input - prev_input;

	if (elapsed_time_sec >= Ts)
	{

		/* u[k] = -A1/A0*u[k-1] - A2/A0*u[k-2] + B0/A0*e[k] + B1/A0*e[k-1] + B2/A0*e[k-2]  */
		float _output = -A1 / A0 * state[2] - A2 / A0 * state[3] + B0 / A0 * error + B1 / A0 * state[0] + B2 / A0 * state[1] + C0 / A0 * _dinput + C1 / A0 * state[4] + C2 / A0 * state[5];

		/* Anti-windup */
		if (_output > upperLimit)
		{
			_output = upperLimit;
		}
		else if (_output < lowerLimit)
		{
			_output = lowerLimit;
		}

		/* Update state */
		state[1] = state[0]; // e[k-2]
		state[0] = error;		 // e[k-1]
		state[3] = state[2]; // u[k-2]
		state[2] = _output;	 // u[k-1]
		state[5] = state[4]; // i[k-2]
		state[4] = _dinput;	 // i[k-1]

		/* Adding the interval instead of using the value of current_time_sec is
		a better way to ensure that successive periods are identical */
		previous_time_sec += Ts;

		output = _output;
		prev_input = input;

		is_allowable_error = fabs(error) < tol;

		// 50ms of settling time for allowable error (for a 100hz robot)
		if (is_allowable_error)
		{
			if (allow_error_count < 5)
			{
				allow_error_count++;
				is_allowable_error = false;
			}
		}
		else
		{
			allow_error_count = 0;
		}
		return true;
	}

	return false;
}

bool Discrete_PID::reset()
{
	/* Clear the state buffer. The size will be always 6 samples */
	memset(state, 0, 6u * sizeof(float));
	return true;
}

void Discrete_PID::setGains(float _kp, float _ki, float _kd)
{
	kp = _kp;
	ki = _ki;
	kd = _kd;

	setTempGains(kp, ki, kd);

	prev_kp = kp;
	prev_ki = ki;
	prev_kd = kd;
}

void Discrete_PID::setTempGains(float _kp, float _ki, float _kd)
{

	B0 = _kp * (1 + N * Ts) + _ki * Ts * (1 + N * Ts);
	/* Derived coefficient B1 */
	B1 = -(_kp * (2 + N * Ts) + _ki * Ts);
	/* Derived coefficient B2 */
	B2 = _kp;
	/* Derived coefficient A0 */
	A0 = 1 + N * Ts;
	/* Derived coefficient A1 */
	A1 = -(2 + N * Ts);
	/* Derived coefficient A2 */
	A2 = 1;

	C0 = _kd * N;

	C1 = -2 * _kd * N;

	C2 = _kd * N;
}

void Discrete_PID::suppress_integral()
{
	if (is_i_suppressed)
		return;
	setTempGains(kp, 0.0f, kd);
	is_i_suppressed = true;
}

void Discrete_PID::unsuppress_integral()
{
	if (!is_i_suppressed)
		return;
	setTempGains(kp, ki, kd);
	is_i_suppressed = false;
}

void Discrete_PID::setTunings(float _kp, float _ki, float _kd)
{
	Discrete_PID::setGains(_kp, _ki, _kd);
}

void Discrete_PID::setSampleTime(uint16_t _sampleTime)
{
	/* Sample time in sec */
	Ts = (float)_sampleTime / 1000;
	/* The derived coefficients have to be recomputed because they are affected by Ts */
	Discrete_PID::setGains(kp, ki, kd);
}

void Discrete_PID::setOutputLimits(float _lowerLimit, float _upperLimit)
{
	if (_upperLimit < _lowerLimit)
		return;

	upperLimit = _upperLimit;
	lowerLimit = _lowerLimit;
}
