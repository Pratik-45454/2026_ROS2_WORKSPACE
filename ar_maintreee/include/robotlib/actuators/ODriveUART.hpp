#ifndef ODriveUART_hpp
#define ODriveUART_hpp

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "usart.h"

// UART Buffer Size
#ifndef ODriveUART_RX_BUFFER_SIZE
#define ODriveUART_RX_BUFFER_SIZE 50
#endif
#ifndef ODriveUART_TX_BUFFER_SIZE
#define ODriveUART_TX_BUFFER_SIZE 50
#endif

// ODrive.Axis.AxisState
enum ODriveAxisState
{
    AXIS_STATE_UNDEFINED = 0,
    AXIS_STATE_IDLE = 1,
    AXIS_STATE_STARTUP_SEQUENCE = 2,
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,
    AXIS_STATE_MOTOR_CALIBRATION = 4,
    AXIS_STATE_ENCODER_INDEX_SEARCH = 6,
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7,
    AXIS_STATE_CLOSED_LOOP_CONTROL = 8,
    AXIS_STATE_LOCKIN_SPIN = 9,
    AXIS_STATE_ENCODER_DIR_FIND = 10,
    AXIS_STATE_HOMING = 11,
    AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION = 12,
    AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION = 13,
    AXIS_STATE_ANTICOGGING_CALIBRATION = 14,
};

struct ODriveFeedback
{
    float pos;
    float vel;
};

class ODriveUART
{
public:
    /**
     * @brief Initializes the ODriveUART object.
     */
    ODriveUART(UART_HandleTypeDef *_huart);

    /**
     * @brief Clears the error status of the ODrive and restarts the brake
     * resistor if it was disabled due to an error.
     */
    void clearErrors();

    /**
     * @brief Sends a new position setpoint.
     * @param position The new position setpoint in turns.
     */
    void setPosition(float position);

    /**
     * @brief Sends a new position setpoint with a velocity feedforward term.
     * @param position The new position setpoint in turns.
     * @param velocity_feedforward The velocity feedforward term.
     */
    void setPosition(float position, float velocity_feedforward);

    /**
     * @brief Sends a new position setpoint with velocity and torque feedforward terms.
     * @param position The new position setpoint in turns.
     * @param velocity_feedforward The velocity feedforward term.
     * @param torque_feedforward The torque feedforward term.
     */
    void setPosition(float position, float velocity_feedforward, float torque_feedforward);

    /**
     * @brief Sends a new velocity setpoint.
     * @param velocity The new velocity setpoint in turns per second.
     */
    void setVelocity(float velocity);

    /**
     * @brief Sends a new velocity setpoint with a torque feedforward term.
     * @param velocity The new velocity setpoint in turns per second.
     * @param torque_feedforward The torque feedforward term.
     */
    void setVelocity(float velocity, float torque_feedforward);

    /**
     * @brief Sends a new torque setpoint.
     * @param torque The new torque setpoint in Nm.
     */
    void setTorque(float torque);

    /**
     * @brief Puts the ODrive into trapezoidal trajectory mode and sends a new
     * @param position The new position setpoint in turns.
     */
    void trapezoidalMove(float position);

    /**
     * @brief Requests the latest position and velocity estimates.
     *
     * Returns pos = 0.0 and vel = 0.0 in case of a communication error.
     */
    ODriveFeedback getFeedback();

    /**
     * @brief Requests the latest position estimate.
     *
     * Returns 0.0 in case of a communication error.
     */
    float getPosition() { return getFeedback().pos; }

    /**
     * @brief Requests the latest velocity estimate.
     *
     * Returns 0.0 in case of a communication error.
     */
    float getVelocity() { return getFeedback().vel; }

    // Generic parameter access
    char *getParameterAsString(const char *path);
    int getParameterAsInt(const char *path) { return atoi(getParameterAsString(path)); }
    float getParameterAsFloat(const char *path) { return atof(getParameterAsString(path)); }
    void setParameter(const char *path, const int value);

    /**
     * @brief Tells the ODrive to change its axis state.
     */
    void setState(ODriveAxisState requested_state);

    /**
     * @brief Requests the current axis state from the ODrive.
     *
     * Returns AXIS_STATE_UNDEFINED in case of a communication error.
     */
    ODriveAxisState getState();

    /**
     * @brief Reads a command line from UART untill linefeed('\n') or timeout_ms
     * @param timeout_ms Timeout in milliseconds
     */
    void readLine(uint32_t timeout_ms = 10);

    /**
     * @brief Sends a command to the ODrive from txBuffer starting to linefeed('\n).
     */
    void sendCmdLine()
    {
        HAL_UART_Transmit(huart, (uint8_t *)txBuffer, strlen(txBuffer), 100);
    }

    void sendCmdLineDMA()
    {
        HAL_UART_Transmit_DMA(huart, (uint8_t *)txBuffer, strlen(txBuffer));
    }

    void reboot();

    // Buffer access
    void setTxBuffer(const char *buffer);
    char *getTxBuffer() { return txBuffer; }
    char *getRxBuffer() { return rxBuffer; }

private:
    UART_HandleTypeDef *huart;                // UART handle
    char txBuffer[ODriveUART_TX_BUFFER_SIZE]; // Transmit buffer
    char rxBuffer[ODriveUART_RX_BUFFER_SIZE]; // Receive buffer
};

#endif // ODriveUART_hpp
