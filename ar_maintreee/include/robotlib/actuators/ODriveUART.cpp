// Author: ODrive Robotics Inc.
// License: MIT
// Documentation: https://docs.odriverobotics.com/v/latest/guides/arduino-uart-guide.html
// Modified (Jun 10, 2024) by Robotics Club, Pulchowk Campus

#include "ODriveUART.hpp"

static const int kMotorNumber = 0;

ODriveUART::ODriveUART(UART_HandleTypeDef *_huart)
    : huart(_huart) {}

void ODriveUART::clearErrors()
{
    snprintf(txBuffer, sizeof(txBuffer), "sc\n");
    sendCmdLine();
}

void ODriveUART::setPosition(float position)
{
    setPosition(position, 0.0f, 0.0f);
}

void ODriveUART::setPosition(float position, float velocity_feedforward)
{
    setPosition(position, velocity_feedforward, 0.0f);
}

void ODriveUART::setPosition(float position, float velocity_feedforward, float torque_feedforward)
{
    snprintf(txBuffer, sizeof(txBuffer), "p %d %.4f %.4f %.4f\n", kMotorNumber, position, velocity_feedforward, torque_feedforward);
    sendCmdLineDMA();
}

void ODriveUART::setVelocity(float velocity)
{
    setVelocity(velocity, 0.0f);
}

void ODriveUART::setVelocity(float velocity, float torque_feedforward)
{
    snprintf(txBuffer, sizeof(txBuffer), "v %d %.4f %.4f\n", kMotorNumber, velocity, torque_feedforward);
    sendCmdLineDMA();
}

void ODriveUART::setTorque(float torque)
{
    snprintf(txBuffer, sizeof(txBuffer), "c %d %.4f\n", kMotorNumber, torque);
    sendCmdLineDMA();
}

void ODriveUART::trapezoidalMove(float position)
{
    snprintf(txBuffer, sizeof(txBuffer), "t %d %.4f\n", kMotorNumber, position);
    sendCmdLineDMA();
}

ODriveFeedback ODriveUART::getFeedback()
{
    __HAL_UART_FLUSH_DRREGISTER(huart);

    snprintf(txBuffer, sizeof(txBuffer), "f %d\n", kMotorNumber);
    sendCmdLine();
    readLine();

    ODriveFeedback feedback = {0.0f, 0.0f};
    char *space = strchr(rxBuffer, ' ');
    if (space != NULL)
    {
        *space = '\0';
        feedback.pos = atof(rxBuffer);
        feedback.vel = atof(space + 1);
    }
    return feedback;
}

char *ODriveUART::getParameterAsString(const char *path)
{
    snprintf(txBuffer, sizeof(txBuffer), "r %s\n", path);
    sendCmdLine();
    readLine();
    return (char *)rxBuffer;
}

void ODriveUART::setParameter(const char *path, const int value)
{
    snprintf(txBuffer, sizeof(txBuffer), "w %s %d\n", path, value);
    sendCmdLine();
}

void ODriveUART::setState(ODriveAxisState requested_state)
{
    setParameter("axis0.requested_state", requested_state);
}

ODriveAxisState ODriveUART::getState()
{
    return (ODriveAxisState)getParameterAsInt("axis0.current_state");
}

void ODriveUART::readLine(uint32_t timeout_ms)
{
    uint32_t timeout_start = HAL_GetTick();
    char *rxPtr = rxBuffer;
    for (;;)
    {
        while (HAL_UART_Receive(huart, (uint8_t *)(rxPtr), 1, 0) != HAL_OK)
        {
            if (HAL_GetTick() - timeout_start >= timeout_ms)
                return;
        }
        if (*(rxPtr++) == '\n')
            break;
    }
}

void ODriveUART::setTxBuffer(const char *cmdLine)
{
    if (strlen(cmdLine) <= sizeof(txBuffer))
    {
        snprintf(txBuffer, sizeof(txBuffer), "%s", cmdLine);
    }
}

void ODriveUART::reboot()
{
    snprintf(txBuffer, sizeof(txBuffer), "sr\n");
    sendCmdLine();
}
