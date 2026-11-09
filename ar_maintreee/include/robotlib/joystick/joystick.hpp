/**
 ******************************************************************************
 * @file    joystick.hpp
 * @brief   Header file for supporting joysticks
 * @author  Robotics Team, IOE Pulchowk Campus
 ******************************************************************************
 */

#ifndef _JOYSTICK_HPP
#define _JOYSTICK_HPP

#include "../common/processor_type.h"
#include "../communication/uart.hpp"
#include "../msgs/joy_msg.hpp"
#include "../maths/math.hpp"
// #include <cmath>

#ifndef JOYSTICK_LONG_PRESS_DURATION
#define JOYSTICK_LONG_PRESS_DURATION 750 // ms
#endif
#ifndef JOYSTICK_DEBOUNCE_DURATION
#define JOYSTICK_DEBOUNCE_DURATION 30 // ms
#endif

#if !(defined USE_PS4 || USE_XBOX)
#define USE_PS4
#endif

/**
 * @brief Base class for joystick functionality.
 *
 * This class provides basic functionality for working with joysticks,
 * including initialization, data retrieval, and printing raw data.
 */
class Joystick
{
private:
    uint8_t dead_band;                  /**< Deadband value for joystick axes. */
    uint16_t timeout;                   /**< Timeout value for checking joystick connection. */
    JoyData data;                       /**< Structure to hold joystick data. */
    uint16_t prev_buttons = 0;          /**< Previous button state. */
    uint16_t pressed_buttons = 0;       /**< Pressed buttons. */
    uint16_t just_pressed_buttons = 0;  /**< Just pressed buttons. */
    uint16_t just_released_buttons = 0; /**< Just released buttons. */
    uint16_t clicked_buttons = 0;       /**< Clicked buttons. */
    uint16_t long_pressed_buttons = 0;  /**< Long pressed buttons. */
    uint32_t first_pressed_tick[16] = {0};
    uint32_t button_update_tick = 0;

public:
    UART uart; /**< UART instance for communication. */

    /**
     * @brief Default constructor.
     */
    Joystick() = default;

    /**
     * @brief Constructor with parameters.
     *
     * Initializes the joystick with the given UART instance, deadband value, and timeout.
     *
     * @param _huart Pointer to the UART_HandleTypeDef for UART communication.
     * @param _dead_band Deadband value for joystick axes.
     * @param _timeout Timeout value for checking joystick connection.
     */
    Joystick(UART_HandleTypeDef *_huart, uint8_t _dead_band = 20, uint16_t _timeout = 100);

    Joystick(const Joystick &) = default;

    Joystick &operator=(const Joystick &) = default;

    /**
     * @brief Default destructor.
     */
    ~Joystick() = default;

    /**
     * @brief Initialize joystick to start UART communication.
     */
    bool init()
    {
        return uart.init();
    }

    /**
     * @brief Update joystick data.
     *
     * @return True if data is updated, false otherwise.
     */
    bool update();

    /**
     * @brief Print received data from joystick uart.
     */
    void print_received_data();

    /**
     * @brief Print data from joystick.
     */
    void print_data();

    /**
     * @brief Callback function for UART communication.
     */
    void uart_callback()
    {
        uart.process_receive_callback();
    }

    /**
     * @brief Check if joystick is connected.

     * @return True if joystick is connected, false otherwise.
     */
    bool connected()
    {
        if ((HAL_GetTick() - uart.get_last_receive_cplt_tick() < timeout) && (uart.get_last_receive_cplt_tick() != 0))
            return true;
        return false;
    }

    /**
     * @brief Get joystick data.
     *
     * @return JoyData structure containing joystick data.
     */
    JoyData get_data()
    {
        return data;
    }

    /**
     * @brief Get button state.
     *
     * @return 16-bit integer representing button state.
     */
    uint16_t get_buttons()
    {
        return data.buttons;
    }

    /**
     * @brief Get previous button state.
     *
     * @return 16-bit integer representing clicked button flag.
     */
    uint16_t get_clicked_buttons()
    {
        return clicked_buttons;
    }

    /**
     * @brief Get toggled button state.
     *
     * @return 16-bit integer representing toggled button flag.
     */
    uint16_t get_toggled_buttons()
    {
        return just_pressed_buttons | just_pressed_buttons;
    }

    /**
     * @brief Get long pressed button state.
     *
     * @return 16-bit integer representing long pressed button flag.
     */
    uint16_t get_long_pressed_buttons()
    {
        return long_pressed_buttons;
    }

    /**
     * @brief Get button state.
     *
     * @param button Button number.
     * @return True if button is pressed, false otherwise.
     */
    bool pressed(uint8_t button)
    {
        return pressed_buttons & BUTTONS(button);
    }

    bool only_pressed(uint8_t button)
    {
        return (pressed_buttons & BUTTONS(button)) && !(pressed_buttons & ~BUTTONS(button));
    }

    /**
     * @brief Get button state.
     *
     * @param button Button number.
     * @return True if button is released, false otherwise.
     */
    bool released(uint8_t button)
    {
        return !(pressed_buttons & BUTTONS(button));
    }

    bool just_pressed(uint8_t button)
    {
        return just_pressed_buttons & BUTTONS(button);
    }

    bool just_released(uint8_t button)
    {
        return just_released_buttons & BUTTONS(button);
    }

    /**
     * @brief Get button state.
     *
     * @param button Button number.
     * @return True if button is clicked, false otherwise.
     */
    bool clicked(uint8_t button)
    {
        return clicked_buttons & BUTTONS(button);
    }

    bool only_clicked(uint8_t button)
    {
        return (clicked_buttons & BUTTONS(button)) && !(pressed_buttons & ~BUTTONS(button));
    }

    /**
     * @brief Get button state.
     *
     * @param button Button number.
     * @return True if button is toggled, false otherwise.
     */
    bool toggled(uint8_t button)
    {
        return get_toggled_buttons() & BUTTONS(button);
    }

    /**
     * @brief Get button state.
     *
     * @param button Button number.
     * @return True if button is long pressed, false otherwise.
     */
    bool long_pressed(uint8_t button)
    {
        return long_pressed_buttons & BUTTONS(button);
    }

    /**
     * @brief Get left stick x-axis.
     *
     * @return float Normalized left stick value of x-axis.
     */
    float lx()
    {
        if (abs(data.lx) < dead_band)
            return 0.0f;
        return (map<float>((float)data.lx, -127.0f, 127.0f, -1.0f, 1.0f));
    }

    /**
     * @brief Get left stick y-axis.
     *
     * @return float Normalized left stick value of y-axis.
     */
    float ly()
    {
        if (abs(data.ly) < dead_band)
            return 0.0f;
        return (map<float>((float)data.ly, -127.0f, 127.0f, -1.0f, 1.0f));
    }

    /**
     * @brief Get right stick x-axis.
     *
     * @return float Normalized right stick value of x-axis.
     */
    float rx()
    {
        if (abs(data.rx) < dead_band)
            return 0.0f;
        return (map<float>((float)data.rx, -127.0f, 127.0f, -1.0f, 1.0f));
    }

    /**
     * @brief Get right stick y-axis.
     *
     * @return float Normalized right stick value of y-axis.
     */
    float ry()
    {
        if (abs(data.ry) < dead_band)
            return 0.0f;
        return (map<float>((float)data.ry, -127.0f, 127.0f, -1.0f, 1.0f));
    }

    /**
     * @brief Get left trigger.
     *
     * @return float Normalized left trigger value.
     */
    float lt()
    {
        return (map<float>((float)data.lt, 0.0f, 255.0f, 0.0f, 1.0f));
    }

    /**
     * @brief Get right trigger normalized value.
     *
     * @return float Normalized right trigger value.
     */
    float rt()
    {
        return (map<float>((float)data.rt, 0.0f, 255.0f, 0.0f, 1.0f));
    }

    void reset();
};

#endif
