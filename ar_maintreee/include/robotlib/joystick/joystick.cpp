/**
 ******************************************************************************
 * @file    joystick.cpp
 * @brief   Implementation for supporting joystics
 * @author  Robotics Team, IOE Pulchowk Campus
 ******************************************************************************
 */

#include <math.h>
#include <stdio.h>

#include "../common/processor_type.h"
#include "../maths/math.hpp"
#include "joystick.hpp"

Joystick::Joystick(UART_HandleTypeDef *_huart, uint8_t _dead_band, uint16_t _timeout)
    : dead_band(_dead_band), timeout(_timeout), uart(_huart, UART_RECEIVING, 15) {
    }

bool Joystick::update()
{
    just_pressed_buttons = 0;
    just_released_buttons = 0;
    clicked_buttons = 0;

    if (!uart.get_received_data((uint8_t *)&data))
        return false;

    uint32_t now = HAL_GetTick();

    if (now - button_update_tick >= JOYSTICK_DEBOUNCE_DURATION)
    {
        long_pressed_buttons = 0;
        pressed_buttons = data.buttons;
        just_pressed_buttons = (data.buttons & ~prev_buttons);
        just_released_buttons = (~data.buttons & prev_buttons);

        for (int i = 0; i < 16; ++i)
        {
            bool is_pressed = data.buttons & BUTTONS(i);

            if (is_pressed)
            {
                if (!(prev_buttons & BUTTONS(i)))
                {
                    first_pressed_tick[i] = now;
                }

                if (now - first_pressed_tick[i] >= JOYSTICK_LONG_PRESS_DURATION)
                {
                    long_pressed_buttons |= BUTTONS(i);
                }
            }
            else
            {
                if (prev_buttons & BUTTONS(i))
                {
                    if (now - first_pressed_tick[i] < JOYSTICK_LONG_PRESS_DURATION)
                    {
                        clicked_buttons |= BUTTONS(i);
                    }
                }
            }
        }

        prev_buttons = data.buttons;
        button_update_tick = now;
    }

    return true;
}

void Joystick::print_received_data()
{
    printf("Joystick: lx:%d ly:%d rx:%d ry:%d lt:%u rt:%u buttons:%04x\n",
           data.lx,
           data.ly,
           data.rx,
           data.ry,
           data.lt,
           data.rt,
           data.buttons);
}

void Joystick::print_data()
{
    printf("Joystick: lx:%f ly:%f rx:%f ry:%f lt:%f rt:%f, prs:%04x, tgl:%04x clk:%04x lng:%04x:\n",
           lx(),
           ly(),
           rx(),
           ry(),
           lt(),
           rt(),
           pressed_buttons,
           get_toggled_buttons(),
           clicked_buttons,
           long_pressed_buttons);
}

void Joystick::reset() {    
    data.lx = 0;           // Left stick X
    data.ly = 0;           // Left stick Y
    data.rx = 0;           // Right stick X
    data.ry = 0;           // Right stick Y
    data.lt = 0;           // Left trigger
    data.rt = 0;           // Right trigger
    data.buttons = 0;     // Button bitmask
    data.status = Status{0};       
    data.accX = 0;         // Accelerometer X
    data.accZ = 0;         // Accelerometer Z
    data.accY = 0;  
    prev_buttons = 0;          /**< Previous button state. */
    pressed_buttons = 0;       /**< Pressed buttons. */
    just_pressed_buttons = 0;  /**< Just pressed buttons. */
    just_released_buttons = 0; /**< Just released buttons. */
    clicked_buttons = 0;       /**< Clicked buttons. */
    long_pressed_buttons = 0;  /**< Long pressed buttons. */
    first_pressed_tick[16] = {0};
    button_update_tick = 0;
}
