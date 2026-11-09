/**
 ******************************************************************************
 * @file    tfmini.cpp
 * @author  Robotics Team, IOE Pulchowk Campus
 * @brief   This file implements the TFmini class for receiving and parsing TFmini data
 * @details
 * - Created: 2023 by Kundan Siwakoti(078bei018)
 * - Modified: Jan 15, 2024 by Sagar Chaudhary(078bei031)
 *   - Improved callback and parsing algorithm
 *   - Added restore settings function
 *   - Added reset device
 *   - Documented file
 ******************************************************************************
 */

#include <stdio.h>
#include "tfmini.hpp"

/**
 * @brief   Constructor for TFmini class
 * @param   huart: UART handle for communication
 */
TFmini::TFmini(UART_HandleTypeDef *huart)
    : huart_(huart) {}

/**
 * @brief   Initialize TFmini communication
 * @details Set the flag for waiting for start bytes and initiate the DMA reception of header bytes.
 */
void TFmini::init()
{
    start_receiving();
}

/**
 * @brief Starts receiving from UART
 *
 */
void TFmini::start_receiving()
{
    // start receiving data by first receiving two header bytes
    is_waiting_for_start_bytes_ = true;
    HAL_UART_Receive_DMA(huart_, (uint8_t *)&buffer_.head, HEAD_SIZE);
}

/**
 * @brief   Callback function for handling received data
 * @details This function processes the received data, checks for valid headers, and calculates checksum for verification.
 *          If the data is valid, it updates the internal variables with distance, strength, and temperature.
 */
void TFmini::rx_callback()
{
    if (is_waiting_for_start_bytes_)
    {
        // check whether both header bytes matches or note
        // if not, again receive the two header bytes
        // if matched, receive data and checksum
        if ((buffer_.head.head0 == TF_HEAD) && (buffer_.head.head1 == TF_HEAD))
        {
            is_waiting_for_start_bytes_ = false;
            HAL_UART_Receive_DMA(huart_, (uint8_t *)&buffer_.data, DATA_SIZE_PLUS_CHECKSUM);
        }
        else
        {
            HAL_UART_Receive_DMA(huart_, (uint8_t *)&buffer_.head, HEAD_SIZE);
        }
    }
    else
    {
        // again receive two header bytes for continuos reception
        is_waiting_for_start_bytes_ = true;
        HAL_UART_Receive_DMA(huart_, (uint8_t *)&buffer_.head, HEAD_SIZE);

        uint8_t *ptr = (uint8_t *)&buffer_;
        uint8_t checksum = 0;

        for (int i = 0; i < (FRAME_SIZE - 1); ++i)
        {
            checksum += *(ptr++);
        }

        // check if checksum matches or not
        // if matched, parse data
        // if not, do not parse
        if (checksum == *ptr)
        {
            distance_ = ((uint16_t)buffer_.data.distance_H << 8) + (uint16_t)buffer_.data.distance_L;
            strength_ = ((uint16_t)buffer_.data.strength_H << 8) + (uint16_t)buffer_.data.strength_L;
            temperature_ = ((uint16_t)buffer_.data.temperature_H << 8) + (uint16_t)buffer_.data.temperature_L;
        }
    }
}

/**
 * @brief   Get the measured distance
 * @return  Measured distance in centimeters
 */
uint16_t TFmini::get_distance()
{
    return distance_;
}

/**
 * @brief   Get the measured strength
 * @return  Measured signal strength
 */
uint16_t TFmini::get_strength()
{
    return strength_;
}

/**
 * @brief   Get the measured temperature
 * @return  Measured temperature in degrees Celsius
 */
uint16_t TFmini::get_temperature()
{
    return temperature_;
}

/**
 * @brief   Restore factory settings
 * @return  true if success esle false
 */
bool TFmini::restore_settings()
{
    uint8_t retry = 0U;

    // restore factory settings
    uint8_t restore_settings_cmd[] = {0x5A, 0x04, 0x10, 0x6E};
    uint8_t restore_succeed_response[] = {0x5A, 0x05, 0x10, 0x00, 0x6E};
    uint8_t restore_received_response[sizeof(restore_succeed_response)];
    bool is_restore_succeed = false;

    while (!is_restore_succeed)
    {
        HAL_UART_Transmit(huart_,
                          restore_settings_cmd,
                          sizeof(restore_settings_cmd),
                          500U);

        HAL_UART_Receive(huart_,
                         restore_received_response,
                         sizeof(restore_received_response),
                         500U);

        // HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
        is_restore_succeed = true;
        for (uint16_t i = 0U; i < sizeof(restore_succeed_response); ++i)
        {
            is_restore_succeed &= (restore_succeed_response[i] == restore_received_response[i]);
        }

        // failure on 10th retry
        if (retry++ == 10U)
        {
            printf("TFmini:: Failed to restore factory settings\n");
            HAL_Delay(500);
            return false;
        }
    }
    printf("TFmini settings restored\n");
    HAL_Delay(500);

    retry = 0;

    // save settings
    uint8_t save_settings_cmd[] = {0x5A, 0x04, 0x11, 0x6F};
    uint8_t save_succeed_response[] = {0x5A, 0x05, 0x11, 0x00, 0x6F};
    uint8_t save_received_response[sizeof(save_succeed_response)];
    bool is_save_succeed = false;

    while (!is_save_succeed)
    {
        HAL_UART_Transmit(huart_,
                          save_settings_cmd,
                          sizeof(save_settings_cmd),
                          500U);

        HAL_UART_Receive(huart_,
                         save_received_response,
                         sizeof(save_received_response),
                         500U);

        // HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);

        is_save_succeed = true;
        for (uint16_t i = 0U; i < sizeof(save_succeed_response); ++i)
        {
            is_save_succeed &= (save_succeed_response[i] == save_received_response[i]);
        }

        // failure on 10th retry
        if (retry++ == 10U)
        {
            printf("TFmini:: Falied to save content\n");
            HAL_Delay(500);
            return false;
        }
    }
    printf("TFmini settings saved\n");
    HAL_Delay(500);

    // reset TFmini
    if (reset())
    {
        return true;
    }

    return false;
}

/**
 * @brief   Reset TFmini
 * @return  true if success esle false
 */
bool TFmini::reset()
{
    uint8_t retry = 0;

    // restore factory settings
    uint8_t reset_cmd[] = {0x5A, 0x04, 0x10, 0x6E};
    uint8_t reset_succeed_response[] = {0x5A, 0x05, 0x10, 0x00, 0x6E};
    uint8_t reset_received_response[sizeof(reset_succeed_response)];
    bool is_reset_succeed = false;

    while (!is_reset_succeed)
    {
        HAL_UART_Transmit(huart_,
                          reset_cmd,
                          sizeof(reset_cmd),
                          500U);

        HAL_UART_Receive(huart_,
                         reset_received_response,
                         sizeof(reset_received_response),
                         500U);

        // HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);

        is_reset_succeed = true;
        for (uint16_t i = 0U; i < sizeof(reset_succeed_response); ++i)
        {
            is_reset_succeed &= (reset_succeed_response[i] == reset_received_response[i]);
        }

        // failure on 10th retry
        if (retry++ == 10U)
        {
            printf("TFmini:: Failed to reset\n");
            HAL_Delay(500);
            return false;
        }
    }
    printf("TFmini reset\n");
    HAL_Delay(500);

    return true;
}

USART_TypeDef *TFmini::get_uart_instance()
{
    return huart_->Instance;
}

/**
 * @brief Check if strenth is reliable or not
 *
 * @return true   Reliable
 * @return false  Unreliable
 */
bool TFmini::is_reliable()
{
    if ((strength_ > 200U) && (strength_ != 65535U))
    {
        return true;
    }

    return false;
}

/**
 * @brief Print received data
 *
 */
void TFmini::print_data()
{
    printf("TFmini:: dist:%d strength:%d temp:%d\n", distance_, strength_, temperature_);
}
