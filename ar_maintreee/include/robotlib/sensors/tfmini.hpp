/**
 ******************************************************************************
 * @file    tfmini.hpp
 * @brief   Header for interfacing TFmini plus Lidar
 * @author  Robotics Team, IOE Pulchowk Campus
 ******************************************************************************
 * @note
 * @brief   Frame Format:
 * - Byte0: 0x59, frame header, same for each frame
 * - Byte1: 0x59, frame header, same for each frame
 * - Byte2: Dist_L distance value lower by 8 bits
 * - Byte3: Dist_H distance value higher by 8 bits
 * - Byte4: Strength_L low 8 bits
 * - Byte5: Strength_H high 8 bits
 * - Byte6: Temp_L low 8 bits (suitable for version later than V1.3.0)
 * - Byte7: Temp_H high 8 bits (suitable for version later than V1.3.0)
 * - Byte8: Checksum is the lower 8 bits of the cumulative sum of the numbers of the first 8 bytes.
 ******************************************************************************
 * @attention
 * @brief Data Communication Protocol:
 * - Communication interface  : UART
 * - Baud rate                : 115200
 * - Data bit                 : 8
 * - Stop bit                 : 1
 * - Parity check             : None
 *******************************************************************************
 */

#ifndef TFMINI_HPP__
#define TFMINI_HPP__

#include <stdint.h>

#include "../common/processor_type.h"
#include "../msgs/sensor_msg.hpp"

#define TF_HEAD                 0x59
#define FRAME_SIZE              9
#define HEAD_SIZE               2
#define DATA_SIZE_PLUS_CHECKSUM 7


/**
 * @class   TFmini
 * @brief   Class for interfacing TFmini LiDAR
 */
class TFmini
{
private:
  UART_HandleTypeDef *huart_;       /**< UART handle for communication */
  TF_Frame buffer_;                 /**< Buffer for storing received TFmini packet */
  uint16_t distance_;               /**< Measured distance */
  uint16_t strength_;               /**< Measured signal strength */
  uint16_t temperature_;            /**< Measured temperature */
  bool is_waiting_for_start_bytes_; /**< Flag to indicate if waiting for the start bytes of a new packet */

public:
  /**
   * @brief   Default constructor
   */
  TFmini() = default;

  /**
   * @brief   Constructor with UART handle
   * @param   huart: UART handle for communication
   */
  TFmini(UART_HandleTypeDef *huart);

  /**
   * @brief   Default destructor
   */
  ~TFmini() = default;

  /**
   * @brief   Initialize TFmini
   */
  void init();

  /**
   * @brief Starts receiving from UART
   */
  void start_receiving();

  /**
   * @brief   Callback for handling received data
   */
  void rx_callback();

  /**
   * @brief   Get the measured distance
   * @return  Measured distance in centimeters
   */
  uint16_t get_distance();

  /**
   * @brief   Get the measured temperature
   * @return  Measured temperature in degrees Celsius
   */
  uint16_t get_temperature();

  /**
   * @brief   Get the measured signal strength
   * @return  Measured signal strength
   */
  uint16_t get_strength();

  /**
   * @brief   Restore factory settings
   * 
   * @return  true  Success 
   * @return  false Failure
   */
  bool restore_settings();

  /**
   * @brief   Reset TFmini
   * @retval  true if success esle false
   */
  bool reset();

  /**
   * @brief Get the uart instance object
   * @return USART_TypeDef* Instance of handled USART
   */
  USART_TypeDef *get_uart_instance();

  /**
   * @brief Print received data
   * 
   */
  void print_data();

/**
 * @brief Check if strenth is reliable or not
 * 
 * @return true   Reliable
 * @return false  Unreliable
 */
  bool is_reliable();
};

#endif // TFMINI_HPP__
