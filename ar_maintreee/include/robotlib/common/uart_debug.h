#ifndef UART_DEBUG_HPP_
#define UART_DEBUG_HPP_

#include "./processor_type.h"
#include <cstdint>
#include <cstdarg>

class UARTDebug {
public:
    UARTDebug();

    void init(UART_HandleTypeDef* huart);
    void print(const char* str);
    void printf(const char* fmt, ...);
    void onTxComplete(UART_HandleTypeDef* huart);

private:
    UART_HandleTypeDef* uart;
    static constexpr size_t BUFFER_SIZE = 256;
    char buffer[BUFFER_SIZE];
    volatile bool txBusy;
};

#endif // UART_DEBUG_HPP_
