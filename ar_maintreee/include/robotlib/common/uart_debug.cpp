#include "uart_debug.h"
#include <cstring>
#include <cstdio>

UARTDebug::UARTDebug() : uart(nullptr), txBusy(false) {}

void UARTDebug::init(UART_HandleTypeDef* huart) {
    uart = huart;
    txBusy = false;
}

void UARTDebug::print(const char* str) {
    if (!uart || !str || txBusy) return;

    size_t len = strlen(str);
    if (len >= BUFFER_SIZE) len = BUFFER_SIZE - 1;

    strncpy(buffer, str, len);
    buffer[len] = '\0';

    txBusy = true;
    HAL_UART_Transmit_DMA(uart, reinterpret_cast<uint8_t*>(buffer), len);
}

void UARTDebug::printf(const char* fmt, ...) {
    if (!uart || txBusy) return;

    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buffer, BUFFER_SIZE, fmt, args);
    va_end(args);

    if (len <= 0 || len >= static_cast<int>(BUFFER_SIZE)) return;

    txBusy = true;
    HAL_UART_Transmit_DMA(uart, reinterpret_cast<uint8_t*>(buffer), len);
}

void UARTDebug::onTxComplete(UART_HandleTypeDef* huart) {
    if (huart == uart) {
        txBusy = false;
    }
}
