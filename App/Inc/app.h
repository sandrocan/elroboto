#ifndef APP_H
#define APP_H

#include "stm32u5xx_hal.h"

#include <stdint.h>

/** Initialize the application state after all required hardware is ready. */
void App_Init(UART_HandleTypeDef *servo_uart, UART_HandleTypeDef *cell_uart);

/** Run one non-blocking application cycle. */
void App_Process(uint32_t now_ms);

/** Notify the application about a user-button interrupt. */
void App_OnButtonInterrupt(void);

#endif /* APP_H */
