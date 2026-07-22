#ifndef APP_H
#define APP_H

#include "stm32u5xx_hal.h"

#include <stdint.h>

/**
 * @brief Initializes the application after all required hardware is ready.
 * @param servo_uart Initialized UART handle for the servo bus.
 * @param cell_uart Initialized UART handle for e-skin reception.
 * @return None. Initializes application state and communication modules.
 */
void App_Init(UART_HandleTypeDef *servo_uart, UART_HandleTypeDef *cell_uart);

/**
 * @brief Runs one non-blocking application state-machine cycle.
 * @param now_ms Current monotonic HAL time in milliseconds.
 * @return None. Processes pending inputs and updates application state.
 */
void App_Process(uint32_t now_ms);

/**
 * @brief Notifies the application about a user-button interrupt.
 * @return None. Records a pending button event for deferred processing.
 */
void App_OnButtonInterrupt(void);

#endif /* APP_H */
