/**
 ******************************************************************************
 * @file           : uart.h / uart.c
 * @author         : Niklas Peter
 * @brief          : All functions and variables for configuring the UART on the STM/nucleo board and enabling communication
 ******************************************************************************
 */

#ifndef UART_H_
#define UART_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32u5xx_hal.h"
#include <stddef.h>
#include <stdint.h>


#define UART_DEBUG_BAUDRATE    115200U
#define UART_SERVO_BAUDRATE   1000000U

/**
 * @brief Initializes the debug UART through the ST-LINK virtual COM port.
 * @return None.
 */
void UartDebug_Init(void);

/**
 * @brief Attaches the CubeMX-created UART handle used for the servo bus.
 * @param huart Pointer to the initialized LPUART1 handle.
 * @return None.
 */
void UartServo_AttachHandle(UART_HandleTypeDef *huart);

/**
 * @brief Initializes LPUART1 for servo bus communication.
 *
 * In the integrated firmware LPUART1 is initialized by CubeMX-generated code.
 * This function is kept as a compatibility hook and does not change pins,
 * clocks or baud rate.
 *
 * @return None.
 */
void UartServo_Init(void);

/**
 * @brief Sends a string over the debug UART.
 * @param text Null-terminated string to send.
 * @return None.
 */
void UartDebug_SendString(const char *text);

/**
 * @brief Sends a string over the servo UART.
 * @param text Null-terminated string to send.
 * @return None.
 */
void UartServo_SendString(const char *text);

/**
 * @brief Sends raw bytes over the servo UART.
 * @param data Pointer to the bytes to send.
 * @param length Number of bytes to send.
 * @param timeout_ms UART transmit timeout in milliseconds.
 * @return HAL status of the transmit operation.
 */
HAL_StatusTypeDef UartServo_SendBytes(const uint8_t *data, uint16_t length, uint32_t timeout_ms);

/**
 * @brief Reads raw bytes from the servo UART.
 * @param data Pointer to the receive buffer.
 * @param length Number of bytes to read.
 * @param timeout_ms UART receive timeout in milliseconds.
 * @return HAL status of the receive operation.
 */
HAL_StatusTypeDef UartServo_ReadBytes(uint8_t *data, uint16_t length, uint32_t timeout_ms);

/**
 * @brief Sends a servo command packet over UART.
 * @param command Pointer to the command packet.
 * @param length Command packet length in bytes.
 * @param timeout_ms UART transmit timeout in milliseconds.
 * @return HAL status of the transmit operation.
 */
HAL_StatusTypeDef UartServo_SendCommand(const uint8_t *command, uint16_t length, uint32_t timeout_ms);

/**
 * @brief Reads a servo response packet from UART.
 * @param response Pointer to the response buffer.
 * @param length Number of bytes to read.
 * @param timeout_ms UART receive timeout in milliseconds.
 * @return HAL status of the receive operation.
 */
HAL_StatusTypeDef UartServo_ReadResponse(uint8_t *response, uint16_t length, uint32_t timeout_ms);

/**
 * @brief Returns the UART handle used for the servo bus.
 * @return Pointer to the LPUART1 handle.
 */
UART_HandleTypeDef *UartServo_GetHandle(void);

/**
 * @brief Clears pending bytes and error flags from the servo UART receive path.
 * @param None.
 * @return None.
 */
void UartServo_ClearRxBuffer(void);


#ifdef __cplusplus
}
#endif

#endif /* UART_H_ */
