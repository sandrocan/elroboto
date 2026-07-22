/**
 ******************************************************************************
 * @file           : uart.h
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

#include <stdbool.h>



#define UART_DEBUG_BAUDRATE    115200U
#define UART_SERVO_BAUDRATE   1000000U

typedef struct
{
    uint32_t received_byte_count;
    uint32_t valid_frame_count;
    uint32_t invalid_frame_count;
    uint32_t uart_error_count;
    uint32_t receive_restart_failure_count;
    uint32_t last_uart_error;
    uint32_t last_valid_frame_ms;
    uint8_t last_received_byte;
} UartCell_Diagnostics_t;

/**
 * @brief Attaches the CubeMX-created UART handle used for e-skin reception.
 * @param huart Pointer to the initialized e-skin UART handle.
 * @return None.
 */
void UartCell_AttachHandle(UART_HandleTypeDef *huart);

/**
 * @brief Starts interrupt-driven e-skin frame reception.
 * @param value Destination updated with each valid parsed frame.
 * @return HAL status of the receive-start operation.
 */
HAL_StatusTypeDef UartCell_StartReceiveIT(volatile float *value);

/**
 * @brief Processes a pending e-skin frame outside interrupt context.
 * @return None. Updates the attached value and diagnostic counters.
 */
void UartCell_Process(void);

/**
 * @brief Copies the current e-skin receive diagnostics.
 * @param diagnostics Destination structure; NULL is ignored.
 * @return None.
 */
void UartCell_GetDiagnostics(UartCell_Diagnostics_t *diagnostics);

/**
 * @brief Attaches the CubeMX-created UART handle used for the servo bus.
 * @param huart Pointer to the initialized LPUART1 handle.
 * @return None.
 */
void UartServo_AttachHandle(UART_HandleTypeDef *huart);

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
