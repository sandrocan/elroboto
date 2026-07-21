/**
 ******************************************************************************
 * @file           : uart.c
 * @author         : Niklas Peter
 * @brief          : All functions and variables for configuring the UART on the
 *                      STM/nucleo board and enabling communication. Most func-
 *                      tions are implemented via HAL drivers already but are ab-
 *                      stracted to make usage more clear and easy.
 ******************************************************************************
 */

#include "uart.h"

#include <stdio.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Private variables                                                          */
/* -------------------------------------------------------------------------- */

static UART_HandleTypeDef *servo_uart = NULL;

/* -------------------------------------------------------------------------- */
/* Public functions                                                           */
/* -------------------------------------------------------------------------- */

void UartServo_AttachHandle(UART_HandleTypeDef *huart)
{
    servo_uart = huart;
}

void UartDebug_SendString(const char *text)
{
    if (text == NULL)
    {
        return;
    }

    //printf is routed to the COM1 UART, so we can use it directly aswell
    printf("%s", text);
}

void UartServo_SendString(const char *text)
{
    if ((servo_uart == NULL) || (text == NULL))
    {
        return;
    }

    //HAL transmit
    (void)HAL_UART_Transmit(
        servo_uart,
        (const uint8_t *)text,
        (uint16_t)strlen(text),
        HAL_MAX_DELAY
    );
}

HAL_StatusTypeDef UartServo_SendBytes(const uint8_t *data, uint16_t length, uint32_t timeout_ms)
{
    if ((servo_uart == NULL) || (data == NULL) || (length == 0U))
    {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(servo_uart, data, length, timeout_ms);
}

HAL_StatusTypeDef UartServo_ReadBytes(uint8_t *data, uint16_t length, uint32_t timeout_ms)
{
    if ((servo_uart == NULL) || (data == NULL) || (length == 0U))
    {
        return HAL_ERROR;
    }

    return HAL_UART_Receive(servo_uart, data, length, timeout_ms);
}

HAL_StatusTypeDef UartServo_SendCommand(const uint8_t *command, uint16_t length, uint32_t timeout_ms)
{
    return UartServo_SendBytes(command, length, timeout_ms);
}

HAL_StatusTypeDef UartServo_ReadResponse(uint8_t *response, uint16_t length, uint32_t timeout_ms)
{
    return UartServo_ReadBytes(response, length, timeout_ms);
}

UART_HandleTypeDef *UartServo_GetHandle(void)
{
    return servo_uart;
}

void UartServo_ClearRxBuffer(void)
{
    uint8_t dummy;

    if (servo_uart == NULL)
    {
        return;
    }

    (void)HAL_UART_AbortReceive(servo_uart);

    //Clear UART ERROR flags
    __HAL_UART_CLEAR_OREFLAG(servo_uart);
    __HAL_UART_CLEAR_FEFLAG(servo_uart);
    __HAL_UART_CLEAR_NEFLAG(servo_uart);
    __HAL_UART_CLEAR_PEFLAG(servo_uart);

    while (HAL_UART_Receive(servo_uart, &dummy, 1U, 1U) == HAL_OK)
    {
        //Drain stale RX bytes
    }
}
