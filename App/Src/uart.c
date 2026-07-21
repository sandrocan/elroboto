#include "uart.h"

#include <stdio.h>
#include <string.h>

#include <stdbool.h>
#include <stdlib.h>

#define UART_CELL_DATA_SIZE 5U



/* -------------------------------------------------------------------------- */
/* Private variables                                                          */
/* -------------------------------------------------------------------------- */

static UART_HandleTypeDef *servo_uart = NULL;
static UART_HandleTypeDef *cell_uart = NULL;

static uint8_t cell_rx_byte;
static char cell_rx_buffer[UART_CELL_DATA_SIZE + 1U];
static uint8_t cell_rx_index = 0U;
static bool cell_rx_discarding = false;

static volatile float *cell_value_pointer = NULL;
static volatile bool cell_value_ready = false;
static volatile uint32_t cell_received_byte_count = 0U;
static volatile uint32_t cell_valid_frame_count = 0U;
static volatile uint32_t cell_invalid_frame_count = 0U;
static volatile uint32_t cell_uart_error_count = 0U;
static volatile uint32_t cell_receive_restart_failure_count = 0U;
static volatile uint32_t cell_last_uart_error = HAL_UART_ERROR_NONE;
static volatile uint32_t cell_last_valid_frame_ms = 0U;
static volatile uint8_t cell_last_received_byte = 0U;

/* -------------------------------------------------------------------------- */
/* Public functions                                                      */
/* -------------------------------------------------------------------------- */

void UartServo_AttachHandle(UART_HandleTypeDef *huart)
{
    servo_uart = huart;
}

void UartCell_AttachHandle(UART_HandleTypeDef *huart)
{
    cell_uart = huart;
}

void UartDebug_SendString(const char *text)
{
    if (text == NULL)
    {
        return;
    }

    /*
     * BSP_COM_Init() sets up the ST-LINK Virtual COM Port.
     * With USE_COM_LOG enabled in stm32u5xx_nucleo_conf.h,
     * printf is routed to COM1.
     */
    printf("%s", text);
}

void UartServo_SendString(const char *text)
{
    if ((servo_uart == NULL) || (text == NULL))
    {
        return;
    }

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

UART_HandleTypeDef *UartCell_GetHandle(void)
{
    return cell_uart;
}

HAL_StatusTypeDef UartCell_StartReceiveIT(volatile float *value)
{
    if ((cell_uart == NULL) || (value == NULL))
    {
        return HAL_ERROR;
    }

    cell_value_pointer = value;
    cell_rx_index = 0U;
    cell_rx_discarding = false;
    cell_value_ready = false;
    cell_received_byte_count = 0U;
    cell_valid_frame_count = 0U;
    cell_invalid_frame_count = 0U;
    cell_uart_error_count = 0U;
    cell_receive_restart_failure_count = 0U;
    cell_last_uart_error = HAL_UART_ERROR_NONE;
    cell_last_valid_frame_ms = 0U;
    cell_last_received_byte = 0U;

    return HAL_UART_Receive_IT(
        cell_uart,
        &cell_rx_byte,
        1U
    );
}

void UartCell_GetDiagnostics(UartCell_Diagnostics_t *diagnostics)
{
    if (diagnostics == NULL)
    {
        return;
    }

    diagnostics->received_byte_count = cell_received_byte_count;
    diagnostics->valid_frame_count = cell_valid_frame_count;
    diagnostics->invalid_frame_count = cell_invalid_frame_count;
    diagnostics->uart_error_count = cell_uart_error_count;
    diagnostics->receive_restart_failure_count = cell_receive_restart_failure_count;
    diagnostics->last_uart_error = cell_last_uart_error;
    diagnostics->last_valid_frame_ms = cell_last_valid_frame_ms;
    diagnostics->last_received_byte = cell_last_received_byte;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((cell_uart != NULL) && (huart == cell_uart))
    {
        HAL_StatusTypeDef restart_status;

        cell_received_byte_count++;
        cell_last_received_byte = cell_rx_byte;

        if (cell_rx_byte == '\n')
        {
            if ((!cell_rx_discarding) &&
                (cell_rx_index == UART_CELL_DATA_SIZE) &&
                (cell_value_pointer != NULL))
            {
                cell_rx_buffer[UART_CELL_DATA_SIZE] = '\0';

                *cell_value_pointer = strtof(cell_rx_buffer, NULL);
                cell_value_ready = true;
                cell_valid_frame_count++;
                cell_last_valid_frame_ms = HAL_GetTick();
            }
            else
            {
                cell_invalid_frame_count++;
            }

            cell_rx_index = 0U;
            cell_rx_discarding = false;
        }
        else if (cell_rx_byte != '\r')
        {
            if ((!cell_rx_discarding) &&
                (cell_rx_index < UART_CELL_DATA_SIZE))
            {
                cell_rx_buffer[cell_rx_index++] = (char)cell_rx_byte;
            }
            else
            {
                cell_rx_discarding = true;
            }
        }

        restart_status = HAL_UART_Receive_IT(
            cell_uart,
            &cell_rx_byte,
            1U
        );

        if (restart_status != HAL_OK)
        {
            cell_receive_restart_failure_count++;
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((cell_uart != NULL) && (huart == cell_uart))
    {
        cell_uart_error_count++;
        cell_last_uart_error = HAL_UART_GetError(cell_uart);
        cell_rx_index = 0U;
        cell_rx_discarding = false;

        if (HAL_UART_Receive_IT(cell_uart, &cell_rx_byte, 1U) != HAL_OK)
        {
            cell_receive_restart_failure_count++;
        }
    }
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
