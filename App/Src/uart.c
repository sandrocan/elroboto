/**
 ******************************************************************************
 * @file           : uart.c
 * @author         : Niklas Peter, Sandro Canalicchio
 * @brief          : UART adapters for debug, servo bus and e-skin reception.
 ******************************************************************************
 */

#include "uart.h"

#include <math.h>
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
static volatile char cell_pending_frame[UART_CELL_DATA_SIZE + 1U];
static uint8_t cell_rx_index = 0U;
static bool cell_rx_discarding = false;
static volatile bool cell_recovery_pending = false;
static volatile uint32_t cell_pending_sequence = 0U;
static uint32_t cell_processed_sequence = 0U;

static volatile float *cell_value_pointer = NULL;
static volatile uint32_t cell_received_byte_count = 0U;
static volatile uint32_t cell_valid_frame_count = 0U;
static volatile uint32_t cell_invalid_frame_count = 0U;
static volatile uint32_t cell_uart_error_count = 0U;
static volatile uint32_t cell_receive_restart_failure_count = 0U;
static volatile uint32_t cell_last_uart_error = HAL_UART_ERROR_NONE;
static volatile uint32_t cell_last_valid_frame_ms = 0U;
static volatile uint8_t cell_last_received_byte = 0U;

/* -------------------------------------------------------------------------- */
/* Public functions                                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Attaches the initialized UART handle used by the servo bus adapter.
 * @param huart UART handle to store; may be NULL to detach it.
 * @return None. Updates the module's servo_uart handle.
 */
void UartServo_AttachHandle(UART_HandleTypeDef *huart)
{
    servo_uart = huart;
}

/**
 * @brief Attaches the initialized UART handle used for e-skin reception.
 * @param huart UART handle to store; may be NULL to detach it.
 * @return None. Updates the module's cell_uart handle.
 */
void UartCell_AttachHandle(UART_HandleTypeDef *huart)
{
    cell_uart = huart;
}

/**
 * @brief Sends a null-terminated string through the printf-backed debug port.
 * @param text Text to send; NULL is ignored.
 * @return None. Writes the text to standard output.
 */
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

/**
 * @brief Sends a null-terminated string over the attached servo UART.
 * @param text Text to send; NULL is ignored.
 * @return None. Performs a blocking UART transmission when inputs are valid.
 */
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

/**
 * @brief Sends raw bytes over the attached servo UART.
 * @param data Bytes to transmit.
 * @param length Number of bytes to transmit.
 * @param timeout_ms Maximum blocking transmit duration in milliseconds.
 * @return HAL status from the transmit operation, or HAL_ERROR for invalid input.
 */
HAL_StatusTypeDef UartServo_SendBytes(const uint8_t *data, uint16_t length, uint32_t timeout_ms)
{
    if ((servo_uart == NULL) || (data == NULL) || (length == 0U))
    {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(servo_uart, data, length, timeout_ms);
}

/**
 * @brief Reads raw bytes from the attached servo UART.
 * @param data Buffer receiving the bytes.
 * @param length Number of bytes to read.
 * @param timeout_ms Maximum blocking receive duration in milliseconds.
 * @return HAL status from the receive operation, or HAL_ERROR for invalid input.
 */
HAL_StatusTypeDef UartServo_ReadBytes(uint8_t *data, uint16_t length, uint32_t timeout_ms)
{
    if ((servo_uart == NULL) || (data == NULL) || (length == 0U))
    {
        return HAL_ERROR;
    }

    return HAL_UART_Receive(servo_uart, data, length, timeout_ms);
}

/**
 * @brief Sends a complete servo command packet through the raw-byte adapter.
 * @param command Command packet to transmit.
 * @param length Command length in bytes.
 * @param timeout_ms Maximum blocking transmit duration in milliseconds.
 * @return HAL status from UartServo_SendBytes.
 */
HAL_StatusTypeDef UartServo_SendCommand(const uint8_t *command, uint16_t length, uint32_t timeout_ms)
{
    return UartServo_SendBytes(command, length, timeout_ms);
}

/**
 * @brief Reads a complete servo response through the raw-byte adapter.
 * @param response Buffer receiving the response packet.
 * @param length Expected response length in bytes.
 * @param timeout_ms Maximum blocking receive duration in milliseconds.
 * @return HAL status from UartServo_ReadBytes.
 */
HAL_StatusTypeDef UartServo_ReadResponse(uint8_t *response, uint16_t length, uint32_t timeout_ms)
{
    return UartServo_ReadBytes(response, length, timeout_ms);
}

/**
 * @brief Returns the currently attached servo UART handle.
 * @return Servo UART handle, or NULL when none is attached.
 */
UART_HandleTypeDef *UartServo_GetHandle(void)
{
    return servo_uart;
}

/**
 * @brief Returns the currently attached e-skin UART handle.
 * @return E-skin UART handle, or NULL when none is attached.
 */
UART_HandleTypeDef *UartCell_GetHandle(void)
{
    return cell_uart;
}

/**
 * @brief Resets e-skin receive state and starts interrupt-driven byte reception.
 * @param value Destination updated whenever a valid numeric frame is processed.
 * @return HAL_OK when reception starts, HAL_ERROR for invalid input, or another HAL error.
 */
HAL_StatusTypeDef UartCell_StartReceiveIT(volatile float *value)
{
    if ((cell_uart == NULL) || (value == NULL))
    {
        return HAL_ERROR;
    }

    cell_value_pointer = value;
    cell_rx_index = 0U;
    cell_rx_discarding = false;
    cell_recovery_pending = false;
    cell_pending_sequence = 0U;
    cell_processed_sequence = 0U;
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

/**
 * @brief Processes the latest complete e-skin frame outside interrupt context.
 * @return None. Updates the attached float value and diagnostic counters for valid input.
 */
void UartCell_Process(void)
{
    char frame[UART_CELL_DATA_SIZE + 1U];
    char *parse_end = NULL;
    uint32_t sequence_before;
    uint32_t sequence_after;
    float parsed_value;

    if (cell_value_pointer == NULL)
    {
        return;
    }

    if (cell_recovery_pending)
    {
        HAL_StatusTypeDef restart_status;

        cell_recovery_pending = false;
        (void)HAL_UART_AbortReceive(cell_uart);
        cell_rx_index = 0U;
        cell_rx_discarding = false;

        restart_status = HAL_UART_Receive_IT(cell_uart, &cell_rx_byte, 1U);
        if (restart_status != HAL_OK)
        {
            cell_receive_restart_failure_count++;
            cell_recovery_pending = true;
            return;
        }
    }

    do
    {
        sequence_before = cell_pending_sequence;
        if ((sequence_before == cell_processed_sequence) ||
            ((sequence_before & 1U) != 0U))
        {
            return;
        }

        for (uint8_t i = 0U; i <= UART_CELL_DATA_SIZE; i++)
        {
            frame[i] = cell_pending_frame[i];
        }

        sequence_after = cell_pending_sequence;
    }
    while ((sequence_before != sequence_after) ||
           ((sequence_after & 1U) != 0U));

    parsed_value = strtof(frame, &parse_end);
    cell_processed_sequence = sequence_after;

    if ((parse_end == frame) || (parse_end == NULL) || (*parse_end != '\0') ||
        (!isfinite(parsed_value)))
    {
        cell_invalid_frame_count++;
        return;
    }

    *cell_value_pointer = parsed_value;
    cell_valid_frame_count++;
    cell_last_valid_frame_ms = HAL_GetTick();
}

/**
 * @brief Copies the current e-skin UART diagnostic counters and status.
 * @param diagnostics Destination structure; NULL is ignored.
 * @return None.
 */
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

/**
 * @brief Handles one completed UART byte reception for the attached e-skin UART.
 * @param huart UART instance that completed reception.
 * @return None. Buffers valid frames, updates counters, and rearms byte reception.
 */
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
                cell_pending_sequence++;
                for (uint8_t i = 0U; i < UART_CELL_DATA_SIZE; i++)
                {
                    cell_pending_frame[i] = cell_rx_buffer[i];
                }
                cell_pending_frame[UART_CELL_DATA_SIZE] = '\0';
                cell_pending_sequence++;
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

/**
 * @brief Records an e-skin UART error and requests receive-path recovery.
 * @param huart UART instance that reported the error.
 * @return None. Updates error diagnostics and the deferred recovery flag.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((cell_uart != NULL) && (huart == cell_uart))
    {
        cell_uart_error_count++;
        cell_last_uart_error = HAL_UART_GetError(cell_uart);
        cell_rx_index = 0U;
        cell_rx_discarding = false;
        cell_recovery_pending = true;
    }
}

/**
 * @brief Aborts reception, clears UART error flags, and drains stale servo RX bytes.
 * @return None. Leaves the attached servo receive path empty when a handle is available.
 */
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
