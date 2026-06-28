#ifndef SERVO_BUS_TRANSPORT_H
#define SERVO_BUS_TRANSPORT_H

#include "servo_bus_protocol.h"
#include "stm32u5xx_hal.h"

#include <stdint.h>

typedef enum
{
  SERVO_BUS_OK = 0,
  SERVO_BUS_INVALID_ARGUMENT,
  SERVO_BUS_UART_ERROR,
  SERVO_BUS_TIMEOUT,
  SERVO_BUS_PROTOCOL_ERROR
} ServoBus_Result;

/** Reinitialize the selected UART with a different baud rate. */
ServoBus_Result ServoBus_SetBaudRate(UART_HandleTypeDef *uart,
                                     uint32_t baud_rate);

/** Send known bytes and receive them through a physical TX-to-RX loopback. */
ServoBus_Result ServoBus_TestUartLoopback(UART_HandleTypeDef *uart,
                                          uint32_t timeout_ms);

/**
 * Send one read-only ping and wait for the matching status packet.
 *
 * This bounded call is intended for controlled commissioning only. Runtime
 * control will use an interrupt- or DMA-driven transport.
 */
ServoBus_Result ServoBus_Ping(UART_HandleTypeDef *uart,
                              uint8_t servo_id,
                              uint32_t timeout_ms,
                              ServoBus_StatusPacket *response);

#endif /* SERVO_BUS_TRANSPORT_H */
