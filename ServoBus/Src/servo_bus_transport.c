#include "servo_bus_transport.h"

ServoBus_Result ServoBus_SetBaudRate(UART_HandleTypeDef *uart,
                                     uint32_t baud_rate)
{
  if ((uart == NULL) || (baud_rate == 0U))
  {
    return SERVO_BUS_INVALID_ARGUMENT;
  }

  if (HAL_UART_DeInit(uart) != HAL_OK)
  {
    return SERVO_BUS_UART_ERROR;
  }

  uart->Init.BaudRate = baud_rate;
  if (HAL_UART_Init(uart) != HAL_OK)
  {
    return SERVO_BUS_UART_ERROR;
  }

  return SERVO_BUS_OK;
}

ServoBus_Result ServoBus_TestUartLoopback(UART_HandleTypeDef *uart,
                                          uint32_t timeout_ms)
{
  static const uint8_t test_bytes[] =
  {
    0x55U, 0xAAU, 0x00U, 0xFFU, 0x81U, 0x7EU
  };

  if ((uart == NULL) || (timeout_ms == 0U))
  {
    return SERVO_BUS_INVALID_ARGUMENT;
  }

  __HAL_UART_CLEAR_OREFLAG(uart);
  __HAL_UART_FLUSH_DRREGISTER(uart);

  for (size_t index = 0U; index < sizeof(test_bytes); ++index)
  {
    uint8_t received_byte = 0U;

    if (HAL_UART_Transmit(uart, (uint8_t *)&test_bytes[index], 1U,
                          timeout_ms) != HAL_OK)
    {
      return SERVO_BUS_UART_ERROR;
    }

    const HAL_StatusTypeDef receive_result =
        HAL_UART_Receive(uart, &received_byte, 1U, timeout_ms);

    if (receive_result == HAL_TIMEOUT)
    {
      return SERVO_BUS_TIMEOUT;
    }
    if (receive_result != HAL_OK)
    {
      return SERVO_BUS_UART_ERROR;
    }
    if (received_byte != test_bytes[index])
    {
      return SERVO_BUS_PROTOCOL_ERROR;
    }
  }

  return SERVO_BUS_OK;
}

ServoBus_Result ServoBus_Ping(UART_HandleTypeDef *uart,
                              uint8_t servo_id,
                              uint32_t timeout_ms,
                              ServoBus_StatusPacket *response)
{
  uint8_t ping_packet[SERVO_BUS_PING_PACKET_SIZE];
  uint8_t response_bytes[SERVO_BUS_PING_STATUS_SIZE];
  ServoBus_Parser parser;

  if ((uart == NULL) || (response == NULL) || (timeout_ms == 0U))
  {
    return SERVO_BUS_INVALID_ARGUMENT;
  }

  if (ServoBus_BuildPing(servo_id, ping_packet, sizeof(ping_packet)) == 0U)
  {
    return SERVO_BUS_INVALID_ARGUMENT;
  }

  if (HAL_UART_Transmit(uart, ping_packet, sizeof(ping_packet), timeout_ms) !=
      HAL_OK)
  {
    return SERVO_BUS_UART_ERROR;
  }

  ServoBus_ParserInit(&parser);

  const HAL_StatusTypeDef receive_result =
      HAL_UART_Receive(uart, response_bytes, sizeof(response_bytes),
                       timeout_ms);

  if (receive_result == HAL_TIMEOUT)
  {
    return SERVO_BUS_TIMEOUT;
  }
  if (receive_result != HAL_OK)
  {
    return SERVO_BUS_UART_ERROR;
  }

  for (size_t index = 0U; index < sizeof(response_bytes); ++index)
  {
    const ServoBus_ParseResult parse_result =
        ServoBus_ParserFeed(&parser, response_bytes[index], response);

    if (parse_result == SERVO_BUS_PARSE_PACKET_READY)
    {
      return (response->id == servo_id)
                 ? SERVO_BUS_OK
                 : SERVO_BUS_PROTOCOL_ERROR;
    }
    else if ((parse_result == SERVO_BUS_PARSE_INVALID_LENGTH) ||
             (parse_result == SERVO_BUS_PARSE_CHECKSUM_ERROR))
    {
      return SERVO_BUS_PROTOCOL_ERROR;
    }
  }

  return SERVO_BUS_PROTOCOL_ERROR;
}
