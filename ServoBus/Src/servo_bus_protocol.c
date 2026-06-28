#include "servo_bus_protocol.h"

enum
{
  PARSER_WAIT_HEADER_1 = 0,
  PARSER_WAIT_HEADER_2,
  PARSER_WAIT_ID,
  PARSER_WAIT_LENGTH,
  PARSER_WAIT_PAYLOAD
};

static uint8_t calculate_checksum(const uint8_t *bytes, size_t count)
{
  uint8_t sum = 0U;

  for (size_t index = 0U; index < count; ++index)
  {
    sum = (uint8_t)(sum + bytes[index]);
  }

  return (uint8_t)(~sum);
}

static void reset_after_error(ServoBus_Parser *parser, uint8_t byte)
{
  parser->state = (byte == SERVO_BUS_HEADER_BYTE)
                      ? PARSER_WAIT_HEADER_2
                      : PARSER_WAIT_HEADER_1;
  parser->payload_count = 0U;
}

size_t ServoBus_BuildPing(uint8_t servo_id,
                          uint8_t *packet,
                          size_t packet_capacity)
{
  if ((packet == NULL) ||
      (packet_capacity < SERVO_BUS_PING_PACKET_SIZE) ||
      (servo_id == SERVO_BUS_HEADER_BYTE))
  {
    return 0U;
  }

  packet[0] = SERVO_BUS_HEADER_BYTE;
  packet[1] = SERVO_BUS_HEADER_BYTE;
  packet[2] = servo_id;
  packet[3] = 2U;
  packet[4] = SERVO_BUS_INSTRUCTION_PING;
  packet[5] = calculate_checksum(&packet[2], 3U);

  return SERVO_BUS_PING_PACKET_SIZE;
}

void ServoBus_ParserInit(ServoBus_Parser *parser)
{
  if (parser != NULL)
  {
    parser->state = PARSER_WAIT_HEADER_1;
    parser->id = 0U;
    parser->length = 0U;
    parser->payload_count = 0U;
  }
}

ServoBus_ParseResult ServoBus_ParserFeed(ServoBus_Parser *parser,
                                         uint8_t byte,
                                         ServoBus_StatusPacket *packet)
{
  if ((parser == NULL) || (packet == NULL))
  {
    return SERVO_BUS_PARSE_INVALID_LENGTH;
  }

  switch (parser->state)
  {
    case PARSER_WAIT_HEADER_1:
      if (byte == SERVO_BUS_HEADER_BYTE)
      {
        parser->state = PARSER_WAIT_HEADER_2;
      }
      break;

    case PARSER_WAIT_HEADER_2:
      if (byte == SERVO_BUS_HEADER_BYTE)
      {
        parser->state = PARSER_WAIT_ID;
      }
      else
      {
        parser->state = PARSER_WAIT_HEADER_1;
      }
      break;

    case PARSER_WAIT_ID:
      if (byte == SERVO_BUS_HEADER_BYTE)
      {
        parser->state = PARSER_WAIT_ID;
      }
      else
      {
        parser->id = byte;
        parser->state = PARSER_WAIT_LENGTH;
      }
      break;

    case PARSER_WAIT_LENGTH:
      if ((byte < 2U) || (byte > (SERVO_BUS_MAX_PARAMETERS + 2U)))
      {
        reset_after_error(parser, byte);
        return SERVO_BUS_PARSE_INVALID_LENGTH;
      }

      parser->length = byte;
      parser->payload_count = 0U;
      parser->state = PARSER_WAIT_PAYLOAD;
      break;

    case PARSER_WAIT_PAYLOAD:
      parser->payload[parser->payload_count++] = byte;

      if (parser->payload_count == parser->length)
      {
        uint8_t checksum_bytes[SERVO_BUS_MAX_PARAMETERS + 3U];
        const size_t checksum_data_count = (size_t)parser->length + 1U;

        checksum_bytes[0] = parser->id;
        checksum_bytes[1] = parser->length;
        for (size_t index = 0U; index < (size_t)parser->length - 1U; ++index)
        {
          checksum_bytes[index + 2U] = parser->payload[index];
        }

        if (calculate_checksum(checksum_bytes, checksum_data_count) !=
            parser->payload[parser->length - 1U])
        {
          ServoBus_ParserInit(parser);
          return SERVO_BUS_PARSE_CHECKSUM_ERROR;
        }

        packet->id = parser->id;
        packet->error = parser->payload[0];
        packet->parameter_count = (size_t)parser->length - 2U;
        for (size_t index = 0U; index < packet->parameter_count; ++index)
        {
          packet->parameters[index] = parser->payload[index + 1U];
        }

        ServoBus_ParserInit(parser);
        return SERVO_BUS_PARSE_PACKET_READY;
      }
      break;

    default:
      ServoBus_ParserInit(parser);
      break;
  }

  return SERVO_BUS_PARSE_INCOMPLETE;
}
