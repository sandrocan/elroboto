#include "servo_bus_protocol.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>

static ServoBus_ParseResult feed_packet(ServoBus_Parser *parser,
                                        const uint8_t *bytes,
                                        size_t count,
                                        ServoBus_StatusPacket *packet)
{
  ServoBus_ParseResult result = SERVO_BUS_PARSE_INCOMPLETE;

  for (size_t index = 0U; index < count; ++index)
  {
    result = ServoBus_ParserFeed(parser, bytes[index], packet);
  }

  return result;
}

int main(void)
{
  uint8_t ping[SERVO_BUS_PING_PACKET_SIZE] = {0U};
  const uint8_t expected_ping[] = {0xFFU, 0xFFU, 0x01U,
                                   0x02U, 0x01U, 0xFBU};

  assert(ServoBus_BuildPing(1U, ping, sizeof(ping)) == sizeof(ping));
  for (size_t index = 0U; index < sizeof(ping); ++index)
  {
    assert(ping[index] == expected_ping[index]);
  }

  ServoBus_Parser parser;
  ServoBus_StatusPacket packet;
  const uint8_t valid_response[] = {0xFFU, 0xFFU, 0x01U,
                                    0x02U, 0x00U, 0xFCU};

  ServoBus_ParserInit(&parser);
  assert(feed_packet(&parser, valid_response, 3U, &packet) ==
         SERVO_BUS_PARSE_INCOMPLETE);
  assert(feed_packet(&parser, &valid_response[3], 3U, &packet) ==
         SERVO_BUS_PARSE_PACKET_READY);
  assert(packet.id == 1U);
  assert(packet.error == 0U);
  assert(packet.parameter_count == 0U);

  const uint8_t invalid_checksum[] = {0xFFU, 0xFFU, 0x01U,
                                     0x02U, 0x00U, 0x00U};
  assert(feed_packet(&parser, invalid_checksum, sizeof(invalid_checksum),
                     &packet) == SERVO_BUS_PARSE_CHECKSUM_ERROR);

  const uint8_t invalid_length[] = {0xFFU, 0xFFU, 0x01U, 0x01U};
  assert(feed_packet(&parser, invalid_length, sizeof(invalid_length),
                     &packet) == SERVO_BUS_PARSE_INVALID_LENGTH);

  return 0;
}
