#ifndef SERVO_BUS_PROTOCOL_H
#define SERVO_BUS_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#define SERVO_BUS_HEADER_BYTE       0xFFU
#define SERVO_BUS_INSTRUCTION_PING  0x01U
#define SERVO_BUS_MAX_PARAMETERS    32U
#define SERVO_BUS_PING_PACKET_SIZE   6U
#define SERVO_BUS_PING_STATUS_SIZE   6U

typedef struct
{
  uint8_t id;
  uint8_t error;
  uint8_t parameters[SERVO_BUS_MAX_PARAMETERS];
  size_t parameter_count;
} ServoBus_StatusPacket;

typedef enum
{
  SERVO_BUS_PARSE_INCOMPLETE = 0,
  SERVO_BUS_PARSE_PACKET_READY,
  SERVO_BUS_PARSE_INVALID_LENGTH,
  SERVO_BUS_PARSE_CHECKSUM_ERROR
} ServoBus_ParseResult;

typedef struct
{
  uint8_t state;
  uint8_t id;
  uint8_t length;
  uint8_t payload[SERVO_BUS_MAX_PARAMETERS + 2U];
  size_t payload_count;
} ServoBus_Parser;

/** Build a read-only ping instruction packet for one servo ID. */
size_t ServoBus_BuildPing(uint8_t servo_id,
                          uint8_t *packet,
                          size_t packet_capacity);

/** Reset a streaming status-packet parser. */
void ServoBus_ParserInit(ServoBus_Parser *parser);

/** Feed one received byte into the parser. */
ServoBus_ParseResult ServoBus_ParserFeed(ServoBus_Parser *parser,
                                         uint8_t byte,
                                         ServoBus_StatusPacket *packet);

#endif /* SERVO_BUS_PROTOCOL_H */
