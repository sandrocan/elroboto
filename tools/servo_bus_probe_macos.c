#include "servo_bus_protocol.h"

#include <IOKit/serial/ioss.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#define RESPONSE_TIMEOUT_MS 20

static int configure_port(int fd, speed_t baud_rate)
{
  struct termios options;

  if (tcgetattr(fd, &options) != 0)
  {
    return -1;
  }

  cfmakeraw(&options);
  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cflag &= (tcflag_t)~CSTOPB;
  options.c_cflag &= (tcflag_t)~PARENB;
  options.c_cflag &= (tcflag_t)~CSIZE;
  options.c_cflag |= CS8;
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  if (tcsetattr(fd, TCSANOW, &options) != 0)
  {
    return -1;
  }
  if (ioctl(fd, IOSSIOSPEED, &baud_rate) != 0)
  {
    return -1;
  }

  return tcflush(fd, TCIOFLUSH);
}

static int ping_id(int fd, uint8_t servo_id, ServoBus_StatusPacket *response)
{
  uint8_t ping[SERVO_BUS_PING_PACKET_SIZE];
  ServoBus_Parser parser;
  struct pollfd poll_descriptor = {.fd = fd, .events = POLLIN, .revents = 0};

  if (ServoBus_BuildPing(servo_id, ping, sizeof(ping)) != sizeof(ping))
  {
    return -1;
  }

  if (tcflush(fd, TCIFLUSH) != 0)
  {
    return -1;
  }
  if (write(fd, ping, sizeof(ping)) != (ssize_t)sizeof(ping))
  {
    return -1;
  }
  if (tcdrain(fd) != 0)
  {
    return -1;
  }

  ServoBus_ParserInit(&parser);

  for (;;)
  {
    const int poll_result = poll(&poll_descriptor, 1U, RESPONSE_TIMEOUT_MS);
    if (poll_result == 0)
    {
      return 0;
    }
    if (poll_result < 0)
    {
      return -1;
    }

    uint8_t received[64];
    const ssize_t received_count = read(fd, received, sizeof(received));
    if (received_count < 0)
    {
      return -1;
    }

    for (ssize_t index = 0; index < received_count; ++index)
    {
      const ServoBus_ParseResult result =
          ServoBus_ParserFeed(&parser, received[index], response);
      if ((result == SERVO_BUS_PARSE_PACKET_READY) &&
          (response->id == servo_id))
      {
        return 1;
      }
    }
  }
}

int main(int argc, char **argv)
{
  static const speed_t baud_rates[] =
  {
    1000000U, 500000U, 250000U, 128000U,
     115200U,  76800U,  57600U,  38400U
  };

  if (argc != 2)
  {
    fprintf(stderr, "Usage: %s /dev/cu.usbmodem...\n", argv[0]);
    return 2;
  }

  const int fd = open(argv[1], O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0)
  {
    fprintf(stderr, "Cannot open %s: %s\n", argv[1], strerror(errno));
    return 1;
  }

  for (size_t baud_index = 0U;
       baud_index < sizeof(baud_rates) / sizeof(baud_rates[0]);
       ++baud_index)
  {
    if (configure_port(fd, baud_rates[baud_index]) != 0)
    {
      fprintf(stderr, "Cannot configure %lu baud: %s\n",
              (unsigned long)baud_rates[baud_index], strerror(errno));
      close(fd);
      return 1;
    }

    printf("Scanning at %lu baud...\n", (unsigned long)baud_rates[baud_index]);
    fflush(stdout);

    for (unsigned int id = 0U; id <= 253U; ++id)
    {
      ServoBus_StatusPacket response;
      const int result = ping_id(fd, (uint8_t)id, &response);

      if (result < 0)
      {
        fprintf(stderr, "Serial error at ID %u: %s\n", id, strerror(errno));
        close(fd);
        return 1;
      }
      if (result > 0)
      {
        printf("Servo found: ID=%u, baud=%lu, status=0x%02X\n",
               id, (unsigned long)baud_rates[baud_index], response.error);
        close(fd);
        return 0;
      }
    }
  }

  printf("No servo found.\n");
  close(fd);
  return 3;
}
