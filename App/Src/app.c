#include "app.h"

#include "servo_bus_transport.h"
#include "stm32u5xx_nucleo.h"

#include <stdio.h>

#define LED_BLINK_SLOW_MS       500U
#define LED_BLINK_FAST_MS       100U
#define BUTTON_DEBOUNCE_MS       50U
#define STATUS_LOG_PERIOD_MS   1000U
#define SERVO_PING_TIMEOUT_MS     10U
#define SERVO_FIRST_ID             0U
#define SERVO_LAST_ID            253U
#define SERVO_SCAN_LOG_STEP        32U
#define SERVO_DEFAULT_BAUD_RATE 1000000U

static uint32_t last_led_toggle_ms;
static uint32_t last_status_log_ms;
static uint32_t last_button_event_ms;
static uint32_t led_blink_period_ms;
static volatile uint8_t button_event_pending;
static UART_HandleTypeDef *servo_bus_uart;
static uint8_t servo_scan_active;
static uint8_t servo_scan_id;

void App_Init(UART_HandleTypeDef *servo_uart)
{
  last_led_toggle_ms = 0U;
  last_status_log_ms = 0U;
  last_button_event_ms = 0U;
  led_blink_period_ms = LED_BLINK_SLOW_MS;
  button_event_pending = 0U;
  servo_bus_uart = servo_uart;
  servo_scan_active = 0U;
  servo_scan_id = SERVO_FIRST_ID;

  printf("\r\nelroboto booted\r\n");
  printf("Press B1 to scan servo IDs 0-253 at 1000000 baud (read-only)\r\n");
}

void App_Process(uint32_t now_ms)
{
  if (button_event_pending != 0U)
  {
    button_event_pending = 0U;

    if ((uint32_t)(now_ms - last_button_event_ms) >= BUTTON_DEBOUNCE_MS)
    {
      last_button_event_ms = now_ms;
      led_blink_period_ms =
          (led_blink_period_ms == LED_BLINK_SLOW_MS)
              ? LED_BLINK_FAST_MS
              : LED_BLINK_SLOW_MS;

      printf("B1 pressed: LED period = %lu ms\r\n",
             (unsigned long)led_blink_period_ms);

      if (servo_bus_uart != NULL)
      {
        if (ServoBus_SetBaudRate(servo_bus_uart,
                                 SERVO_DEFAULT_BAUD_RATE) ==
            SERVO_BUS_OK)
        {
          servo_scan_id = SERVO_FIRST_ID;
          servo_scan_active = 1U;
          printf("Servo scan started at 1000000 baud\r\n");
        }
        else
        {
          printf("Servo scan could not configure UART\r\n");
          servo_scan_active = 0U;
        }
      }
    }
  }

  if (servo_scan_active != 0U)
  {
    ServoBus_StatusPacket response;

    if ((servo_scan_id % SERVO_SCAN_LOG_STEP) == 0U)
    {
      const unsigned int range_end =
          ((unsigned int)servo_scan_id + SERVO_SCAN_LOG_STEP - 1U <=
           SERVO_LAST_ID)
              ? (unsigned int)servo_scan_id + SERVO_SCAN_LOG_STEP - 1U
              : SERVO_LAST_ID;
      printf("Scanning servo IDs %u-%u...\r\n",
             (unsigned int)servo_scan_id, range_end);
    }

    const ServoBus_Result result =
        ServoBus_Ping(servo_bus_uart, servo_scan_id,
                      SERVO_PING_TIMEOUT_MS, &response);

    if (result == SERVO_BUS_OK)
    {
      printf("Servo found: ID=%u, baud=1000000, status=0x%02X\r\n",
             (unsigned int)response.id,
             (unsigned int)response.error);
      servo_scan_active = 0U;
    }
    else if (result == SERVO_BUS_TIMEOUT)
    {
      if (servo_scan_id < SERVO_LAST_ID)
      {
        ++servo_scan_id;
      }
      else
      {
        printf("No servo found at IDs 0-253 and 1000000 baud\r\n");
        servo_scan_active = 0U;
      }
    }
    else
    {
      printf("Servo scan failed at ID=%u: error=%u\r\n",
             (unsigned int)servo_scan_id,
             (unsigned int)result);
      servo_scan_active = 0U;
    }
  }

  if ((uint32_t)(now_ms - last_led_toggle_ms) >= led_blink_period_ms)
  {
    last_led_toggle_ms = now_ms;
    (void)BSP_LED_Toggle(LED_GREEN);
  }

  if ((uint32_t)(now_ms - last_status_log_ms) >= STATUS_LOG_PERIOD_MS)
  {
    last_status_log_ms = now_ms;
    printf("elroboto alive: %lu ms\r\n", (unsigned long)now_ms);
  }
}

void App_OnButtonInterrupt(void)
{
  button_event_pending = 1U;
}
