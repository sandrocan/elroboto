#include "app.h"

#include "stm32u5xx_nucleo.h"

#include <stdio.h>

#define LED_BLINK_SLOW_MS       500U
#define LED_BLINK_FAST_MS       100U
#define BUTTON_DEBOUNCE_MS       50U
#define STATUS_LOG_PERIOD_MS   1000U

static uint32_t last_led_toggle_ms;
static uint32_t last_status_log_ms;
static uint32_t last_button_event_ms;
static uint32_t led_blink_period_ms;
static volatile uint8_t button_event_pending;

void App_Init(void)
{
  last_led_toggle_ms = 0U;
  last_status_log_ms = 0U;
  last_button_event_ms = 0U;
  led_blink_period_ms = LED_BLINK_SLOW_MS;
  button_event_pending = 0U;

  printf("\r\nelroboto booted\r\n");
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
