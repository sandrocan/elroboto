#include "app.h"
#include "kinematics.h"
#include "servo.h"
#include "uart.h"
#include "tests.h"

#include <stdio.h>

#define BUTTON_DEBOUNCE_MS      50U
#define STATUS_LOG_PERIOD_MS  1000U
#define HOME_CHECK_TOLERANCE_TICKS 50U

typedef enum
{
  APP_STATE_INIT = 0,
  APP_STATE_CHECKING_HOME,
  APP_STATE_IDLE,
  APP_STATE_UNLOCKING,
  APP_STATE_HOMING,
  APP_STATE_FAULT
} App_State;

static uint32_t last_status_log_ms;
static uint32_t last_button_event_ms;
static volatile uint8_t button_event_pending;
static App_State app_state;

static void app_set_state(App_State next_state);
static const char *app_state_to_string(App_State state);
static void app_process_button(uint32_t now_ms);
static Servo_Result_t app_unlock_all_joints(void);

void App_Init(UART_HandleTypeDef *servo_uart)
{
  last_status_log_ms = 0U;
  last_button_event_ms = 0U;
  button_event_pending = 0U;
  app_state = APP_STATE_INIT;

  UartServo_AttachHandle(servo_uart);
  Servo_Init();

  printf("\r\nelroboto booted\r\n");

  Servo_Result_t result = Tests_HomeTest();

  if (result != SERVO_RESULT_OK)
  {
    app_set_state(APP_STATE_FAULT);
    printf("Home test failed: result=%s\r\n", Servo_ResultToString(result));
  }

  app_set_state(APP_STATE_IDLE);

  result = Tests_DkTest();

    if (result != SERVO_RESULT_OK)
    {
      printf("DK test failed: result=%s\r\n", Servo_ResultToString(result));
      app_set_state(APP_STATE_FAULT);
      return;
    }

    printf("Startup tests finished successfully\r\n");
    app_set_state(APP_STATE_IDLE);
}

void App_Process(uint32_t now_ms)
{
  app_process_button(now_ms);

  if ((uint32_t)(now_ms - last_status_log_ms) >= STATUS_LOG_PERIOD_MS)
  {
    last_status_log_ms = now_ms;
    printf("elroboto alive: %lu ms, state=%s\r\n",
           (unsigned long)now_ms,
           app_state_to_string(app_state));
  }
}

void App_OnButtonInterrupt(void)
{
  button_event_pending = 1U;
}

static void app_set_state(App_State next_state)
{
  app_state = next_state;
  printf("App state: %s\r\n", app_state_to_string(next_state));
}

static const char *app_state_to_string(App_State state)
{
  switch (state)
  {
    case APP_STATE_INIT:
      return "INIT";

    case APP_STATE_CHECKING_HOME:
      return "CHECKING_HOME";

    case APP_STATE_IDLE:
      return "IDLE";

    case APP_STATE_UNLOCKING:
      return "UNLOCKING";

    case APP_STATE_HOMING:
      return "HOMING";

    case APP_STATE_FAULT:
      return "FAULT";

    default:
      return "UNKNOWN";
  }
}

static void app_process_button(uint32_t now_ms)
{
  if (button_event_pending == 0U)
  {
    return;
  }

  button_event_pending = 0U;

  if ((uint32_t)(now_ms - last_button_event_ms) < BUTTON_DEBOUNCE_MS)
  {
    return;
  }

  if (app_state == APP_STATE_IDLE)
  {
    (void)app_unlock_all_joints();
  }
  else
  {
    printf("B1 ignored while state=%s\r\n", app_state_to_string(app_state));
  }

  last_button_event_ms = now_ms;
}

static Servo_Result_t app_unlock_all_joints(void)
{
  const uint8_t joint_count = Servo_GetJointCount();
  Servo_Result_t first_error = SERVO_RESULT_OK;

  printf("Button unlock all joints started\r\n");

  for (uint8_t index = 0U; index < joint_count; ++index)
  {
    const Servo_JointConfig_t *joint = Servo_GetJointConfigByIndex(index);

    if (joint == NULL)
    {
      printf("Unlock failed: missing joint at index=%u\r\n",
             (unsigned int)index);

      if (first_error == SERVO_RESULT_OK)
      {
        first_error = SERVO_RESULT_UNKNOWN_JOINT_ID;
      }

      continue;
    }

    const Servo_Result_t result = Servo_UnlockJoint(joint->id);

    printf("Unlock joint: ID=%u %-14s result=%s\r\n",
           (unsigned int)joint->id,
           joint->name,
           Servo_ResultToString(result));

    if ((result != SERVO_RESULT_OK) && (first_error == SERVO_RESULT_OK))
    {
      first_error = result;
    }
  }

  printf("Button unlock all joints finished: result=%s\r\n",
         Servo_ResultToString(first_error));

  return first_error;
}
