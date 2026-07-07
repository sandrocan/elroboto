#include "app.h"

#include "servo.h"
#include "uart.h"

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
static void app_log_joint_table(void);
static void app_process_button(uint32_t now_ms);
static Servo_Result_t app_check_all_joints_home(uint8_t *all_home);
static uint16_t app_abs_diff_u16(uint16_t first, uint16_t second);
static Servo_Result_t app_unlock_all_joints(void);
static Servo_Result_t app_drive_home(void);

void App_Init(UART_HandleTypeDef *servo_uart)
{
  last_status_log_ms = 0U;
  last_button_event_ms = 0U;
  button_event_pending = 0U;
  app_state = APP_STATE_INIT;

  UartServo_AttachHandle(servo_uart);
  Servo_Init();

  printf("\r\nelroboto booted\r\n");
  printf("Startup test: check home, unlock, then wait for B1 to drive home\r\n");
  app_log_joint_table();

  uint8_t all_home = 0U;
  app_set_state(APP_STATE_CHECKING_HOME);
  Servo_Result_t startup_result = app_check_all_joints_home(&all_home);

  if (startup_result != SERVO_RESULT_OK)
  {
    printf("Startup home check failed: result=%s\r\n",
           Servo_ResultToString(startup_result));
    app_set_state(APP_STATE_FAULT);
    return;
  }

  if (all_home != 0U)
  {
    printf("Startup home check: all joints are already home\r\n");
  }
  else
  {
    printf("Startup home check: at least one joint is not home; waiting for B1\r\n");
  }

  app_set_state(APP_STATE_UNLOCKING);
  const Servo_Result_t unlock_result = app_unlock_all_joints();

  if (unlock_result != SERVO_RESULT_OK)
  {
    printf("Startup unlock failed: result=%s\r\n",
           Servo_ResultToString(unlock_result));
    app_set_state(APP_STATE_FAULT);
    return;
  }

  printf("Startup unlock succeeded; move the arm manually, then press B1\r\n");
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

static void app_log_joint_table(void)
{
  const uint8_t joint_count = Servo_GetJointCount();

  printf("Configured servo joints: %u\r\n", (unsigned int)joint_count);

  for (uint8_t index = 0U; index < joint_count; ++index)
  {
    const Servo_JointConfig_t *joint = Servo_GetJointConfigByIndex(index);

    if (joint == NULL)
    {
      continue;
    }

    printf("  ID=%u %-14s min=%u home=%u max=%u fixed=%u\r\n",
           (unsigned int)joint->id,
           joint->name,
           (unsigned int)joint->min_position_ticks,
           (unsigned int)joint->home_position_ticks,
           (unsigned int)joint->max_position_ticks,
           (unsigned int)joint->is_fixed);
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

  last_button_event_ms = now_ms;

  if (app_state == APP_STATE_IDLE)
  {
    (void)app_drive_home();
  }
  else
  {
    printf("B1 ignored while state=%s\r\n", app_state_to_string(app_state));
  }
}

static Servo_Result_t app_check_all_joints_home(uint8_t *all_home)
{
  const uint8_t joint_count = Servo_GetJointCount();

  if (all_home == NULL)
  {
    return SERVO_RESULT_NULL_POINTER;
  }

  *all_home = 1U;
  printf("Startup home check started: tolerance=%u ticks\r\n",
         (unsigned int)HOME_CHECK_TOLERANCE_TICKS);

  for (uint8_t index = 0U; index < joint_count; ++index)
  {
    const Servo_JointConfig_t *joint = Servo_GetJointConfigByIndex(index);
    uint16_t current_position_ticks = 0U;

    if (joint == NULL)
    {
      printf("Startup home check failed: missing joint at index=%u\r\n",
             (unsigned int)index);
      return SERVO_RESULT_UNKNOWN_JOINT_ID;
    }

    const Servo_Result_t result =
        Servo_ReadPosition(joint->id, &current_position_ticks);

    if (result != SERVO_RESULT_OK)
    {
      printf("Startup home check read failed: ID=%u %-14s result=%s\r\n",
             (unsigned int)joint->id,
             joint->name,
             Servo_ResultToString(result));
      return result;
    }

    const uint16_t error_ticks =
        app_abs_diff_u16(current_position_ticks, joint->home_position_ticks);

    printf("Startup home check: ID=%u %-14s current=%u home=%u error=%u\r\n",
           (unsigned int)joint->id,
           joint->name,
           (unsigned int)current_position_ticks,
           (unsigned int)joint->home_position_ticks,
           (unsigned int)error_ticks);

    if (error_ticks > HOME_CHECK_TOLERANCE_TICKS)
    {
      *all_home = 0U;
    }
  }

  return SERVO_RESULT_OK;
}

static uint16_t app_abs_diff_u16(uint16_t first, uint16_t second)
{
  if (first >= second)
  {
    return (uint16_t)(first - second);
  }

  return (uint16_t)(second - first);
}

static Servo_Result_t app_unlock_all_joints(void)
{
  const uint8_t joint_count = Servo_GetJointCount();
  Servo_Result_t first_error = SERVO_RESULT_OK;

  printf("Unlock all joints started\r\n");

  for (uint8_t index = 0U; index < joint_count; ++index)
  {
    const Servo_JointConfig_t *joint = Servo_GetJointConfigByIndex(index);

    if (joint == NULL)
    {
      first_error = SERVO_RESULT_UNKNOWN_JOINT_ID;
      printf("Unlock failed: joint table entry missing at index=%u\r\n",
             (unsigned int)index);
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

  printf("Unlock all joints finished: result=%s\r\n",
         Servo_ResultToString(first_error));
  return first_error;
}

static Servo_Result_t app_drive_home(void)
{
  app_set_state(APP_STATE_HOMING);
  printf("Drive home started\r\n");

  const Servo_Result_t result = Servo_DriveHome();

  printf("Drive home finished: result=%s\r\n",
         Servo_ResultToString(result));

  app_set_state((result == SERVO_RESULT_OK) ? APP_STATE_IDLE : APP_STATE_FAULT);
  return result;
}
