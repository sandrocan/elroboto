#include "app.h"
#include "kinematics.h"
#include "servo.h"
#include "uart.h"

#include <stdio.h>
#include <stdint.h>

#define BUTTON_DEBOUNCE_MS      50U
#define HOME_CHECK_TOLERANCE_TICKS 50U

#define APP_MOVEMENT_SPEED              300U
#define APP_MOVEMENT_ACCELERATION       50U
#define APP_CONTROL_TOLERANCE_TICKS     10U
#define APP_SQUARE_RADIUS_CM            12.0f

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
static volatile uint8_t button_event_pending;
static volatile uint32_t last_button_event_ms;
static App_State app_state;
static volatile uint8_t app_motion_enabled = 1U;

//static uint8_t app_benchmark_done = 0U;

static void app_set_state(App_State next_state);
static const char *app_state_to_string(App_State state);
static void app_process_button(uint32_t now_ms);
static Servo_Result_t app_unlock_all_joints(void);
static uint8_t app_motion_abort_requested(void);
static void app_log_control_telemetry(const Kinematics_ControlTelemetry_t *telemetry);
static void app_log_resolved_rate_telemetry(const Kinematics_ResolvedRateTelemetry_t *telemetry);



/* -------------------------------------------------------------------------- */
/* START: Benchmarking - Keep commented out                                   */
/* -------------------------------------------------------------------------- */
//void App_Init(UART_HandleTypeDef *servo_uart)
//{
//	UartServo_AttachHandle(servo_uart);
//	Servo_Init();
//	Tests_Benchmarks();
//}
//void App_Process(uint32_t now_ms)
//{
//    (void)now_ms;
//    if (app_benchmark_done == 0U)
//    {
//        return;
//    }
//}
/* -------------------------------------------------------------------------- */
/* END: Benchmarking - Keep commented out                                     */
/* -------------------------------------------------------------------------- */





void App_Init(UART_HandleTypeDef *servo_uart)
{
    Servo_Result_t result;
    uint16_t target_raw[KINEMATICS_ACTIVE_JOINT_COUNT];

    last_status_log_ms = 0U;
    last_button_event_ms = 0U;
    button_event_pending = 0U;
    app_state = APP_STATE_INIT;
    app_motion_enabled = 1U;

    UartServo_AttachHandle(servo_uart);
    Servo_Init();

    printf("\r\nelroboto booted\r\n");

    target_raw[0] = 2047U;   /* joint 1 */
    target_raw[1] = 1208U;   /* joint 2 */
    target_raw[2] = 2548U;   /* joint 3 */
    target_raw[3] = 2372U;   /* joint 4 */

    printf("Moving to app start position\r\n");

    for (uint8_t i = 0U; i < KINEMATICS_ACTIVE_JOINT_COUNT; i++)
    {
        uint8_t joint_id = (uint8_t)(i + 1U);

        result = Servo_WritePosition(
            joint_id,
            target_raw[i],
            APP_MOVEMENT_SPEED,
            APP_MOVEMENT_ACCELERATION
        );

        if (result != SERVO_RESULT_OK)
        {
            printf("Start move write failed: joint=%u result=%s\r\n",
                   (unsigned int)joint_id,
                   Servo_ResultToString(result));

            app_set_state(APP_STATE_FAULT);
            return;
        }

        HAL_Delay(20U);
    }

    HAL_Delay(300U);

    for (uint8_t i = 0U; i < KINEMATICS_ACTIVE_JOINT_COUNT; i++)
    {
        uint8_t joint_id = (uint8_t)(i + 1U);

        result = Kinematics_WaitUntilJointReached(
            joint_id,
            target_raw[i],
            50U,
            15000U,
			app_motion_abort_requested
        );

        if (result != SERVO_RESULT_OK)
        {
            printf("Start move wait failed: joint=%u result=%s\r\n",
                   (unsigned int)joint_id,
                   Servo_ResultToString(result));

            app_set_state(APP_STATE_FAULT);
            return;
        }

        HAL_Delay(20U);
    }

    printf("App start position reached\r\n");

    app_set_state(APP_STATE_IDLE);
}

void App_Process(uint32_t now_ms)
{
    static uint8_t tcp_start_saved = 0U;
    static uint8_t step_index = 0U;
    static Kinematics_Position_t tcp_start_position;

    Servo_Result_t result;
    Kinematics_Transform_t transform;
    Kinematics_Position_t target_position;
    Kinematics_IkConfig_t ik_config;
    float square_radius_m;

    app_process_button(now_ms);

    if (app_state != APP_STATE_IDLE || app_motion_enabled == 0U)
    {
        return;
    }

    square_radius_m = APP_SQUARE_RADIUS_CM / 100.0f;

    Kinematics_GetDefaultIkConfig(&ik_config);
    ik_config.position_tolerance_m = 0.001f;
    ik_config.max_iterations = 200U;
    ik_config.max_step_deg = 5.0f;
    ik_config.finite_difference_step_deg = 0.5f;
    ik_config.damping = 0.02f;

    if (tcp_start_saved == 0U)
    {
        float start_joint_deg[KINEMATICS_ACTIVE_JOINT_COUNT];

        result = Kinematics_RawToAngleDeg(1U, 2047U, &start_joint_deg[0]);
        if (result != SERVO_RESULT_OK)
        {
            printf("Start raw to angle failed: joint=1 result=%s\r\n",
                   Servo_ResultToString(result));

            app_set_state(APP_STATE_FAULT);
            return;
        }

        result = Kinematics_RawToAngleDeg(2U, 1208U, &start_joint_deg[1]);
        if (result != SERVO_RESULT_OK)
        {
            printf("Start raw to angle failed: joint=2 result=%s\r\n",
                   Servo_ResultToString(result));

            app_set_state(APP_STATE_FAULT);
            return;
        }

        result = Kinematics_RawToAngleDeg(3U, 2548U, &start_joint_deg[2]);
        if (result != SERVO_RESULT_OK)
        {
            printf("Start raw to angle failed: joint=3 result=%s\r\n",
                   Servo_ResultToString(result));

            app_set_state(APP_STATE_FAULT);
            return;
        }

        result = Kinematics_RawToAngleDeg(4U, 2372U, &start_joint_deg[3]);
        if (result != SERVO_RESULT_OK)
        {
            printf("Start raw to angle failed: joint=4 result=%s\r\n",
                   Servo_ResultToString(result));

            app_set_state(APP_STATE_FAULT);
            return;
        }

        result = Kinematics_ForwardDeg(start_joint_deg, &transform);
        if (result != SERVO_RESULT_OK)
        {
            printf("Start FK failed: result=%s\r\n",
                   Servo_ResultToString(result));

            app_set_state(APP_STATE_FAULT);
            return;
        }

        result = Kinematics_GetPosition(&transform, &tcp_start_position);
        if (result != SERVO_RESULT_OK)
        {
            printf("Start TCP extract failed: result=%s\r\n",
                   Servo_ResultToString(result));

            app_set_state(APP_STATE_FAULT);
            return;
        }

        printf("TCP start calculated: x=%.6f y=%.6f z=%.6f\r\n",
               tcp_start_position.x,
               tcp_start_position.y,
               tcp_start_position.z);

        tcp_start_saved = 1U;
    }

    target_position = tcp_start_position;

    if (step_index == 0U)
    {
        target_position.y -= square_radius_m;
        target_position.z += square_radius_m;

        printf("Square step 0: up-left target: x=%.6f y=%.6f z=%.6f\r\n",
               target_position.x,
               target_position.y,
               target_position.z);
    }
    else if (step_index == 1U)
    {
        target_position.y += square_radius_m;
        target_position.z += square_radius_m;

        printf("Square step 1: up-right target: x=%.6f y=%.6f z=%.6f\r\n",
               target_position.x,
               target_position.y,
               target_position.z);
    }
    else if (step_index == 2U)
    {
        target_position.y += square_radius_m;
        target_position.z -= square_radius_m;

        printf("Square step 2: down-right target: x=%.6f y=%.6f z=%.6f\r\n",
               target_position.x,
               target_position.y,
               target_position.z);
    }
    else
    {
        target_position.y -= square_radius_m;
        target_position.z -= square_radius_m;

        printf("Square step 3: down-left target: x=%.6f y=%.6f z=%.6f\r\n",
               target_position.x,
               target_position.y,
               target_position.z);
    }

    (void)app_log_control_telemetry;

#if 0
    result = Kinematics_MoveEndEffectorToPositionControlled(
        &target_position,
        APP_MOVEMENT_SPEED,
        APP_MOVEMENT_ACCELERATION,
        APP_CONTROL_TOLERANCE_TICKS,
        20000U,
        &ik_config,
        app_motion_abort_requested,
        app_log_control_telemetry
    );

#endif


    result = Kinematics_MoveEndEffectorToPositionResolvedRate(
        &target_position,
        APP_MOVEMENT_SPEED,
        APP_MOVEMENT_ACCELERATION,
        20000U,
        &ik_config,
        app_motion_abort_requested,
        app_log_resolved_rate_telemetry
    );


#if 0
    result = Kinematics_MoveEndEffectorToPositionOneShotAndCheck(
        &target_position,
        APP_MOVEMENT_SPEED,
        APP_MOVEMENT_ACCELERATION,
        20000U,
        &ik_config,
        app_motion_abort_requested,
        app_log_resolved_rate_telemetry
    );
#endif

#if 0
    result = Kinematics_MoveEndEffectorToPositionOneShotThenResolvedRate(
        &target_position,
        APP_MOVEMENT_SPEED,
        APP_MOVEMENT_ACCELERATION,
        20000U,
        &ik_config,
        app_motion_abort_requested,
        app_log_resolved_rate_telemetry
    );

#endif
    app_process_button(HAL_GetTick());

    if (result != SERVO_RESULT_OK)
    {
        printf("Square TCP move failed: step=%u result=%s\r\n",
               (unsigned int)step_index,
               Servo_ResultToString(result));

        app_set_state(APP_STATE_FAULT);
        return;
    }

    step_index++;
    if (step_index >= 4U)
    {
        step_index = 0U;
    }

    HAL_Delay(50U);

    app_process_button(HAL_GetTick());

    if (app_motion_enabled == 0U)
    {
        return;
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
    Servo_Result_t result;

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

    app_motion_enabled = 0U;

    printf("B1 pressed: motion disabled, unlocking all joints\r\n");

    result = app_unlock_all_joints();

    if (result != SERVO_RESULT_OK)
    {
        printf("B1 unlock failed: result=%s\r\n",
               Servo_ResultToString(result));

        app_set_state(APP_STATE_FAULT);
        return;
    }

    printf("B1 unlock done\r\n");
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

static uint8_t app_motion_abort_requested(void)
{
    app_process_button(HAL_GetTick());

    if (app_motion_enabled == 0U)
    {
        return 1U;
    }

    return 0U;
}

static void app_log_control_telemetry(const Kinematics_ControlTelemetry_t *telemetry)
{
    if (telemetry == NULL)
    {
        return;
    }

    printf(
        "CTRL cycle=%lu dt_ms=%.1f joint=%u current=%u target=%u "
        "error=%ld pid_step=%.2f applied=%ld command=%u reached=%u "
        "sent=%u joint_limit=%u\r\n",
        (unsigned long)telemetry->cycle_index,
        (double)(telemetry->dt_s * 1000.0f),
        (unsigned int)telemetry->joint_id,
        (unsigned int)telemetry->current_position_ticks,
        (unsigned int)telemetry->target_position_ticks,
        (long)telemetry->error_ticks,
        (double)telemetry->controller_output_ticks,
        (long)telemetry->applied_correction_ticks,
        (unsigned int)telemetry->commanded_position_ticks,
        (unsigned int)telemetry->within_tolerance,
        (unsigned int)telemetry->command_sent,
        (unsigned int)telemetry->joint_limit_clamped
    );
}

static void app_log_resolved_rate_telemetry(const Kinematics_ResolvedRateTelemetry_t *telemetry)
{
    if (telemetry == NULL)
    {
        return;
    }

    printf(
        "CART_CTRL cycle=%lu current=(%.4f,%.4f,%.4f) "
        "error_mm=(%.2f,%.2f,%.2f) norm_mm=%.2f "
        "measured=(%u,%u,%u,%u) command=(%u,%u,%u,%u)\r\n",
        (unsigned long)telemetry->cycle_index,
        (double)telemetry->current_position_m.x,
        (double)telemetry->current_position_m.y,
        (double)telemetry->current_position_m.z,
        (double)(telemetry->error_m.x * 1000.0f),
        (double)(telemetry->error_m.y * 1000.0f),
        (double)(telemetry->error_m.z * 1000.0f),
        (double)(telemetry->error_norm_m * 1000.0f),
        (unsigned int)telemetry->measured_position_ticks[0],
        (unsigned int)telemetry->measured_position_ticks[1],
        (unsigned int)telemetry->measured_position_ticks[2],
        (unsigned int)telemetry->measured_position_ticks[3],
        (unsigned int)telemetry->commanded_position_ticks[0],
        (unsigned int)telemetry->commanded_position_ticks[1],
        (unsigned int)telemetry->commanded_position_ticks[2],
        (unsigned int)telemetry->commanded_position_ticks[3]
    );
}
