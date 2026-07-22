#include "app.h"
#include "kinematics.h"
#include "servo.h"
#include "uart.h"

#include <stdio.h>
#include <stdint.h>

#define BUTTON_DEBOUNCE_MS      50U
#define HOME_CHECK_TOLERANCE_TICKS 50U

/*
 * Bring-up mode for the e-skin UART connection. While enabled,
 * neither servo initialization nor any movement/unlock command is executed.
 */
#define APP_SKIN_TEST_ONLY              0U
#define APP_SKIN_LOG_PERIOD_MS        250U
#define APP_SKIN_STOP_THRESHOLD       0.050f
#define APP_SKIN_CLEAR_THRESHOLD      0.020f
#define APP_SKIN_CLEAR_STABLE_MS       500U
#define APP_SKIN_STARTUP_TIMEOUT_MS   2000U
#define APP_SKIN_DATA_TIMEOUT_MS      1000U

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
  APP_STATE_PAUSED_SKIN,
  APP_STATE_FAULT
} App_State;

static uint32_t last_status_log_ms;
static volatile uint8_t button_event_pending;
static volatile uint32_t last_button_event_ms;
static App_State app_state;
static volatile uint8_t app_motion_enabled = 1U;
static volatile float skin_distance = 0.0f;
static uint8_t app_skin_pause_active;
static uint32_t app_skin_clear_since_ms;

//static uint8_t app_benchmark_done = 0U;

static void app_set_state(App_State next_state);
static const char *app_state_to_string(App_State state);
static void app_process_button(uint32_t now_ms);
static Servo_Result_t app_unlock_all_joints(void);
static uint8_t app_motion_abort_requested(void);
static void app_log_joint_tick_pid_telemetry(const Kinematics_JointTickPidTelemetry_t *telemetry);
static void app_log_resolved_rate_telemetry(const Kinematics_ResolvedRateTelemetry_t *telemetry);
static uint8_t app_wait_for_skin_sample(uint32_t timeout_ms);
static void app_pause_for_skin(uint32_t now_ms);
static void app_update_skin_pause(uint32_t now_ms);
static void app_latch_skin_fault(const char *reason, uint32_t now_ms);
static Servo_Result_t app_hold_active_joints(void);



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


/**
 * @brief Initializes the application state, UART paths, servo module, and startup safety checks.
 * @param servo_uart Initialized UART handle for the servo bus.
 * @param cell_uart Initialized UART handle for e-skin data reception.
 * @return None. Updates global application state and may latch APP_STATE_FAULT on failure.
 */
void App_Init(UART_HandleTypeDef *servo_uart, UART_HandleTypeDef *cell_uart)
{
    HAL_StatusTypeDef cell_rx_status;
    Servo_Result_t result;
    uint16_t target_raw[KINEMATICS_ACTIVE_JOINT_COUNT];

    last_status_log_ms = 0U;
    last_button_event_ms = 0U;
    button_event_pending = 0U;
    app_state = APP_STATE_INIT;
    app_motion_enabled = 1U;
    app_skin_pause_active = 0U;
    app_skin_clear_since_ms = 0U;

    UartServo_AttachHandle(servo_uart);
    UartCell_AttachHandle(cell_uart);

    cell_rx_status = UartCell_StartReceiveIT(&skin_distance);
    if (cell_rx_status != HAL_OK)
    {
        app_motion_enabled = 0U;
        printf("UART4 e-skin receive start failed: status=%d\r\n",
               (int)cell_rx_status);
        app_set_state(APP_STATE_FAULT);
        return;
    }

    printf("\r\nelroboto booted\r\n");

    Servo_Init();

    if (app_wait_for_skin_sample(APP_SKIN_STARTUP_TIMEOUT_MS) == 0U)
    {
        app_motion_enabled = 0U;
        printf("E-skin startup failed: no valid packet within %lu ms\r\n",
               (unsigned long)APP_SKIN_STARTUP_TIMEOUT_MS);
        app_set_state(APP_STATE_FAULT);
        return;
    }

    if (skin_distance >= APP_SKIN_STOP_THRESHOLD)
    {
        app_motion_enabled = 0U;
        printf("E-skin blocks startup: value=%.3f threshold=%.3f\r\n",
               (float)skin_distance,
               (double)APP_SKIN_STOP_THRESHOLD);
        app_set_state(APP_STATE_FAULT);
        return;
    }

    printf("E-skin ready: value=%.3f stop_threshold=%.3f\r\n",
           (float)skin_distance,
           (double)APP_SKIN_STOP_THRESHOLD);

    target_raw[0] = 2047U;   /* joint 1 */
    target_raw[1] = 1208U;   /* joint 2 */
    target_raw[2] = 2548U;   /* joint 3 */
    target_raw[3] = 2372U;   /* joint 4 */

    printf("Moving to app start position\r\n");

    for (uint8_t i = 0U; i < KINEMATICS_ACTIVE_JOINT_COUNT; i++)
    {
        uint8_t joint_id = (uint8_t)(i + 1U);

        if (app_motion_abort_requested() != 0U)
        {
            printf("Start move aborted before joint=%u\r\n",
                   (unsigned int)joint_id);
            if (app_state != APP_STATE_FAULT)
            {
                app_set_state(APP_STATE_FAULT);
            }
            return;
        }

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

            if (app_state != APP_STATE_FAULT)
            {
                app_set_state(APP_STATE_FAULT);
            }
            return;
        }

        HAL_Delay(20U);
    }

    printf("App start position reached\r\n");

    app_set_state(APP_STATE_IDLE);
}

/**
 * @brief Executes one non-blocking application state-machine cycle.
 * @param now_ms Current monotonic HAL time in milliseconds.
 * @return None. Processes inputs and may issue motion commands or change the application state.
 */
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

    if (app_motion_abort_requested() != 0U)
    {
        return;
    }

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

    (void)app_log_joint_tick_pid_telemetry;

#if 0
    result = Kinematics_MoveEndEffectorToPositionJointTickPid(
        &target_position,
        APP_MOVEMENT_SPEED,
        APP_MOVEMENT_ACCELERATION,
        APP_CONTROL_TOLERANCE_TICKS,
        20000U,
        &ik_config,
        app_motion_abort_requested,
        app_log_joint_tick_pid_telemetry
    );

#endif

#if 0
    /* Baseline experiment: one IK command without outer TCP feedback. */
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

    /* Final application path: Cartesian resolved-rate feedback control. */
    result = Kinematics_MoveEndEffectorToPositionResolvedRate(
        &target_position,
        APP_MOVEMENT_SPEED,
        APP_MOVEMENT_ACCELERATION,
        20000U,
        &ik_config,
        app_motion_abort_requested,
        app_log_resolved_rate_telemetry
    );


    app_process_button(HAL_GetTick());

    if (result != SERVO_RESULT_OK)
    {
        if (result == SERVO_RESULT_ABORTED)
        {
            if (app_skin_pause_active != 0U)
            {
                printf("Square TCP move paused by e-skin: step=%u\r\n",
                       (unsigned int)step_index);
            }
            else
            {
                printf("Square TCP move aborted: step=%u state=%s\r\n",
                       (unsigned int)step_index,
                       app_state_to_string(app_state));
            }

            return;
        }

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

    if (app_motion_abort_requested() != 0U)
    {
        return;
    }

}

/**
 * @brief Records a pending user-button event for deferred processing.
 * @return None. Sets the interrupt-safe button_event_pending flag.
 */
void App_OnButtonInterrupt(void)
{
  button_event_pending = 1U;
}

/**
 * @brief Changes the current application state and logs its readable name.
 * @param next_state State to enter.
 * @return None. Updates app_state and writes a diagnostic message.
 */
static void app_set_state(App_State next_state)
{
  app_state = next_state;
  printf("App state: %s\r\n", app_state_to_string(next_state));
}

/**
 * @brief Converts an application state value to readable diagnostic text.
 * @param state Application state to convert.
 * @return Constant state name, or "UNKNOWN" for an unsupported value.
 */
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

    case APP_STATE_PAUSED_SKIN:
      return "PAUSED_SKIN";

    case APP_STATE_FAULT:
      return "FAULT";

    default:
      return "UNKNOWN";
  }
}

/**
 * @brief Debounces and handles a pending user-button stop request.
 * @param now_ms Current monotonic HAL time in milliseconds.
 * @return None. Disables motion, unlocks joints, and latches a fault state when handled.
 */
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
    app_skin_pause_active = 0U;
    app_skin_clear_since_ms = 0U;

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
    app_set_state(APP_STATE_FAULT);
}

/**
 * @brief Disables torque for every configured joint while preserving the first error.
 * @return SERVO_RESULT_OK when all joints unlock, otherwise the first servo error.
 */
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

/**
 * @brief Processes safety inputs and determines whether the current motion must abort.
 * @return 1 when motion is disabled or a safety condition is active, otherwise 0.
 */
static uint8_t app_motion_abort_requested(void)
{
    uint32_t now_ms;
    UartCell_Diagnostics_t diagnostics;

    UartCell_Process();
    now_ms = HAL_GetTick();
    app_process_button(now_ms);

    if (app_skin_pause_active != 0U)
    {
        app_update_skin_pause(now_ms);
    }

    if (app_motion_enabled != 0U)
    {
        UartCell_GetDiagnostics(&diagnostics);

        if ((diagnostics.valid_frame_count == 0U) ||
            ((uint32_t)(now_ms - diagnostics.last_valid_frame_ms) >
             APP_SKIN_DATA_TIMEOUT_MS))
        {
            app_latch_skin_fault("sensor timeout", now_ms);
        }
        else if (skin_distance >= APP_SKIN_STOP_THRESHOLD)
        {
            if (app_state == APP_STATE_IDLE)
            {
                app_pause_for_skin(now_ms);
            }
            else
            {
                app_latch_skin_fault("proximity during startup", now_ms);
            }
        }
    }

    if (app_motion_enabled == 0U)
    {
        return 1U;
    }

    return 0U;
}

/**
 * @brief Waits for the first valid e-skin sample within a bounded startup interval.
 * @param timeout_ms Maximum wait duration in milliseconds.
 * @return 1 when a valid sample arrives, otherwise 0 after the timeout.
 */
static uint8_t app_wait_for_skin_sample(uint32_t timeout_ms)
{
    uint32_t start_ms = HAL_GetTick();
    UartCell_Diagnostics_t diagnostics;

    do
    {
        UartCell_Process();
        UartCell_GetDiagnostics(&diagnostics);
        if (diagnostics.valid_frame_count > 0U)
        {
            return 1U;
        }

        /* Bounded startup wait; no servo command has been issued yet. */
        HAL_Delay(10U);
    }
    while ((uint32_t)(HAL_GetTick() - start_ms) < timeout_ms);

    return 0U;
}

/**
 * @brief Pauses motion and commands the active joints to hold after an e-skin trigger.
 * @param now_ms Current monotonic HAL time in milliseconds for diagnostics.
 * @return None. Updates pause, motion-enable, and application-state variables.
 */
static void app_pause_for_skin(uint32_t now_ms)
{
    Servo_Result_t hold_result;

    if (app_skin_pause_active != 0U)
    {
        return;
    }

    app_skin_pause_active = 1U;
    app_skin_clear_since_ms = 0U;
    app_motion_enabled = 0U;

    printf("E-skin pause: value=%.3f stop_threshold=%.3f time=%lu ms\r\n",
           (float)skin_distance,
           (double)APP_SKIN_STOP_THRESHOLD,
           (unsigned long)now_ms);

    /*
     * Commanding each active joint to its measured position decelerates and
     * holds it without deliberately removing torque from the loaded arm.
     * This remains a software stop, not a certified emergency stop.
     */
    hold_result = app_hold_active_joints();
    printf("E-skin hold result: %s\r\n", Servo_ResultToString(hold_result));

    if (hold_result == SERVO_RESULT_OK)
    {
        app_set_state(APP_STATE_PAUSED_SKIN);
    }
    else
    {
        app_skin_pause_active = 0U;
        app_set_state(APP_STATE_FAULT);
    }
}

/**
 * @brief Monitors a skin-triggered pause and resumes only after a stable clear interval.
 * @param now_ms Current monotonic HAL time in milliseconds.
 * @return None. May clear the pause, resume motion, or latch a sensor fault.
 */
static void app_update_skin_pause(uint32_t now_ms)
{
    UartCell_Diagnostics_t diagnostics;
    uint32_t clear_elapsed_ms = 0U;

    UartCell_GetDiagnostics(&diagnostics);

    if ((diagnostics.valid_frame_count == 0U) ||
        ((uint32_t)(now_ms - diagnostics.last_valid_frame_ms) >
         APP_SKIN_DATA_TIMEOUT_MS))
    {
        app_latch_skin_fault("sensor timeout while paused", now_ms);
        return;
    }

    if (app_skin_clear_since_ms != 0U)
    {
        clear_elapsed_ms = (uint32_t)(now_ms - app_skin_clear_since_ms);
    }

    if ((uint32_t)(now_ms - last_status_log_ms) >= APP_SKIN_LOG_PERIOD_MS)
    {
        last_status_log_ms = now_ms;
        printf("E-skin paused: value=%.3f clear_threshold=%.3f clear_ms=%lu\r\n",
               (float)skin_distance,
               (double)APP_SKIN_CLEAR_THRESHOLD,
               (unsigned long)clear_elapsed_ms);
    }

    if (skin_distance <= APP_SKIN_CLEAR_THRESHOLD)
    {
        if (app_skin_clear_since_ms == 0U)
        {
            app_skin_clear_since_ms = now_ms;
            printf("E-skin clear candidate: value=%.3f\r\n",
                   (float)skin_distance);
        }
        else if ((uint32_t)(now_ms - app_skin_clear_since_ms) >=
                 APP_SKIN_CLEAR_STABLE_MS)
        {
            app_skin_pause_active = 0U;
            app_skin_clear_since_ms = 0U;
            app_motion_enabled = 1U;

            printf("E-skin clear: value=%.3f stable_ms=%lu; resuming\r\n",
                   (float)skin_distance,
                   (unsigned long)APP_SKIN_CLEAR_STABLE_MS);
            app_set_state(APP_STATE_IDLE);
        }
    }
    else
    {
        app_skin_clear_since_ms = 0U;
    }
}

/**
 * @brief Latches an e-skin safety fault and attempts to hold all active joints.
 * @param reason Readable reason included in the diagnostic output.
 * @param now_ms Current monotonic HAL time in milliseconds.
 * @return None. Disables motion and transitions the application to APP_STATE_FAULT.
 */
static void app_latch_skin_fault(const char *reason, uint32_t now_ms)
{
    Servo_Result_t hold_result = SERVO_RESULT_OK;
    UartCell_Diagnostics_t diagnostics;
    uint32_t last_valid_age_ms;

    if (app_state == APP_STATE_FAULT)
    {
        return;
    }

    app_motion_enabled = 0U;
    UartCell_GetDiagnostics(&diagnostics);
    last_valid_age_ms = (diagnostics.valid_frame_count == 0U)
                      ? 0U
                      : (uint32_t)(now_ms - diagnostics.last_valid_frame_ms);

    printf("E-skin fault latched: reason=%s value=%.3f time=%lu ms\r\n",
           reason,
           (float)skin_distance,
           (unsigned long)now_ms);
    printf("E-skin UART: rx=%lu valid=%lu invalid=%lu uart_err=%lu "
           "last_err=0x%08lx rearm_fail=%lu last=0x%02x "
           "last_valid_age_ms=%lu\r\n",
           (unsigned long)diagnostics.received_byte_count,
           (unsigned long)diagnostics.valid_frame_count,
           (unsigned long)diagnostics.invalid_frame_count,
           (unsigned long)diagnostics.uart_error_count,
           (unsigned long)diagnostics.last_uart_error,
           (unsigned long)diagnostics.receive_restart_failure_count,
           (unsigned int)diagnostics.last_received_byte,
           (unsigned long)last_valid_age_ms);

    if (app_skin_pause_active == 0U)
    {
        hold_result = app_hold_active_joints();
        printf("E-skin fault hold result: %s\r\n",
               Servo_ResultToString(hold_result));
    }

    app_skin_pause_active = 0U;
    app_skin_clear_since_ms = 0U;
    app_set_state(APP_STATE_FAULT);
}

/**
 * @brief Reads each active joint and commands it to hold its measured position.
 * @return SERVO_RESULT_OK on success, otherwise the first read or write error.
 */
static Servo_Result_t app_hold_active_joints(void)
{
    Servo_Result_t first_error = SERVO_RESULT_OK;

    for (uint8_t i = 0U; i < KINEMATICS_ACTIVE_JOINT_COUNT; i++)
    {
        const uint8_t joint_id = (uint8_t)(i + 1U);
        uint16_t current_position_ticks = 0U;
        Servo_Result_t result = Servo_ReadPosition(joint_id,
                                                   &current_position_ticks);

        if (result == SERVO_RESULT_OK)
        {
            result = Servo_WritePosition(joint_id,
                                         current_position_ticks,
                                         APP_MOVEMENT_SPEED,
                                         APP_MOVEMENT_ACCELERATION);
        }

        printf("E-skin hold joint=%u position=%u result=%s\r\n",
               (unsigned int)joint_id,
               (unsigned int)current_position_ticks,
               Servo_ResultToString(result));

        if ((result != SERVO_RESULT_OK) &&
            (first_error == SERVO_RESULT_OK))
        {
            first_error = result;
        }
    }

    return first_error;
}

/**
 * @brief Prints one joint-tick PID telemetry sample.
 * @param telemetry Controller telemetry to print; NULL is ignored.
 * @return None. Writes a formatted diagnostic line to standard output.
 */
static void app_log_joint_tick_pid_telemetry(const Kinematics_JointTickPidTelemetry_t *telemetry)
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

/**
 * @brief Prints one Cartesian resolved-rate telemetry sample.
 * @param telemetry Cartesian controller telemetry to print; NULL is ignored.
 * @return None. Writes a formatted diagnostic line to standard output.
 */
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
