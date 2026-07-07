/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Niklas Peter
 * @brief          : Main program body
 ******************************************************************************
 */

#include <stdio.h>
#include "config.h"
#include "led.h"
#include "uart.h"
#include "servo.h"

/* -------------------------------------------------------------------------- */
/* Calibration target                                                         */
/* -------------------------------------------------------------------------- */

#define TARGET_JOINT_ID          5U
#define TARGET_POSITION_TICKS    2047U

#define MOVE_SPEED               50U
#define MOVE_ACCELERATION        5U

/*
 * 0 = joint stays locked after movement
 * 1 = joint unlocks after movement
 */
#define UNLOCK_AFTER_MOVE        0U


/* -------------------------------------------------------------------------- */
/* Helper functions                                                           */
/* -------------------------------------------------------------------------- */


static void PrintResult(const char *label, Servo_Result_t result)
{
    UartDebug_SendString(label);
    UartDebug_SendString(": ");
    UartDebug_SendString(Servo_ResultToString(result));
    UartDebug_SendString("\r\n");
}

/**
 * @brief Unlocks joints 1 to 6 and prints the result of each unlock command.
 * @return None.
 */
static void UnlockAllAndPrint(void)
{
    Servo_Result_t result;

    result = Servo_UnlockJoint(1U);
    PrintResult("Unlock joint 1", result);
    HAL_Delay(100);

    result = Servo_UnlockJoint(2U);
    PrintResult("Unlock joint 2", result);
    HAL_Delay(100);

    result = Servo_UnlockJoint(3U);
    PrintResult("Unlock joint 3", result);
    HAL_Delay(100);

    result = Servo_UnlockJoint(4U);
    PrintResult("Unlock joint 4", result);
    HAL_Delay(100);

    result = Servo_UnlockJoint(5U);
    PrintResult("Unlock joint 5", result);
    HAL_Delay(100);

    result = Servo_UnlockJoint(6U);
    PrintResult("Unlock joint 6", result);
    HAL_Delay(100);
}


/* -------------------------------------------------------------------------- */
/* Main                                                                       */
/* -------------------------------------------------------------------------- */

int main(void)
{
    Servo_Result_t result;

    Config_Init();

    Led_Init();

    UartDebug_Init();
    UartServo_Init();

    Servo_Init();

    Servo_Result_t home_result;

    home_result = Servo_DriveHome();

    UartDebug_SendString("Drive home: ");
    UartDebug_SendString(Servo_ResultToString(home_result));
    UartDebug_SendString("\r\n");

    if (home_result != SERVO_RESULT_OK)
    {
        UartDebug_SendString("DriveHome failed. Stopping program.\r\n");

        UnlockAllAndPrint();

        while (1)
        {
            Led_Toggle();
            HAL_Delay(200);
        }
    }

    UartDebug_SendString("DriveHome succeeded. Stopping program.\r\n");

	UnlockAllAndPrint();

	while (1)
	{
		Led_Toggle();
		HAL_Delay(200);
	}
}
