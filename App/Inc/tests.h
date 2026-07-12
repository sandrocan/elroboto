#ifndef TESTS_H_
#define TESTS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32u5xx_hal.h"
#include "servo.h"

/**
 * @brief Runs the complete home test sequence.
 * @return Servo operation result.
 */
Servo_Result_t Tests_HomeTest(void);

/**
 * @brief Tests direct kinematics by moving each active joint forward and back.
 * @return Servo operation result.
 */
Servo_Result_t Tests_DkTest(void);

/**
 * @brief Tests inverse kinematics by moving the end effector in X, Y and Z.
 * @return Servo operation result.
 */
Servo_Result_t Tests_IkTest(void);

#ifdef __cplusplus
}
#endif

#endif /* TESTS_H_ */
