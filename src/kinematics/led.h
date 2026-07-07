#ifndef LED_H_
#define LED_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32u5xx_hal.h"

void Led_Init(void);
void Led_On(void);
void Led_Off(void);
void Led_Toggle(void);

#ifdef __cplusplus
}
#endif

#endif /* LED_H_ */
