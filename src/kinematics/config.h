#ifndef CONFIG_H_
#define CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32u5xx_hal.h"

void Config_Init(void);
void Config_ErrorHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H_ */
