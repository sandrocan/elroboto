#ifndef APP_H
#define APP_H

#include <stdint.h>

/** Initialize the application state after all required hardware is ready. */
void App_Init(void);

/** Run one non-blocking application cycle. */
void App_Process(uint32_t now_ms);

/** Notify the application about a user-button interrupt. */
void App_OnButtonInterrupt(void);

#endif /* APP_H */
