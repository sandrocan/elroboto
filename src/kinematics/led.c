#include "led.h"


/* -------------------------------------------------------------------------- */
/* Private defines                                                            */
/* -------------------------------------------------------------------------- */

#define LED_GPIO_PORT    GPIOA
#define LED_GPIO_PIN     GPIO_PIN_5


/* -------------------------------------------------------------------------- */
/* Public functions                                                           */
/* -------------------------------------------------------------------------- */

void Led_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*
     * Enable GPIOA clock.
     * LD2 is connected to PA5.
     */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*
     * Start with LED off.
     */
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_RESET);

    /*
     * Configure PA5 as push-pull output.
     */
    GPIO_InitStruct.Pin = LED_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);
}

void Led_On(void)
{
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_SET);
}

void Led_Off(void)
{
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_RESET);
}

void Led_Toggle(void)
{
    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_GPIO_PIN);
}
