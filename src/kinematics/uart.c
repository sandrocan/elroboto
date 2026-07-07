#include "uart.h"

#include "config.h"
#include "stm32u5xx_nucleo.h"

#include <stdio.h>
#include <string.h>


/* -------------------------------------------------------------------------- */
/* Private variables                                                          */
/* -------------------------------------------------------------------------- */

static UART_HandleTypeDef hlpuart1;
static COM_InitTypeDef debug_com_config;

/* -------------------------------------------------------------------------- */
/* Public functions                                                      */
/* -------------------------------------------------------------------------- */

void UartDebug_Init(void)
{
    /*
     * Debug terminal through ST-LINK Virtual COM Port.
     * This is NOT the servo UART.
     *
     * COM1 is handled by the Nucleo BSP.
     */
    debug_com_config.BaudRate = UART_DEBUG_BAUDRATE;
    debug_com_config.WordLength = COM_WORDLENGTH_8B;
    debug_com_config.StopBits = COM_STOPBITS_1;
    debug_com_config.Parity = COM_PARITY_NONE;
    debug_com_config.HwFlowCtl = COM_HWCONTROL_NONE;

    if (BSP_COM_Init(COM1, &debug_com_config) != BSP_ERROR_NONE)
    {
        Config_ErrorHandler();
    }
}

void UartServo_Init(void)
{
    /*
     * Servo bus UART:
     *
     * PA2 = LPUART1_TX = Arduino D1 / TXD1
     * PA3 = LPUART1_RX = Arduino D0 / RXD0
     *
     * Baudrate = 1,000,000
     */
    hlpuart1.Instance = LPUART1;
    hlpuart1.Init.BaudRate = UART_SERVO_BAUDRATE;
    hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
    hlpuart1.Init.StopBits = UART_STOPBITS_1;
    hlpuart1.Init.Parity = UART_PARITY_NONE;
    hlpuart1.Init.Mode = UART_MODE_TX_RX;
    hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;

    if (HAL_UART_Init(&hlpuart1) != HAL_OK)
    {
        Config_ErrorHandler();
    }

    if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        Config_ErrorHandler();
    }

    if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        Config_ErrorHandler();
    }

    if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
    {
        Config_ErrorHandler();
    }
}

void UartDebug_SendString(const char *text)
{
    if (text == NULL)
    {
        return;
    }

    /*
     * BSP_COM_Init() sets up the ST-LINK Virtual COM Port.
     * With USE_COM_LOG enabled in stm32u5xx_nucleo_conf.h,
     * printf is routed to COM1.
     */
    printf("%s", text);
}

void UartServo_SendString(const char *text)
{
    if (text == NULL)
    {
        return;
    }

    (void)HAL_UART_Transmit(
        &hlpuart1,
        (const uint8_t *)text,
        (uint16_t)strlen(text),
        HAL_MAX_DELAY
    );
}

HAL_StatusTypeDef UartServo_SendBytes(const uint8_t *data, uint16_t length, uint32_t timeout_ms)
{
    if ((data == NULL) || (length == 0U))
    {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(&hlpuart1, data, length, timeout_ms);
}

HAL_StatusTypeDef UartServo_ReadBytes(uint8_t *data, uint16_t length, uint32_t timeout_ms)
{
    if ((data == NULL) || (length == 0U))
    {
        return HAL_ERROR;
    }

    return HAL_UART_Receive(&hlpuart1, data, length, timeout_ms);
}

HAL_StatusTypeDef UartServo_SendCommand(const uint8_t *command, uint16_t length, uint32_t timeout_ms)
{
    return UartServo_SendBytes(command, length, timeout_ms);
}

HAL_StatusTypeDef UartServo_ReadResponse(uint8_t *response, uint16_t length, uint32_t timeout_ms)
{
    return UartServo_ReadBytes(response, length, timeout_ms);
}

UART_HandleTypeDef *UartServo_GetHandle(void)
{
    return &hlpuart1;
}


/* -------------------------------------------------------------------------- */
/* HAL UART MSP callbacks                                                     */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initializes GPIO and peripheral clocks for LPUART1.
 * @param huart Pointer to the UART handle being initialized.
 * @return None.
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    if (huart->Instance == LPUART1)
    {
        /*
         * Critical:
         * LPUART1 clock source = HSI16.
         */
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
        PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_HSI;

        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
        {
            Config_ErrorHandler();
        }

        __HAL_RCC_LPUART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /*
         * PA2 = LPUART1_TX
         * PA3 = LPUART1_RX
         * AF8 = LPUART1
         */
        GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_LPUART1;

        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

/**
 * @brief Deinitializes GPIO and peripheral clocks for LPUART1.
 * @param huart Pointer to the UART handle being deinitialized.
 * @return None.
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == LPUART1)
    {
        __HAL_RCC_LPUART1_CLK_DISABLE();

        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);
    }
}
