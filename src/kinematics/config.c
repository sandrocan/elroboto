#include "config.h"


/* -------------------------------------------------------------------------- */
/* Private function prototypes                                                */
/* -------------------------------------------------------------------------- */

static void SystemPower_Config(void);
static void SystemClock_Config(void);
static void MX_ICACHE_Init(void);


/* -------------------------------------------------------------------------- */
/* Public functions                                                           */
/* -------------------------------------------------------------------------- */

void Config_Init(void)
{
    /*
     * Initialize HAL library.
     * This also configures SysTick and calls HAL_MspInit().
     */
    HAL_Init();

    /*
     * Configure power supply and system clock.
     */
    SystemPower_Config();
    SystemClock_Config();

    /*
     * Optional instruction cache setup.
     */
    MX_ICACHE_Init();
}

void Config_ErrorHandler(void)
{
    __disable_irq();

    while (1)
    {
        /*
         * Later we can blink an error LED here.
         */
    }
}


/* -------------------------------------------------------------------------- */
/* Power configuration                                                        */
/* -------------------------------------------------------------------------- */

static void SystemPower_Config(void)
{
    /*
     * Use SMPS supply.
     * This matches the Nucleo U545 setup from the working project.
     */
    if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
    {
        Config_ErrorHandler();
    }
}


/* -------------------------------------------------------------------------- */
/* Clock configuration                                                        */
/* -------------------------------------------------------------------------- */

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /*
     * Voltage scaling.
     * Scale 4 is enough for our simple 16 MHz setup.
     */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE4) != HAL_OK)
    {
        Config_ErrorHandler();
    }

    /*
     * Enable HSI16.
     * No PLL for now.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Config_ErrorHandler();
    }

    /*
     * System clock = HSI16 = 16 MHz.
     * AHB/APB prescalers = 1.
     */
    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK   |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1  |
        RCC_CLOCKTYPE_PCLK2  |
        RCC_CLOCKTYPE_PCLK3;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Config_ErrorHandler();
    }
}


/* -------------------------------------------------------------------------- */
/* ICACHE configuration                                                       */
/* -------------------------------------------------------------------------- */

static void MX_ICACHE_Init(void)
{
#ifdef HAL_ICACHE_MODULE_ENABLED
    if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
    {
        Config_ErrorHandler();
    }

    if (HAL_ICACHE_Enable() != HAL_OK)
    {
        Config_ErrorHandler();
    }
#endif
}


/* -------------------------------------------------------------------------- */
/* HAL MSP                                                                    */
/* -------------------------------------------------------------------------- */

void HAL_MspInit(void)
{
    /*
     * Enable PWR peripheral clock.
     * Needed for power configuration functions.
     */
    __HAL_RCC_PWR_CLK_ENABLE();
}
