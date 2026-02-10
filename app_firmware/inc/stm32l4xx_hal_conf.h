#pragma once

#define USE_HAL_DRIVER

#define HAL_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_CAN_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED

#define HSE_VALUE           ((uint32_t)8000000U)
#define HSI_VALUE           ((uint32_t)16000000U)
#define MSI_VALUE           ((uint32_t)4000000U)
#define LSE_VALUE           ((uint32_t)32768U)
#define LSI_VALUE           ((uint32_t)32000U)
#define HSI48_VALUE         ((uint32_t)48000000U)

#define EXTERNAL_SAI1_CLOCK_VALUE ((uint32_t)48000U)
#define EXTERNAL_SAI2_CLOCK_VALUE ((uint32_t)48000U)

#define HSE_STARTUP_TIMEOUT ((uint32_t)100U)
#define LSE_STARTUP_TIMEOUT ((uint32_t)5000U)

#define VDD_VALUE           ((uint32_t)3300U)

#define TICK_INT_PRIORITY         0x0FU
#define USE_RTOS                  0
#define PREFETCH_ENABLE           1
#define INSTRUCTION_CACHE_ENABLE  1
#define DATA_CACHE_ENABLE         1

#include "stm32l4xx_hal_def.h"

#ifdef HAL_RCC_MODULE_ENABLED
#include "stm32l4xx_hal_rcc.h"
#include "stm32l4xx_hal_rcc_ex.h"
#endif

#ifdef HAL_GPIO_MODULE_ENABLED
#include "stm32l4xx_hal_gpio.h"
#endif

#ifdef HAL_CORTEX_MODULE_ENABLED
#include "stm32l4xx_hal_cortex.h"
#endif

#ifdef HAL_FLASH_MODULE_ENABLED
#include "stm32l4xx_hal_flash.h"
#include "stm32l4xx_hal_flash_ex.h"
#endif

#ifdef HAL_CAN_MODULE_ENABLED
#include "stm32l4xx_hal_can.h"
#endif

#ifdef HAL_DMA_MODULE_ENABLED
#include "stm32l4xx_hal_dma.h"
#endif

#ifdef HAL_I2C_MODULE_ENABLED
#include "stm32l4xx_hal_i2c.h"
#include "stm32l4xx_hal_i2c_ex.h"
#endif

#ifdef HAL_PWR_MODULE_ENABLED
#include "stm32l4xx_hal_pwr.h"
#include "stm32l4xx_hal_pwr_ex.h"
#endif

#ifndef assert_param
#define assert_param(expr) ((void)0U)
#endif

