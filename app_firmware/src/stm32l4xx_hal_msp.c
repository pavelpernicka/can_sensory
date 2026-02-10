#include "stm32l4xx_hal.h"
#include "app_config.h"

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1) {
        __HAL_RCC_CAN1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitTypeDef gpio = {0};
        gpio.Pin = GPIO_PIN_11 | GPIO_PIN_12;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF9_CAN1;
        HAL_GPIO_Init(GPIOA, &gpio);
    }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1) {
        __HAL_RCC_CAN1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
    }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == APP_I2C_INSTANCE) {
        RCC_PeriphCLKInitTypeDef periphClkInit = {0};
        periphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
        periphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
        HAL_RCCEx_PeriphCLKConfig(&periphClkInit);

        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_I2C1_CLK_ENABLE();

        GPIO_InitTypeDef gpio = {0};
        gpio.Mode = GPIO_MODE_AF_OD;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = APP_I2C_GPIO_AF;

        gpio.Pin = APP_I2C_SCL_PIN;
        HAL_GPIO_Init(APP_I2C_SCL_GPIO_PORT, &gpio);

        gpio.Pin = APP_I2C_SDA_PIN;
        HAL_GPIO_Init(APP_I2C_SDA_GPIO_PORT, &gpio);
    }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == APP_I2C_INSTANCE) {
        __HAL_RCC_I2C1_CLK_DISABLE();
        HAL_GPIO_DeInit(APP_I2C_SCL_GPIO_PORT, APP_I2C_SCL_PIN);
        HAL_GPIO_DeInit(APP_I2C_SDA_GPIO_PORT, APP_I2C_SDA_PIN);
    }
}

