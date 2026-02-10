#include "sensor_i2c.h"

static I2C_HandleTypeDef *g_hi2c;

int SensorI2C_Init(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == 0) {
        return 0;
    }

    g_hi2c = hi2c;
    g_hi2c->Instance = APP_I2C_INSTANCE;
    g_hi2c->Init.Timing = APP_I2C_TIMING;
    g_hi2c->Init.OwnAddress1 = 0U;
    g_hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    g_hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    g_hi2c->Init.OwnAddress2 = 0U;
    g_hi2c->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    g_hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    g_hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(g_hi2c) != HAL_OK) {
        return 0;
    }
    if (HAL_I2CEx_ConfigAnalogFilter(g_hi2c, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
        return 0;
    }

    return 1;
}

int SensorI2C_Write(uint8_t addr7, const uint8_t *data, uint16_t len)
{
    if (g_hi2c == 0 || data == 0 || len == 0U) {
        return 0;
    }

    return HAL_I2C_Master_Transmit(g_hi2c, (uint16_t)(addr7 << 1), (uint8_t *)data, len, APP_I2C_TIMEOUT_MS) == HAL_OK;
}

int SensorI2C_Read(uint8_t addr7, uint8_t *data, uint16_t len)
{
    if (g_hi2c == 0 || data == 0 || len == 0U) {
        return 0;
    }

    return HAL_I2C_Master_Receive(g_hi2c, (uint16_t)(addr7 << 1), data, len, APP_I2C_TIMEOUT_MS) == HAL_OK;
}

int SensorI2C_MemRead(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (g_hi2c == 0 || data == 0 || len == 0U) {
        return 0;
    }

    return HAL_I2C_Mem_Read(g_hi2c, (uint16_t)(addr7 << 1), reg, I2C_MEMADD_SIZE_8BIT, data, len, APP_I2C_TIMEOUT_MS) == HAL_OK;
}
