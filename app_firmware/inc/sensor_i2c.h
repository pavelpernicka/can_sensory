#pragma once

#include "app_config.h"

int SensorI2C_Init(I2C_HandleTypeDef *hi2c);
int SensorI2C_Write(uint8_t addr7, const uint8_t *data, uint16_t len);
int SensorI2C_Read(uint8_t addr7, uint8_t *data, uint16_t len);
int SensorI2C_MemRead(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t len);

