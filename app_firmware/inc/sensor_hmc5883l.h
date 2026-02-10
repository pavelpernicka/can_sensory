#pragma once

#include <stdint.h>

int SensorHMC5883L_Init(uint8_t range, uint8_t data_rate, uint8_t samples, uint8_t mode, uint16_t *mg_per_digit_centi);
int SensorHMC5883L_SetConfig(uint8_t range, uint8_t data_rate, uint8_t samples, uint8_t mode, uint16_t *mg_per_digit_centi);
int SensorHMC5883L_ReadNormalizedMg(int16_t off_x, int16_t off_y, int16_t off_z, uint16_t mg_per_digit_centi,
                                    int32_t *x_mg, int32_t *y_mg, int32_t *z_mg);
int SensorHMC5883L_ConfigValid(uint8_t range, uint8_t data_rate, uint8_t samples, uint8_t mode);
uint16_t SensorHMC5883L_RangeToMgPerDigitCenti(uint8_t range);
