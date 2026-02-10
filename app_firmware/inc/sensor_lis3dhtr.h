#pragma once

#include <stdint.h>

int SensorLIS3DHTR_Init(void);
int SensorLIS3DHTR_ReadMg(int16_t *x_mg, int16_t *y_mg, int16_t *z_mg);
