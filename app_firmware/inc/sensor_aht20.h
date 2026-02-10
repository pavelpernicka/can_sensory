#pragma once

#include "sensors.h"

int SensorAHT20_Init(uint8_t *status);
int SensorAHT20_Trigger(void);
int SensorAHT20_ReadAfterTrigger(app_aht20_diag_t *out);
int SensorAHT20_ReadBlocking(app_aht20_diag_t *out);
int SensorAHT20_GetStatus(uint8_t *status);
int SensorAHT20_Reset(void);
int SensorAHT20_SetReg(const uint8_t *buf, uint8_t len);
int SensorAHT20_GetReg(uint8_t *buf, uint8_t len);
