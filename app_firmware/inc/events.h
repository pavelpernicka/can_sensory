#pragma once

#include "app_config.h"
#include "calibration.h"

typedef struct {
    uint8_t type;
    uint8_t p0;
    uint8_t p1;
    uint8_t p2;
    uint16_t p3;
} app_event_t;

void Events_Init(void);
void Events_ProcessMagSample(float x, float y, float z, uint32_t now_ms);
void Events_PostNoData(uint32_t now_ms);
int Events_Pop(app_event_t *out);
void Events_GetSectorState(uint8_t *sector, uint8_t *elevation);
void Events_ApplyCalibration(const app_calibration_t *cal);
