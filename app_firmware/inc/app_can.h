#pragma once

#include "app_config.h"

void APP_CAN_Init(CAN_HandleTypeDef *hcan);
void APP_CAN_SetDeviceId(uint8_t device_id);
uint8_t APP_CAN_GetDeviceId(void);
void APP_CAN_SendFrame(const uint8_t *data, uint8_t dlc);
void APP_CAN_SendStatus(app_status_t status, uint8_t extra);
int APP_CAN_TryRecv(uint8_t *data, uint8_t *len);
