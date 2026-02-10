#pragma once
#include "stm32l4xx_hal.h"
#include "bl_config.h"
#include "flash_if.h"
#include "crc32.h"

typedef enum {
    BL_CAN_EVENT_NONE     = 0,
    BL_CAN_EVENT_ACTIVITY = 1 << 0,
    BL_CAN_EVENT_PING     = 1 << 1
} bl_can_event_t;

void BL_CAN_Init(CAN_HandleTypeDef *hcan);
uint32_t BL_CAN_Poll(void);
int  BL_CAN_StayInBootloaderRequested(void);
void BL_CAN_ForceStayInBootloader(void);
void BL_CAN_SetLastBootError(uint8_t code);
uint8_t BL_CAN_GetLastBootError(void);
void BL_CAN_ReportBootError(uint8_t code);
extern uint8_t BL_BootToAppRequest;
