#pragma once
#include "stm32l4xx_hal.h"
#include "bl_config.h"
#include "crc32.h"

void Flash_Init(void);
HAL_StatusTypeDef Flash_EraseAppArea(void);
HAL_StatusTypeDef Flash_ProgramBytes(uint32_t addr, const uint8_t *data, uint32_t len);
void Flash_ReadMeta(bl_meta_t *meta);
HAL_StatusTypeDef Flash_WriteMeta(const bl_meta_t *meta);
uint32_t Flash_ComputeAppCrc(uint32_t size);
int Flash_IsAppValid(bl_meta_t *meta_out);
