#pragma once
#include <stdint.h>

void     crc32_reset(uint32_t *crc);
void     crc32_update(uint32_t *crc, const uint8_t *data, uint32_t len);
uint32_t crc32_value(uint32_t *crc);
uint32_t crc32_compute(const uint8_t *data, uint32_t len);

