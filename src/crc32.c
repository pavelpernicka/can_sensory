#include "crc32.h"

#define CRC32_POLY 0x04C11DB7UL

void crc32_reset(uint32_t *crc)
{
    *crc = 0xFFFFFFFFUL;
}

void crc32_update(uint32_t *crc, const uint8_t *data, uint32_t len)
{
    uint32_t c = *crc;
    for (uint32_t i = 0; i < len; ++i) {
        c ^= ((uint32_t)data[i] << 24);
        for (int b = 0; b < 8; ++b) {
            if (c & 0x80000000UL) {
                c = (c << 1) ^ CRC32_POLY;
            } else {
                c <<= 1;
            }
        }
    }
    *crc = c;
}

uint32_t crc32_value(uint32_t *crc)
{
    return ~(*crc);
}

uint32_t crc32_compute(const uint8_t *data, uint32_t len)
{
    uint32_t crc;
    crc32_reset(&crc);
    crc32_update(&crc, data, len);
    return crc32_value(&crc);
}

