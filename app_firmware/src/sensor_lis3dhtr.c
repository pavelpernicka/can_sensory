#include "sensor_lis3dhtr.h"

#include "sensor_i2c.h"

#define LIS3DHTR_ADDR 0x19U

int SensorLIS3DHTR_Init(void)
{
    uint8_t who = 0U;
    uint8_t cfg[2];

    if (!SensorI2C_MemRead(LIS3DHTR_ADDR, 0x0FU, &who, 1U)) {
        return 0;
    }
    if (who != 0x33U) {
        return 0;
    }

    cfg[0] = 0x20U;
    cfg[1] = 0x57U; /* 100 Hz, XYZ enable */
    if (!SensorI2C_Write(LIS3DHTR_ADDR, cfg, 2U)) {
        return 0;
    }

    cfg[0] = 0x23U;
    cfg[1] = 0x00U; /* +/-2g, normal mode */
    if (!SensorI2C_Write(LIS3DHTR_ADDR, cfg, 2U)) {
        return 0;
    }

    return 1;
}

int SensorLIS3DHTR_ReadMg(int16_t *x_mg, int16_t *y_mg, int16_t *z_mg)
{
    uint8_t raw[6];
    int16_t x_raw;
    int16_t y_raw;
    int16_t z_raw;
    int16_t x_counts;
    int16_t y_counts;
    int16_t z_counts;

    if (x_mg == 0 || y_mg == 0 || z_mg == 0) {
        return 0;
    }

    if (!SensorI2C_MemRead(LIS3DHTR_ADDR, 0xA8U, raw, 6U)) {
        return 0;
    }

    x_raw = (int16_t)(((uint16_t)raw[1] << 8) | raw[0]);
    y_raw = (int16_t)(((uint16_t)raw[3] << 8) | raw[2]);
    z_raw = (int16_t)(((uint16_t)raw[5] << 8) | raw[4]);

    x_counts = (int16_t)(x_raw >> 6);
    y_counts = (int16_t)(y_raw >> 6);
    z_counts = (int16_t)(z_raw >> 6);

    *x_mg = (int16_t)(x_counts * 4); /* mg at +/-2g */
    *y_mg = (int16_t)(y_counts * 4);
    *z_mg = (int16_t)(z_counts * 4);
    return 1;
}
