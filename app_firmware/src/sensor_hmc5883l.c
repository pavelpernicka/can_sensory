#include "sensor_hmc5883l.h"

#include "sensor_i2c.h"

#define HMC5883L_ADDR 0x1EU

uint16_t SensorHMC5883L_RangeToMgPerDigitCenti(uint8_t range)
{
    static const uint16_t mg_per_digit_centi[8] = {
        73U, 92U, 122U, 152U, 227U, 256U, 303U, 435U
    };

    if (range > 7U) {
        return 0U;
    }
    return mg_per_digit_centi[range];
}

int SensorHMC5883L_ConfigValid(uint8_t range, uint8_t data_rate, uint8_t samples, uint8_t mode)
{
    if (range > 7U) {
        return 0;
    }
    if (data_rate > 6U) {
        return 0;
    }
    if (samples > 3U) {
        return 0;
    }
    if (mode > 2U) {
        return 0;
    }
    return 1;
}

static int sensor_hmc5883l_apply_config(uint8_t range, uint8_t data_rate, uint8_t samples, uint8_t mode,
                                        uint16_t *mg_per_digit_centi)
{
    uint8_t cfg[2];
    uint16_t mg_centi;

    if (!SensorHMC5883L_ConfigValid(range, data_rate, samples, mode)) {
        return 2;
    }

    mg_centi = SensorHMC5883L_RangeToMgPerDigitCenti(range);
    if (mg_centi == 0U) {
        return 2;
    }

    cfg[0] = 0x00U;
    cfg[1] = (uint8_t)((samples << 5) | (data_rate << 2));
    if (!SensorI2C_Write(HMC5883L_ADDR, cfg, 2U)) {
        return 1;
    }

    cfg[0] = 0x01U;
    cfg[1] = (uint8_t)(range << 5);
    if (!SensorI2C_Write(HMC5883L_ADDR, cfg, 2U)) {
        return 1;
    }

    cfg[0] = 0x02U;
    cfg[1] = (uint8_t)(mode & 0x03U);
    if (!SensorI2C_Write(HMC5883L_ADDR, cfg, 2U)) {
        return 1;
    }

    if (mg_per_digit_centi != 0) {
        *mg_per_digit_centi = mg_centi;
    }
    return 0;
}

int SensorHMC5883L_Init(uint8_t range, uint8_t data_rate, uint8_t samples, uint8_t mode, uint16_t *mg_per_digit_centi)
{
    uint8_t id[3];

    if (!SensorI2C_MemRead(HMC5883L_ADDR, 0x0AU, id, 3U)) {
        return 1;
    }
    if (id[0] != 'H' || id[1] != '4' || id[2] != '3') {
        return 1;
    }

    return sensor_hmc5883l_apply_config(range, data_rate, samples, mode, mg_per_digit_centi);
}

int SensorHMC5883L_SetConfig(uint8_t range, uint8_t data_rate, uint8_t samples, uint8_t mode, uint16_t *mg_per_digit_centi)
{
    return sensor_hmc5883l_apply_config(range, data_rate, samples, mode, mg_per_digit_centi);
}

int SensorHMC5883L_ReadNormalizedMg(int16_t off_x, int16_t off_y, int16_t off_z, uint16_t mg_per_digit_centi,
                                    int32_t *x_mg, int32_t *y_mg, int32_t *z_mg)
{
    uint8_t raw[6];
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t x_corr;
    int16_t y_corr;
    int16_t z_corr;

    if (x_mg == 0 || y_mg == 0 || z_mg == 0 || mg_per_digit_centi == 0U) {
        return 0;
    }

    if (!SensorI2C_MemRead(HMC5883L_ADDR, 0x03U, raw, 6U)) {
        return 0;
    }

    x = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
    z = (int16_t)(((uint16_t)raw[2] << 8) | raw[3]);
    y = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]);

    x_corr = (int16_t)(x - off_x);
    y_corr = (int16_t)(y - off_y);
    z_corr = (int16_t)(z - off_z);

    *x_mg = ((int32_t)x_corr * (int32_t)mg_per_digit_centi) / 100;
    *y_mg = ((int32_t)y_corr * (int32_t)mg_per_digit_centi) / 100;
    *z_mg = ((int32_t)z_corr * (int32_t)mg_per_digit_centi) / 100;
    return 1;
}
