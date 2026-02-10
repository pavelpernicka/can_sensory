#pragma once

#include "app_config.h"
#include "calibration.h"

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t valid;
} app_mag_data_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t valid;
} app_acc_data_t;

typedef struct {
    int16_t temp_centi_c;
    uint16_t rh_centi_pct;
    uint8_t valid;
} app_env_data_t;

typedef struct {
    uint8_t hmc_present;
    uint8_t lis_present;
    uint8_t aht_present;
} app_sensor_status_t;

typedef struct {
    uint8_t status;
    uint8_t crc_ok;
    uint32_t raw_temp;
    uint32_t raw_hum;
    int16_t temp_centi_c;
    uint16_t rh_centi_pct;
} app_aht20_diag_t;

typedef struct {
    uint8_t range;
    uint8_t data_rate;
    uint8_t samples;
    uint8_t mode;
    uint16_t mg_per_digit_centi;
} app_hmc_cfg_t;

void Sensors_Init(I2C_HandleTypeDef *hi2c);
int Sensors_ReadMag(app_mag_data_t *out);
int Sensors_ReadAcc(app_acc_data_t *out);
void Sensors_ServiceEnv(uint32_t now_ms);
const app_env_data_t *Sensors_GetEnv(void);
const app_sensor_status_t *Sensors_GetStatus(void);
int Sensors_AHT20_Read(app_aht20_diag_t *out);
int Sensors_AHT20_GetStatus(uint8_t *status);
int Sensors_AHT20_Reset(void);
int Sensors_AHT20_SetReg(const uint8_t *buf, uint8_t len);
int Sensors_AHT20_GetReg(uint8_t *buf, uint8_t len);
void Sensors_AHT20_GetLast(app_aht20_diag_t *out);
void Sensors_ApplyCalibration(const app_calibration_t *cal);
void Sensors_GetAppliedCalibration(app_calibration_t *cal);
int Sensors_CaptureEarthField(int16_t *x_mg, int16_t *y_mg, int16_t *z_mg);
int Sensors_HMC_SetConfig(uint8_t range, uint8_t data_rate, uint8_t samples, uint8_t mode);
void Sensors_HMC_GetConfig(app_hmc_cfg_t *cfg);
