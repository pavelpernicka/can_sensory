#pragma once

#include "app_config.h"

typedef struct {
    int16_t center_x_mg;
    int16_t center_y_mg;
    int16_t center_z_mg;
    int16_t rotate_xy_cdeg;
    int16_t rotate_xz_cdeg;
    int16_t rotate_yz_cdeg;
    uint16_t keepout_rad_mg;
    int16_t z_limit_mg;
    uint16_t data_radius_mg;
    int16_t mag_offset_x;
    int16_t mag_offset_y;
    int16_t mag_offset_z;
    int16_t earth_x_mg;
    int16_t earth_y_mg;
    int16_t earth_z_mg;
    uint8_t earth_valid;
    uint8_t stream_enable_mask;
    uint16_t interval_mag_ms;
    uint16_t interval_acc_ms;
    uint16_t interval_env_ms;
    uint16_t interval_event_ms;
    uint8_t num_sectors;
    uint8_t hmc_range;
    uint8_t hmc_data_rate;
    uint8_t hmc_samples;
    uint8_t hmc_mode;
    uint16_t reserved0;
} app_calibration_t;

typedef enum {
    APP_CAL_FIELD_CENTER_X = 1,
    APP_CAL_FIELD_CENTER_Y = 2,
    APP_CAL_FIELD_CENTER_Z = 3,
    APP_CAL_FIELD_ROTATE_XY = 4,
    APP_CAL_FIELD_ROTATE_XZ = 5,
    APP_CAL_FIELD_ROTATE_YZ = 6,
    APP_CAL_FIELD_KEEPOUT_RAD = 7,
    APP_CAL_FIELD_Z_LIMIT = 8,
    APP_CAL_FIELD_DATA_RADIUS = 9,
    APP_CAL_FIELD_MAG_OFFSET_X = 10,
    APP_CAL_FIELD_MAG_OFFSET_Y = 11,
    APP_CAL_FIELD_MAG_OFFSET_Z = 12,
    APP_CAL_FIELD_EARTH_X = 13,
    APP_CAL_FIELD_EARTH_Y = 14,
    APP_CAL_FIELD_EARTH_Z = 15,
    APP_CAL_FIELD_EARTH_VALID = 16,
    APP_CAL_FIELD_NUM_SECTORS = 17
} app_cal_field_t;

#define APP_CAL_FIELD_FIRST APP_CAL_FIELD_CENTER_X
#define APP_CAL_FIELD_LAST  APP_CAL_FIELD_NUM_SECTORS

void Calib_Init(void);
const app_calibration_t *Calib_Get(void);
void Calib_ResetToDefaults(void);
int Calib_LoadFromFlash(void);
int Calib_SaveToFlash(void);
int Calib_SetField(uint8_t field, int16_t value);
int Calib_GetField(uint8_t field, int16_t *value);
void Calib_SetEarth(int16_t x_mg, int16_t y_mg, int16_t z_mg, uint8_t valid);
void Calib_SetStreamConfig(uint16_t mag_ms, uint16_t acc_ms, uint16_t env_ms, uint16_t evt_ms, uint8_t enable_mask);
void Calib_GetStreamConfig(uint16_t *mag_ms, uint16_t *acc_ms, uint16_t *env_ms, uint16_t *evt_ms, uint8_t *enable_mask);
void Calib_SetHmcConfig(uint8_t range, uint8_t data_rate, uint8_t samples, uint8_t mode);
void Calib_GetHmcConfig(uint8_t *range, uint8_t *data_rate, uint8_t *samples, uint8_t *mode);
