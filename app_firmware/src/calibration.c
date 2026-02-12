#include "calibration.h"

#include <string.h>

#define APP_CALIB_MAGIC   0x43414C42UL
#define APP_CALIB_VERSION 3U
#define APP_CALIB_MIN_SECTORS 1U
#define APP_CALIB_MAX_SECTORS 16U
#define APP_CALIB_DEFAULT_SECTORS 6U

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    app_calibration_t cal;
    uint32_t crc32;
} app_calib_blob_t;

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
    uint8_t hmc_range;
    uint8_t hmc_data_rate;
    uint8_t hmc_samples;
    uint8_t hmc_mode;
    uint16_t reserved0;
} app_calibration_v2_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    app_calibration_v2_t cal;
    uint32_t crc32;
} app_calib_blob_v2_t;

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
    uint16_t reserved0;
} app_calibration_v1_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    app_calibration_v1_t cal;
    uint32_t crc32;
} app_calib_blob_v1_t;

static app_calibration_t g_cal;

static void calib_sanitize(app_calibration_t *cal)
{
    if (cal->num_sectors < APP_CALIB_MIN_SECTORS || cal->num_sectors > APP_CALIB_MAX_SECTORS) {
        cal->num_sectors = APP_CALIB_DEFAULT_SECTORS;
    }
}

static uint32_t crc32_soft(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    for (uint32_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint32_t bit = 0; bit < 8U; ++bit) {
            uint32_t mask = (uint32_t)-(int32_t)(crc & 1UL);
            crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
        }
    }
    return ~crc;
}

static void calib_set_defaults(app_calibration_t *cal)
{
    memset(cal, 0, sizeof(*cal));
    cal->center_x_mg = 0;
    cal->center_y_mg = 0;
    cal->center_z_mg = 0;
    cal->rotate_xy_cdeg = 0;
    cal->rotate_xz_cdeg = 0;
    cal->rotate_yz_cdeg = 0;
    cal->keepout_rad_mg = 1000U;
    cal->z_limit_mg = 150;
    cal->data_radius_mg = 3000U;
    cal->mag_offset_x = 0;
    cal->mag_offset_y = 0;
    cal->mag_offset_z = 0;
    cal->earth_x_mg = 0;
    cal->earth_y_mg = 0;
    cal->earth_z_mg = 0;
    cal->earth_valid = 0U;
    cal->stream_enable_mask = 0x0FU;
    cal->interval_mag_ms = APP_TX_INTERVAL_MAG_DEFAULT_MS;
    cal->interval_acc_ms = APP_TX_INTERVAL_ACC_DEFAULT_MS;
    cal->interval_env_ms = APP_TX_INTERVAL_ENV_DEFAULT_MS;
    cal->interval_event_ms = APP_TX_INTERVAL_EVT_DEFAULT_MS;
    cal->num_sectors = APP_CALIB_DEFAULT_SECTORS;
    cal->hmc_range = APP_HMC_DEFAULT_RANGE;
    cal->hmc_data_rate = APP_HMC_DEFAULT_DATA_RATE;
    cal->hmc_samples = APP_HMC_DEFAULT_SAMPLES;
    cal->hmc_mode = APP_HMC_DEFAULT_MODE;
}

void Calib_Init(void)
{
    calib_set_defaults(&g_cal);
    (void)Calib_LoadFromFlash();
}

const app_calibration_t *Calib_Get(void)
{
    return &g_cal;
}

void Calib_ResetToDefaults(void)
{
    calib_set_defaults(&g_cal);
}

int Calib_LoadFromFlash(void)
{
    const app_calib_blob_t *blob = (const app_calib_blob_t *)APP_CALIB_FLASH_ADDR;
    const app_calib_blob_v2_t *blob_v2 = (const app_calib_blob_v2_t *)APP_CALIB_FLASH_ADDR;
    const app_calib_blob_v1_t *blob_v1 = (const app_calib_blob_v1_t *)APP_CALIB_FLASH_ADDR;
    uint32_t expected_crc;

    if (blob->magic != APP_CALIB_MAGIC) {
        return 1;
    }
    if (blob->version == APP_CALIB_VERSION) {
        if (blob->size != (uint16_t)sizeof(app_calibration_t)) {
            return 3;
        }

        expected_crc = crc32_soft((const uint8_t *)&blob->version, (uint32_t)(sizeof(*blob) - 8U));
        if (expected_crc != blob->crc32) {
            return 4;
        }

        g_cal = blob->cal;
        calib_sanitize(&g_cal);
        return 0;
    }

    if (blob_v2->version == 2U) {
        if (blob_v2->size != (uint16_t)sizeof(app_calibration_v2_t)) {
            return 3;
        }

        expected_crc = crc32_soft((const uint8_t *)&blob_v2->version, (uint32_t)(sizeof(*blob_v2) - 8U));
        if (expected_crc != blob_v2->crc32) {
            return 4;
        }

        g_cal.center_x_mg = blob_v2->cal.center_x_mg;
        g_cal.center_y_mg = blob_v2->cal.center_y_mg;
        g_cal.center_z_mg = blob_v2->cal.center_z_mg;
        g_cal.rotate_xy_cdeg = blob_v2->cal.rotate_xy_cdeg;
        g_cal.rotate_xz_cdeg = blob_v2->cal.rotate_xz_cdeg;
        g_cal.rotate_yz_cdeg = blob_v2->cal.rotate_yz_cdeg;
        g_cal.keepout_rad_mg = blob_v2->cal.keepout_rad_mg;
        g_cal.z_limit_mg = blob_v2->cal.z_limit_mg;
        g_cal.data_radius_mg = blob_v2->cal.data_radius_mg;
        g_cal.mag_offset_x = blob_v2->cal.mag_offset_x;
        g_cal.mag_offset_y = blob_v2->cal.mag_offset_y;
        g_cal.mag_offset_z = blob_v2->cal.mag_offset_z;
        g_cal.earth_x_mg = blob_v2->cal.earth_x_mg;
        g_cal.earth_y_mg = blob_v2->cal.earth_y_mg;
        g_cal.earth_z_mg = blob_v2->cal.earth_z_mg;
        g_cal.earth_valid = blob_v2->cal.earth_valid;
        g_cal.stream_enable_mask = blob_v2->cal.stream_enable_mask;
        g_cal.interval_mag_ms = blob_v2->cal.interval_mag_ms;
        g_cal.interval_acc_ms = blob_v2->cal.interval_acc_ms;
        g_cal.interval_env_ms = blob_v2->cal.interval_env_ms;
        g_cal.interval_event_ms = blob_v2->cal.interval_event_ms;
        g_cal.hmc_range = blob_v2->cal.hmc_range;
        g_cal.hmc_data_rate = blob_v2->cal.hmc_data_rate;
        g_cal.hmc_samples = blob_v2->cal.hmc_samples;
        g_cal.hmc_mode = blob_v2->cal.hmc_mode;
        g_cal.reserved0 = blob_v2->cal.reserved0;
        g_cal.num_sectors = APP_CALIB_DEFAULT_SECTORS;
        calib_sanitize(&g_cal);
        return 0;
    }

    if (blob_v1->version == 1U) {
        if (blob_v1->size != (uint16_t)sizeof(app_calibration_v1_t)) {
            return 3;
        }

        expected_crc = crc32_soft((const uint8_t *)&blob_v1->version, (uint32_t)(sizeof(*blob_v1) - 8U));
        if (expected_crc != blob_v1->crc32) {
            return 4;
        }

        g_cal.center_x_mg = blob_v1->cal.center_x_mg;
        g_cal.center_y_mg = blob_v1->cal.center_y_mg;
        g_cal.center_z_mg = blob_v1->cal.center_z_mg;
        g_cal.rotate_xy_cdeg = blob_v1->cal.rotate_xy_cdeg;
        g_cal.rotate_xz_cdeg = blob_v1->cal.rotate_xz_cdeg;
        g_cal.rotate_yz_cdeg = blob_v1->cal.rotate_yz_cdeg;
        g_cal.keepout_rad_mg = blob_v1->cal.keepout_rad_mg;
        g_cal.z_limit_mg = blob_v1->cal.z_limit_mg;
        g_cal.data_radius_mg = blob_v1->cal.data_radius_mg;
        g_cal.mag_offset_x = blob_v1->cal.mag_offset_x;
        g_cal.mag_offset_y = blob_v1->cal.mag_offset_y;
        g_cal.mag_offset_z = blob_v1->cal.mag_offset_z;
        g_cal.earth_x_mg = blob_v1->cal.earth_x_mg;
        g_cal.earth_y_mg = blob_v1->cal.earth_y_mg;
        g_cal.earth_z_mg = blob_v1->cal.earth_z_mg;
        g_cal.earth_valid = blob_v1->cal.earth_valid;
        g_cal.stream_enable_mask = blob_v1->cal.stream_enable_mask;
        g_cal.interval_mag_ms = blob_v1->cal.interval_mag_ms;
        g_cal.interval_acc_ms = blob_v1->cal.interval_acc_ms;
        g_cal.interval_env_ms = blob_v1->cal.interval_env_ms;
        g_cal.interval_event_ms = blob_v1->cal.interval_event_ms;
        g_cal.reserved0 = blob_v1->cal.reserved0;
        g_cal.num_sectors = APP_CALIB_DEFAULT_SECTORS;
        calib_sanitize(&g_cal);
        return 0;
    }

    return 2;
}

int Calib_SaveToFlash(void)
{
    app_calib_blob_t blob;
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error = 0;
    uint32_t page = (APP_CALIB_FLASH_ADDR - FLASH_BASE) / APP_CALIB_FLASH_PAGE_SIZE;
    uint8_t *p8;
    uint32_t addr;

    blob.magic = APP_CALIB_MAGIC;
    blob.version = APP_CALIB_VERSION;
    blob.size = (uint16_t)sizeof(app_calibration_t);
    blob.cal = g_cal;
    blob.crc32 = crc32_soft((const uint8_t *)&blob.version, (uint32_t)(sizeof(blob) - 8U));

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Page = page;
    erase.NbPages = 1U;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK) {
        HAL_FLASH_Lock();
        return 5;
    }

    p8 = (uint8_t *)&blob;
    addr = APP_CALIB_FLASH_ADDR;
    for (uint32_t i = 0; i < sizeof(blob); i += 8U) {
        uint64_t dw = 0xFFFFFFFFFFFFFFFFULL;
        uint8_t *dwb = (uint8_t *)&dw;

        for (uint32_t j = 0; j < 8U && (i + j) < sizeof(blob); ++j) {
            dwb[j] = p8[i + j];
        }

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, dw) != HAL_OK) {
            HAL_FLASH_Lock();
            return 6;
        }
        addr += 8U;
    }

    HAL_FLASH_Lock();
    return 0;
}

int Calib_SetField(uint8_t field, int16_t value)
{
    switch ((app_cal_field_t)field) {
    case APP_CAL_FIELD_CENTER_X:
        g_cal.center_x_mg = value;
        break;
    case APP_CAL_FIELD_CENTER_Y:
        g_cal.center_y_mg = value;
        break;
    case APP_CAL_FIELD_CENTER_Z:
        g_cal.center_z_mg = value;
        break;
    case APP_CAL_FIELD_ROTATE_XY:
        g_cal.rotate_xy_cdeg = value;
        break;
    case APP_CAL_FIELD_ROTATE_XZ:
        g_cal.rotate_xz_cdeg = value;
        break;
    case APP_CAL_FIELD_ROTATE_YZ:
        g_cal.rotate_yz_cdeg = value;
        break;
    case APP_CAL_FIELD_KEEPOUT_RAD:
        if (value < 0) {
            return 2;
        }
        g_cal.keepout_rad_mg = (uint16_t)value;
        break;
    case APP_CAL_FIELD_Z_LIMIT:
        g_cal.z_limit_mg = value;
        break;
    case APP_CAL_FIELD_DATA_RADIUS:
        if (value < 0) {
            return 2;
        }
        g_cal.data_radius_mg = (uint16_t)value;
        break;
    case APP_CAL_FIELD_MAG_OFFSET_X:
        g_cal.mag_offset_x = value;
        break;
    case APP_CAL_FIELD_MAG_OFFSET_Y:
        g_cal.mag_offset_y = value;
        break;
    case APP_CAL_FIELD_MAG_OFFSET_Z:
        g_cal.mag_offset_z = value;
        break;
    case APP_CAL_FIELD_EARTH_X:
        g_cal.earth_x_mg = value;
        break;
    case APP_CAL_FIELD_EARTH_Y:
        g_cal.earth_y_mg = value;
        break;
    case APP_CAL_FIELD_EARTH_Z:
        g_cal.earth_z_mg = value;
        break;
    case APP_CAL_FIELD_EARTH_VALID:
        g_cal.earth_valid = value ? 1U : 0U;
        break;
    case APP_CAL_FIELD_NUM_SECTORS:
        if (value < (int16_t)APP_CALIB_MIN_SECTORS || value > (int16_t)APP_CALIB_MAX_SECTORS) {
            return 2;
        }
        g_cal.num_sectors = (uint8_t)value;
        break;
    default:
        return 1;
    }
    return 0;
}

int Calib_GetField(uint8_t field, int16_t *value)
{
    if (value == 0) {
        return 2;
    }

    switch ((app_cal_field_t)field) {
    case APP_CAL_FIELD_CENTER_X:
        *value = g_cal.center_x_mg;
        break;
    case APP_CAL_FIELD_CENTER_Y:
        *value = g_cal.center_y_mg;
        break;
    case APP_CAL_FIELD_CENTER_Z:
        *value = g_cal.center_z_mg;
        break;
    case APP_CAL_FIELD_ROTATE_XY:
        *value = g_cal.rotate_xy_cdeg;
        break;
    case APP_CAL_FIELD_ROTATE_XZ:
        *value = g_cal.rotate_xz_cdeg;
        break;
    case APP_CAL_FIELD_ROTATE_YZ:
        *value = g_cal.rotate_yz_cdeg;
        break;
    case APP_CAL_FIELD_KEEPOUT_RAD:
        *value = (int16_t)g_cal.keepout_rad_mg;
        break;
    case APP_CAL_FIELD_Z_LIMIT:
        *value = g_cal.z_limit_mg;
        break;
    case APP_CAL_FIELD_DATA_RADIUS:
        *value = (int16_t)g_cal.data_radius_mg;
        break;
    case APP_CAL_FIELD_MAG_OFFSET_X:
        *value = g_cal.mag_offset_x;
        break;
    case APP_CAL_FIELD_MAG_OFFSET_Y:
        *value = g_cal.mag_offset_y;
        break;
    case APP_CAL_FIELD_MAG_OFFSET_Z:
        *value = g_cal.mag_offset_z;
        break;
    case APP_CAL_FIELD_EARTH_X:
        *value = g_cal.earth_x_mg;
        break;
    case APP_CAL_FIELD_EARTH_Y:
        *value = g_cal.earth_y_mg;
        break;
    case APP_CAL_FIELD_EARTH_Z:
        *value = g_cal.earth_z_mg;
        break;
    case APP_CAL_FIELD_EARTH_VALID:
        *value = (int16_t)g_cal.earth_valid;
        break;
    case APP_CAL_FIELD_NUM_SECTORS:
        *value = (int16_t)g_cal.num_sectors;
        break;
    default:
        return 1;
    }
    return 0;
}

void Calib_SetEarth(int16_t x_mg, int16_t y_mg, int16_t z_mg, uint8_t valid)
{
    g_cal.earth_x_mg = x_mg;
    g_cal.earth_y_mg = y_mg;
    g_cal.earth_z_mg = z_mg;
    g_cal.earth_valid = valid ? 1U : 0U;
}

void Calib_SetStreamConfig(uint16_t mag_ms, uint16_t acc_ms, uint16_t env_ms, uint16_t evt_ms, uint8_t enable_mask)
{
    if (mag_ms > 60000U) mag_ms = 60000U;
    if (acc_ms > 60000U) acc_ms = 60000U;
    if (env_ms > 60000U) env_ms = 60000U;
    if (evt_ms > 60000U) evt_ms = 60000U;

    g_cal.interval_mag_ms = mag_ms;
    g_cal.interval_acc_ms = acc_ms;
    g_cal.interval_env_ms = env_ms;
    g_cal.interval_event_ms = evt_ms;
    g_cal.stream_enable_mask = (uint8_t)(enable_mask & 0x0FU);
}

void Calib_GetStreamConfig(uint16_t *mag_ms, uint16_t *acc_ms, uint16_t *env_ms, uint16_t *evt_ms, uint8_t *enable_mask)
{
    if (mag_ms != 0) *mag_ms = g_cal.interval_mag_ms;
    if (acc_ms != 0) *acc_ms = g_cal.interval_acc_ms;
    if (env_ms != 0) *env_ms = g_cal.interval_env_ms;
    if (evt_ms != 0) *evt_ms = g_cal.interval_event_ms;
    if (enable_mask != 0) *enable_mask = g_cal.stream_enable_mask;
}

void Calib_SetHmcConfig(uint8_t range, uint8_t data_rate, uint8_t samples, uint8_t mode)
{
    g_cal.hmc_range = range;
    g_cal.hmc_data_rate = data_rate;
    g_cal.hmc_samples = samples;
    g_cal.hmc_mode = mode;
}

void Calib_GetHmcConfig(uint8_t *range, uint8_t *data_rate, uint8_t *samples, uint8_t *mode)
{
    if (range != 0) *range = g_cal.hmc_range;
    if (data_rate != 0) *data_rate = g_cal.hmc_data_rate;
    if (samples != 0) *samples = g_cal.hmc_samples;
    if (mode != 0) *mode = g_cal.hmc_mode;
}
