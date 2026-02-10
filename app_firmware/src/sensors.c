#include "sensors.h"

#include "sensor_aht20.h"
#include "sensor_hmc5883l.h"
#include "sensor_i2c.h"
#include "sensor_lis3dhtr.h"

static app_sensor_status_t g_sensor_status;
static app_env_data_t g_env;
static app_aht20_diag_t g_aht_last;
static int16_t g_hmc_offset_x;
static int16_t g_hmc_offset_y;
static int16_t g_hmc_offset_z;
static int32_t g_hmc_earth_x_mg;
static int32_t g_hmc_earth_y_mg;
static int32_t g_hmc_earth_z_mg;
static uint8_t g_hmc_earth_valid;
static uint8_t g_hmc_range;
static uint8_t g_hmc_data_rate;
static uint8_t g_hmc_samples;
static uint8_t g_hmc_mode;
static uint16_t g_hmc_mg_per_digit_centi;

typedef enum {
    ENV_IDLE = 0,
    ENV_WAIT
} env_state_t;

static env_state_t g_env_state;
static uint32_t g_env_next_trigger_ms;
static uint32_t g_env_ready_ms;
static uint8_t g_env_retry;

#define HMC5883L_RANGE_DEFAULT      7U
#define HMC5883L_DATA_RATE_DEFAULT  4U
#define HMC5883L_SAMPLES_DEFAULT    3U
#define HMC5883L_MODE_DEFAULT       0U

static int16_t clamp_i16(int32_t v)
{
    if (v > 32767) {
        return 32767;
    }
    if (v < -32768) {
        return -32768;
    }
    return (int16_t)v;
}

static void sensor_init_hmc(void)
{
    uint16_t mg = g_hmc_mg_per_digit_centi;

    g_sensor_status.hmc_present = 0U;
    if (SensorHMC5883L_Init(g_hmc_range, g_hmc_data_rate, g_hmc_samples, g_hmc_mode, &mg) != 0) {
        return;
    }

    g_hmc_mg_per_digit_centi = mg;
    g_sensor_status.hmc_present = 1U;
}

static void sensor_init_lis(void)
{
    g_sensor_status.lis_present = SensorLIS3DHTR_Init() ? 1U : 0U;
}

static void sensor_init_aht(void)
{
    uint8_t status = 0U;

    g_sensor_status.aht_present = 0U;
    if (SensorAHT20_Init(&status) != 0) {
        return;
    }

    g_aht_last.status = status;
    g_sensor_status.aht_present = 1U;
}

void Sensors_Init(I2C_HandleTypeDef *hi2c)
{
    g_sensor_status.hmc_present = 0U;
    g_sensor_status.lis_present = 0U;
    g_sensor_status.aht_present = 0U;

    g_env.valid = 0U;
    g_env_state = ENV_IDLE;
    g_env_next_trigger_ms = HAL_GetTick() + 200U;
    g_env_ready_ms = 0U;
    g_env_retry = 0U;

    g_aht_last.status = 0U;
    g_aht_last.crc_ok = 0U;
    g_aht_last.raw_hum = 0U;
    g_aht_last.raw_temp = 0U;
    g_aht_last.rh_centi_pct = 0U;
    g_aht_last.temp_centi_c = 0;

    g_hmc_offset_x = 0;
    g_hmc_offset_y = 0;
    g_hmc_offset_z = 0;
    g_hmc_earth_x_mg = 0;
    g_hmc_earth_y_mg = 0;
    g_hmc_earth_z_mg = 0;
    g_hmc_earth_valid = 0U;
    g_hmc_range = HMC5883L_RANGE_DEFAULT;
    g_hmc_data_rate = HMC5883L_DATA_RATE_DEFAULT;
    g_hmc_samples = HMC5883L_SAMPLES_DEFAULT;
    g_hmc_mode = HMC5883L_MODE_DEFAULT;
    g_hmc_mg_per_digit_centi = SensorHMC5883L_RangeToMgPerDigitCenti(HMC5883L_RANGE_DEFAULT);

    if (!SensorI2C_Init(hi2c)) {
        return;
    }

    sensor_init_hmc();
    sensor_init_lis();
    sensor_init_aht();
}

int Sensors_ReadMag(app_mag_data_t *out)
{
    int32_t x_mg;
    int32_t y_mg;
    int32_t z_mg;

    if (out == 0 || !g_sensor_status.hmc_present) {
        return 0;
    }

    if (!SensorHMC5883L_ReadNormalizedMg(g_hmc_offset_x, g_hmc_offset_y, g_hmc_offset_z, g_hmc_mg_per_digit_centi,
                                         &x_mg, &y_mg, &z_mg)) {
        out->valid = 0U;
        return 0;
    }

    if (g_hmc_earth_valid) {
        x_mg -= g_hmc_earth_x_mg;
        y_mg -= g_hmc_earth_y_mg;
        z_mg -= g_hmc_earth_z_mg;
    }

    out->x = clamp_i16(x_mg);
    out->y = clamp_i16(y_mg);
    out->z = clamp_i16(z_mg);
    out->valid = 1U;
    return 1;
}

int Sensors_ReadAcc(app_acc_data_t *out)
{
    int16_t x_mg;
    int16_t y_mg;
    int16_t z_mg;

    if (out == 0 || !g_sensor_status.lis_present) {
        return 0;
    }

    if (!SensorLIS3DHTR_ReadMg(&x_mg, &y_mg, &z_mg)) {
        out->valid = 0U;
        return 0;
    }

    out->x = x_mg;
    out->y = y_mg;
    out->z = z_mg;
    out->valid = 1U;
    return 1;
}

void Sensors_ServiceEnv(uint32_t now_ms)
{
    app_aht20_diag_t diag;
    int st;

    if (!g_sensor_status.aht_present) {
        return;
    }

    if (g_env_state == ENV_IDLE) {
        if (now_ms < g_env_next_trigger_ms) {
            return;
        }

        if (SensorAHT20_Trigger() != 0) {
            g_env_next_trigger_ms = now_ms + 200U;
            return;
        }

        g_env_state = ENV_WAIT;
        g_env_ready_ms = now_ms + APP_ENV_CONVERSION_MS;
        g_env_retry = 0U;
        return;
    }

    if (now_ms < g_env_ready_ms) {
        return;
    }

    st = SensorAHT20_ReadAfterTrigger(&diag);
    if (st == 0) {
        g_aht_last = diag;
        g_env.temp_centi_c = diag.temp_centi_c;
        g_env.rh_centi_pct = diag.rh_centi_pct;
        g_env.valid = diag.crc_ok ? 1U : 0U;

        g_env_state = ENV_IDLE;
        g_env_next_trigger_ms = now_ms + APP_ENV_TRIGGER_PERIOD_MS;
        return;
    }

    if (g_env_retry < 5U) {
        g_env_retry++;
        if (st == 4) {
            g_env_ready_ms = now_ms + 10U;
        } else {
            g_env_ready_ms = now_ms + 15U;
        }
        return;
    }

    g_env_state = ENV_IDLE;
    g_env_next_trigger_ms = now_ms + APP_ENV_TRIGGER_PERIOD_MS;
}

const app_env_data_t *Sensors_GetEnv(void)
{
    return &g_env;
}

const app_sensor_status_t *Sensors_GetStatus(void)
{
    return &g_sensor_status;
}

int Sensors_AHT20_Read(app_aht20_diag_t *out)
{
    app_aht20_diag_t diag = g_aht_last;
    int st;

    if (!g_sensor_status.aht_present) {
        return 3;
    }

    st = SensorAHT20_ReadBlocking(&diag);
    if (st == 0) {
        g_aht_last = diag;
    }

    if (out != 0) {
        *out = diag;
    }
    return st;
}

int Sensors_AHT20_GetStatus(uint8_t *status)
{
    uint8_t s = 0U;
    int st;

    if (!g_sensor_status.aht_present) {
        return 3;
    }

    st = SensorAHT20_GetStatus(&s);
    if (st != 0) {
        return st;
    }

    g_aht_last.status = s;
    if (status != 0) {
        *status = s;
    }
    return 0;
}

int Sensors_AHT20_Reset(void)
{
    uint8_t status = 0U;

    if (!g_sensor_status.aht_present) {
        return 3;
    }
    if (SensorAHT20_Reset() != 0) {
        return 1;
    }
    if (SensorAHT20_GetStatus(&status) != 0) {
        return 1;
    }

    g_aht_last.status = status;
    return 0;
}

int Sensors_AHT20_SetReg(const uint8_t *buf, uint8_t len)
{
    if (!g_sensor_status.aht_present) {
        return 3;
    }
    return SensorAHT20_SetReg(buf, len);
}

int Sensors_AHT20_GetReg(uint8_t *buf, uint8_t len)
{
    if (!g_sensor_status.aht_present) {
        return 3;
    }
    return SensorAHT20_GetReg(buf, len);
}

void Sensors_AHT20_GetLast(app_aht20_diag_t *out)
{
    if (out != 0) {
        *out = g_aht_last;
    }
}

void Sensors_ApplyCalibration(const app_calibration_t *cal)
{
    int st;

    if (cal == 0) {
        return;
    }

    g_hmc_offset_x = cal->mag_offset_x;
    g_hmc_offset_y = cal->mag_offset_y;
    g_hmc_offset_z = cal->mag_offset_z;
    g_hmc_earth_x_mg = cal->earth_x_mg;
    g_hmc_earth_y_mg = cal->earth_y_mg;
    g_hmc_earth_z_mg = cal->earth_z_mg;
    g_hmc_earth_valid = cal->earth_valid ? 1U : 0U;
    g_hmc_range = cal->hmc_range;
    g_hmc_data_rate = cal->hmc_data_rate;
    g_hmc_samples = cal->hmc_samples;
    g_hmc_mode = cal->hmc_mode;

    st = Sensors_HMC_SetConfig(g_hmc_range, g_hmc_data_rate, g_hmc_samples, g_hmc_mode);
    if (st != 0 && st != 3) {
        uint16_t mg;

        g_hmc_range = HMC5883L_RANGE_DEFAULT;
        g_hmc_data_rate = HMC5883L_DATA_RATE_DEFAULT;
        g_hmc_samples = HMC5883L_SAMPLES_DEFAULT;
        g_hmc_mode = HMC5883L_MODE_DEFAULT;
        g_hmc_mg_per_digit_centi = SensorHMC5883L_RangeToMgPerDigitCenti(HMC5883L_RANGE_DEFAULT);

        if (g_sensor_status.hmc_present) {
            mg = g_hmc_mg_per_digit_centi;
            if (SensorHMC5883L_SetConfig(g_hmc_range, g_hmc_data_rate, g_hmc_samples, g_hmc_mode, &mg) == 0) {
                g_hmc_mg_per_digit_centi = mg;
            }
        }
    }
}

void Sensors_GetAppliedCalibration(app_calibration_t *cal)
{
    if (cal == 0) {
        return;
    }

    cal->mag_offset_x = g_hmc_offset_x;
    cal->mag_offset_y = g_hmc_offset_y;
    cal->mag_offset_z = g_hmc_offset_z;
    cal->earth_x_mg = (int16_t)g_hmc_earth_x_mg;
    cal->earth_y_mg = (int16_t)g_hmc_earth_y_mg;
    cal->earth_z_mg = (int16_t)g_hmc_earth_z_mg;
    cal->earth_valid = g_hmc_earth_valid;
    cal->hmc_range = g_hmc_range;
    cal->hmc_data_rate = g_hmc_data_rate;
    cal->hmc_samples = g_hmc_samples;
    cal->hmc_mode = g_hmc_mode;
}

int Sensors_CaptureEarthField(int16_t *x_mg, int16_t *y_mg, int16_t *z_mg)
{
    int32_t mx;
    int32_t my;
    int32_t mz;

    if (!g_sensor_status.hmc_present) {
        return 0;
    }
    if (!SensorHMC5883L_ReadNormalizedMg(g_hmc_offset_x, g_hmc_offset_y, g_hmc_offset_z, g_hmc_mg_per_digit_centi,
                                         &mx, &my, &mz)) {
        return 0;
    }

    g_hmc_earth_x_mg = mx;
    g_hmc_earth_y_mg = my;
    g_hmc_earth_z_mg = mz;
    g_hmc_earth_valid = 1U;

    if (x_mg != 0) {
        *x_mg = (int16_t)mx;
    }
    if (y_mg != 0) {
        *y_mg = (int16_t)my;
    }
    if (z_mg != 0) {
        *z_mg = (int16_t)mz;
    }
    return 1;
}

int Sensors_HMC_SetConfig(uint8_t range, uint8_t data_rate, uint8_t samples, uint8_t mode)
{
    uint16_t mg;
    int st;

    if (!SensorHMC5883L_ConfigValid(range, data_rate, samples, mode)) {
        return 2;
    }
    if (!g_sensor_status.hmc_present) {
        return 3;
    }

    g_hmc_range = range;
    g_hmc_data_rate = data_rate;
    g_hmc_samples = samples;
    g_hmc_mode = mode;

    mg = g_hmc_mg_per_digit_centi;
    st = SensorHMC5883L_SetConfig(g_hmc_range, g_hmc_data_rate, g_hmc_samples, g_hmc_mode, &mg);
    if (st != 0) {
        return st;
    }

    g_hmc_mg_per_digit_centi = mg;
    return 0;
}

void Sensors_HMC_GetConfig(app_hmc_cfg_t *cfg)
{
    if (cfg == 0) {
        return;
    }

    cfg->range = g_hmc_range;
    cfg->data_rate = g_hmc_data_rate;
    cfg->samples = g_hmc_samples;
    cfg->mode = g_hmc_mode;
    cfg->mg_per_digit_centi = g_hmc_mg_per_digit_centi;
}
