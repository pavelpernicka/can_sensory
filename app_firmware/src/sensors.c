#include "sensors.h"

static I2C_HandleTypeDef *g_hi2c;
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

#define HMC5883L_ADDR 0x1EU
#define LIS3DHTR_ADDR 0x19U
#define AHT20_ADDR    0x38U

#define HMC5883L_RANGE_DEFAULT      7U
#define HMC5883L_DATA_RATE_DEFAULT  4U
#define HMC5883L_SAMPLES_DEFAULT    3U
#define HMC5883L_MODE_DEFAULT       0U

#define AHT20_CMD_STATUS   0x71U
#define AHT20_CMD_INIT     0xBEU
#define AHT20_CMD_MEASURE  0xACU

static int i2c_write_bytes(uint8_t addr7, const uint8_t *data, uint16_t len)
{
    return HAL_I2C_Master_Transmit(g_hi2c, (uint16_t)(addr7 << 1), (uint8_t *)data, len, APP_I2C_TIMEOUT_MS) == HAL_OK;
}

static int i2c_read_bytes(uint8_t addr7, uint8_t *data, uint16_t len)
{
    return HAL_I2C_Master_Receive(g_hi2c, (uint16_t)(addr7 << 1), data, len, APP_I2C_TIMEOUT_MS) == HAL_OK;
}

static int i2c_mem_read(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t len)
{
    return HAL_I2C_Mem_Read(g_hi2c, (uint16_t)(addr7 << 1), reg, I2C_MEMADD_SIZE_8BIT, data, len, APP_I2C_TIMEOUT_MS) == HAL_OK;
}

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

static uint16_t hmc_range_to_mg_centi(uint8_t range)
{
    static const uint16_t mg_per_digit_centi[8] = {
        73U, 92U, 122U, 152U, 227U, 256U, 303U, 435U
    };

    if (range > 7U) {
        return 0U;
    }
    return mg_per_digit_centi[range];
}

static int hmc_cfg_valid(uint8_t range, uint8_t data_rate, uint8_t samples, uint8_t mode)
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

static int hmc_apply_config_hw(void)
{
    uint8_t cfg[2];
    uint16_t mg_centi;

    if (!hmc_cfg_valid(g_hmc_range, g_hmc_data_rate, g_hmc_samples, g_hmc_mode)) {
        return 2;
    }

    mg_centi = hmc_range_to_mg_centi(g_hmc_range);
    if (mg_centi == 0U) {
        return 2;
    }

    cfg[0] = 0x00U;
    cfg[1] = (uint8_t)((g_hmc_samples << 5) | (g_hmc_data_rate << 2));
    if (!i2c_write_bytes(HMC5883L_ADDR, cfg, 2U)) {
        return 1;
    }

    cfg[0] = 0x01U;
    cfg[1] = (uint8_t)(g_hmc_range << 5);
    if (!i2c_write_bytes(HMC5883L_ADDR, cfg, 2U)) {
        return 1;
    }

    cfg[0] = 0x02U;
    cfg[1] = (uint8_t)(g_hmc_mode & 0x03U);
    if (!i2c_write_bytes(HMC5883L_ADDR, cfg, 2U)) {
        return 1;
    }

    g_hmc_mg_per_digit_centi = mg_centi;
    return 0;
}

static uint8_t aht20_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFFU;
    uint8_t byte;
    uint8_t i;

    for (byte = 0U; byte < len; ++byte) {
        crc ^= data[byte];
        for (i = 0U; i < 8U; ++i) {
            if ((crc & 0x80U) != 0U) {
                crc = (uint8_t)((crc << 1U) ^ 0x31U);
            } else {
                crc <<= 1U;
            }
        }
    }

    return crc;
}

static int aht20_get_status_raw(uint8_t *status)
{
    uint8_t cmd = AHT20_CMD_STATUS;
    if (!i2c_write_bytes(AHT20_ADDR, &cmd, 1U)) {
        return 1;
    }
    if (!i2c_read_bytes(AHT20_ADDR, status, 1U)) {
        return 1;
    }
    return 0;
}

static int aht20_jh_reset_reg(uint8_t addr)
{
    uint8_t cmd[3];
    uint8_t regs[3];

    cmd[0] = addr;
    cmd[1] = 0x00U;
    cmd[2] = 0x00U;
    if (!i2c_write_bytes(AHT20_ADDR, cmd, 3U)) {
        return 1;
    }
    HAL_Delay(5U);

    if (!i2c_read_bytes(AHT20_ADDR, regs, 3U)) {
        return 1;
    }
    HAL_Delay(10U);

    cmd[0] = (uint8_t)(0xB0U | addr);
    cmd[1] = regs[1];
    cmd[2] = regs[2];
    if (!i2c_write_bytes(AHT20_ADDR, cmd, 3U)) {
        return 1;
    }

    return 0;
}

static int aht20_reset_regs(void)
{
    if (aht20_jh_reset_reg(0x1BU) != 0) {
        return 1;
    }
    if (aht20_jh_reset_reg(0x1CU) != 0) {
        return 1;
    }
    if (aht20_jh_reset_reg(0x1EU) != 0) {
        return 1;
    }
    return 0;
}

static void aht20_decode_payload(const uint8_t *buf, uint8_t crc_ok, app_aht20_diag_t *out)
{
    uint32_t raw_h = ((uint32_t)buf[1] << 12) | ((uint32_t)buf[2] << 4) | ((uint32_t)buf[3] >> 4);
    uint32_t raw_t = (((uint32_t)buf[3] & 0x0FU) << 16) | ((uint32_t)buf[4] << 8) | (uint32_t)buf[5];
    uint64_t rh_centi = ((uint64_t)raw_h * 10000ULL) / 1048576ULL;
    int32_t temp_centi = (int32_t)(((int64_t)raw_t * 20000LL) / 1048576LL) - 5000;

    if (rh_centi > 10000ULL) {
        rh_centi = 10000ULL;
    }

    out->status = buf[0];
    out->crc_ok = crc_ok;
    out->raw_hum = raw_h;
    out->raw_temp = raw_t;
    out->rh_centi_pct = (uint16_t)rh_centi;
    out->temp_centi_c = (int16_t)temp_centi;
}

static int aht20_read_after_trigger(app_aht20_diag_t *out)
{
    uint8_t buf[7];
    uint8_t crc;

    if (!i2c_read_bytes(AHT20_ADDR, buf, 7U)) {
        return 1;
    }
    if ((buf[0] & 0x80U) != 0U) {
        out->status = buf[0];
        out->crc_ok = 0U;
        return 4;
    }

    crc = aht20_crc8(buf, 6U);
    aht20_decode_payload(buf, (uint8_t)(crc == buf[6]), out);
    if (crc != buf[6]) {
        return 5;
    }
    return 0;
}

static int aht20_read_blocking(app_aht20_diag_t *out)
{
    uint8_t cmd[3] = {AHT20_CMD_MEASURE, 0x33U, 0x00U};
    uint8_t status;
    uint8_t buf[7];
    uint8_t crc;

    if (!i2c_write_bytes(AHT20_ADDR, cmd, 3U)) {
        return 1;
    }
    HAL_Delay(85U);

    if (!i2c_read_bytes(AHT20_ADDR, &status, 1U)) {
        return 1;
    }
    if ((status & 0x80U) != 0U) {
        out->status = status;
        out->crc_ok = 0U;
        return 4;
    }

    if (!i2c_read_bytes(AHT20_ADDR, buf, 7U)) {
        return 1;
    }

    crc = aht20_crc8(buf, 6U);
    aht20_decode_payload(buf, (uint8_t)(crc == buf[6]), out);
    if (crc != buf[6]) {
        return 5;
    }

    return 0;
}

static int hmc_read_normalized_mg(int32_t *x_mg, int32_t *y_mg, int32_t *z_mg, uint8_t subtract_earth)
{
    uint8_t raw[6];
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t x_corr;
    int16_t y_corr;
    int16_t z_corr;
    int32_t mx;
    int32_t my;
    int32_t mz;

    if (!i2c_mem_read(HMC5883L_ADDR, 0x03, raw, 6U)) {
        return 0;
    }

    x = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
    z = (int16_t)(((uint16_t)raw[2] << 8) | raw[3]);
    y = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]);

    x_corr = (int16_t)(x - g_hmc_offset_x);
    y_corr = (int16_t)(y - g_hmc_offset_y);
    z_corr = (int16_t)(z - g_hmc_offset_z);

    mx = ((int32_t)x_corr * (int32_t)g_hmc_mg_per_digit_centi) / 100;
    my = ((int32_t)y_corr * (int32_t)g_hmc_mg_per_digit_centi) / 100;
    mz = ((int32_t)z_corr * (int32_t)g_hmc_mg_per_digit_centi) / 100;

    if (subtract_earth && g_hmc_earth_valid) {
        mx -= g_hmc_earth_x_mg;
        my -= g_hmc_earth_y_mg;
        mz -= g_hmc_earth_z_mg;
    }

    *x_mg = mx;
    *y_mg = my;
    *z_mg = mz;
    return 1;
}

static void sensor_init_hmc(void)
{
    uint8_t id[3] = {0};

    g_sensor_status.hmc_present = 0;
    if (!i2c_mem_read(HMC5883L_ADDR, 0x0A, id, 3U)) {
        return;
    }
    if (id[0] != 'H' || id[1] != '4' || id[2] != '3') {
        return;
    }

    if (hmc_apply_config_hw() != 0) {
        return;
    }

    g_sensor_status.hmc_present = 1;
}

static void sensor_init_lis(void)
{
    uint8_t who = 0;
    uint8_t cfg[2];

    g_sensor_status.lis_present = 0;
    if (!i2c_mem_read(LIS3DHTR_ADDR, 0x0F, &who, 1U)) {
        return;
    }
    if (who != 0x33U) {
        return;
    }

    cfg[0] = 0x20U; cfg[1] = 0x57U; /* 100 Hz, XYZ enable */
    if (!i2c_write_bytes(LIS3DHTR_ADDR, cfg, 2U)) {
        return;
    }
    cfg[0] = 0x23U; cfg[1] = 0x00U; /* +/-2g, normal mode */
    if (!i2c_write_bytes(LIS3DHTR_ADDR, cfg, 2U)) {
        return;
    }

    g_sensor_status.lis_present = 1;
}

static void sensor_init_aht(void)
{
    uint8_t status = 0U;
    uint8_t init_cmd[3] = {AHT20_CMD_INIT, 0x08U, 0x00U};

    g_sensor_status.aht_present = 0;
    HAL_Delay(500U);

    if (aht20_get_status_raw(&status) != 0) {
        return;
    }

    if ((status & 0x18U) != 0x18U) {
        if (aht20_reset_regs() != 0) {
            return;
        }
        HAL_Delay(10U);
        if (aht20_get_status_raw(&status) != 0) {
            return;
        }
        if ((status & 0x18U) != 0x18U) {
            if (!i2c_write_bytes(AHT20_ADDR, init_cmd, 3U)) {
                return;
            }
            HAL_Delay(20U);
            if (aht20_get_status_raw(&status) != 0) {
                return;
            }
        }
    }

    g_aht_last.status = status;
    g_sensor_status.aht_present = 1;
}

void Sensors_Init(I2C_HandleTypeDef *hi2c)
{
    g_hi2c = hi2c;

    g_hi2c->Instance = APP_I2C_INSTANCE;
    g_hi2c->Init.Timing = APP_I2C_TIMING;
    g_hi2c->Init.OwnAddress1 = 0U;
    g_hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    g_hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    g_hi2c->Init.OwnAddress2 = 0U;
    g_hi2c->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    g_hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    g_hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    HAL_I2C_Init(g_hi2c);
    HAL_I2CEx_ConfigAnalogFilter(g_hi2c, I2C_ANALOGFILTER_ENABLE);

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
    g_hmc_mg_per_digit_centi = hmc_range_to_mg_centi(HMC5883L_RANGE_DEFAULT);

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
    if (!hmc_read_normalized_mg(&x_mg, &y_mg, &z_mg, 1U)) {
        out->valid = 0;
        return 0;
    }

    out->x = clamp_i16(x_mg);
    out->y = clamp_i16(y_mg);
    out->z = clamp_i16(z_mg);
    out->valid = 1;
    return 1;
}

int Sensors_ReadAcc(app_acc_data_t *out)
{
    uint8_t raw[6];
    int16_t x_raw;
    int16_t y_raw;
    int16_t z_raw;
    int16_t x_counts;
    int16_t y_counts;
    int16_t z_counts;

    if (out == 0 || !g_sensor_status.lis_present) {
        return 0;
    }
    if (!i2c_mem_read(LIS3DHTR_ADDR, 0xA8, raw, 6U)) {
        out->valid = 0;
        return 0;
    }

    x_raw = (int16_t)(((uint16_t)raw[1] << 8) | raw[0]);
    y_raw = (int16_t)(((uint16_t)raw[3] << 8) | raw[2]);
    z_raw = (int16_t)(((uint16_t)raw[5] << 8) | raw[4]);

    x_counts = (int16_t)(x_raw >> 6);
    y_counts = (int16_t)(y_raw >> 6);
    z_counts = (int16_t)(z_raw >> 6);

    out->x = (int16_t)(x_counts * 4); /* mg */
    out->y = (int16_t)(y_counts * 4);
    out->z = (int16_t)(z_counts * 4);
    out->valid = 1;
    return 1;
}

void Sensors_ServiceEnv(uint32_t now_ms)
{
    uint8_t cmd[3] = {AHT20_CMD_MEASURE, 0x33U, 0x00U};
    app_aht20_diag_t diag;
    int st;

    if (!g_sensor_status.aht_present) {
        return;
    }

    if (g_env_state == ENV_IDLE) {
        if (now_ms < g_env_next_trigger_ms) {
            return;
        }
        if (!i2c_write_bytes(AHT20_ADDR, cmd, 3U)) {
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

    st = aht20_read_after_trigger(&diag);
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

    st = aht20_read_blocking(&diag);
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
    if (!g_sensor_status.aht_present) {
        return 3;
    }
    if (aht20_get_status_raw(&s) != 0) {
        return 1;
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
    if (aht20_reset_regs() != 0) {
        return 1;
    }
    HAL_Delay(10U);
    if (aht20_get_status_raw(&status) != 0) {
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
    if (buf == 0 || len == 0U || len > 7U) {
        return 2;
    }
    if (!i2c_write_bytes(AHT20_ADDR, buf, len)) {
        return 1;
    }
    return 0;
}

int Sensors_AHT20_GetReg(uint8_t *buf, uint8_t len)
{
    if (!g_sensor_status.aht_present) {
        return 3;
    }
    if (buf == 0 || len == 0U || len > 5U) {
        return 2;
    }
    if (!i2c_read_bytes(AHT20_ADDR, buf, len)) {
        return 1;
    }
    return 0;
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
        g_hmc_range = HMC5883L_RANGE_DEFAULT;
        g_hmc_data_rate = HMC5883L_DATA_RATE_DEFAULT;
        g_hmc_samples = HMC5883L_SAMPLES_DEFAULT;
        g_hmc_mode = HMC5883L_MODE_DEFAULT;
        g_hmc_mg_per_digit_centi = hmc_range_to_mg_centi(HMC5883L_RANGE_DEFAULT);
        if (g_sensor_status.hmc_present) {
            (void)hmc_apply_config_hw();
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
    if (!hmc_read_normalized_mg(&mx, &my, &mz, 0U)) {
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
    int st;

    if (!hmc_cfg_valid(range, data_rate, samples, mode)) {
        return 2;
    }
    if (!g_sensor_status.hmc_present) {
        return 3;
    }

    g_hmc_range = range;
    g_hmc_data_rate = data_rate;
    g_hmc_samples = samples;
    g_hmc_mode = mode;

    st = hmc_apply_config_hw();
    if (st != 0) {
        return st;
    }

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
