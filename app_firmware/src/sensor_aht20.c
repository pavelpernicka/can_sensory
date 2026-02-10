#include "sensor_aht20.h"

#include "sensor_i2c.h"

#define AHT20_ADDR         0x38U
#define AHT20_CMD_STATUS   0x71U
#define AHT20_CMD_INIT     0xBEU
#define AHT20_CMD_MEASURE  0xACU

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

static void aht20_decode_payload(const uint8_t *buf, uint8_t crc_ok, app_aht20_diag_t *out)
{
    uint32_t raw_h;
    uint32_t raw_t;
    uint64_t rh_centi;
    int32_t temp_centi;

    raw_h = ((uint32_t)buf[1] << 12) | ((uint32_t)buf[2] << 4) | ((uint32_t)buf[3] >> 4);
    raw_t = (((uint32_t)buf[3] & 0x0FU) << 16) | ((uint32_t)buf[4] << 8) | (uint32_t)buf[5];
    rh_centi = ((uint64_t)raw_h * 10000ULL) / 1048576ULL;
    temp_centi = (int32_t)(((int64_t)raw_t * 20000LL) / 1048576LL) - 5000;

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

int SensorAHT20_GetStatus(uint8_t *status)
{
    uint8_t cmd = AHT20_CMD_STATUS;
    uint8_t s = 0U;

    if (!SensorI2C_Write(AHT20_ADDR, &cmd, 1U)) {
        return 1;
    }
    if (!SensorI2C_Read(AHT20_ADDR, &s, 1U)) {
        return 1;
    }
    if (status != 0) {
        *status = s;
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
    if (!SensorI2C_Write(AHT20_ADDR, cmd, 3U)) {
        return 1;
    }
    HAL_Delay(5U);

    if (!SensorI2C_Read(AHT20_ADDR, regs, 3U)) {
        return 1;
    }
    HAL_Delay(10U);

    cmd[0] = (uint8_t)(0xB0U | addr);
    cmd[1] = regs[1];
    cmd[2] = regs[2];
    if (!SensorI2C_Write(AHT20_ADDR, cmd, 3U)) {
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

int SensorAHT20_Init(uint8_t *status)
{
    uint8_t s = 0U;
    uint8_t init_cmd[3] = {AHT20_CMD_INIT, 0x08U, 0x00U};

    HAL_Delay(500U);
    if (SensorAHT20_GetStatus(&s) != 0) {
        return 1;
    }

    if ((s & 0x18U) != 0x18U) {
        if (aht20_reset_regs() != 0) {
            return 1;
        }
        HAL_Delay(10U);
        if (SensorAHT20_GetStatus(&s) != 0) {
            return 1;
        }
        if ((s & 0x18U) != 0x18U) {
            if (!SensorI2C_Write(AHT20_ADDR, init_cmd, 3U)) {
                return 1;
            }
            HAL_Delay(20U);
            if (SensorAHT20_GetStatus(&s) != 0) {
                return 1;
            }
        }
    }

    if (status != 0) {
        *status = s;
    }
    return 0;
}

int SensorAHT20_Trigger(void)
{
    uint8_t cmd[3] = {AHT20_CMD_MEASURE, 0x33U, 0x00U};

    if (!SensorI2C_Write(AHT20_ADDR, cmd, 3U)) {
        return 1;
    }
    return 0;
}

int SensorAHT20_ReadAfterTrigger(app_aht20_diag_t *out)
{
    uint8_t buf[7];
    uint8_t crc;

    if (out == 0) {
        return 2;
    }

    if (!SensorI2C_Read(AHT20_ADDR, buf, 7U)) {
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

int SensorAHT20_ReadBlocking(app_aht20_diag_t *out)
{
    uint8_t status = 0U;
    int st;

    if (out == 0) {
        return 2;
    }

    st = SensorAHT20_Trigger();
    if (st != 0) {
        return st;
    }
    HAL_Delay(85U);

    st = SensorAHT20_GetStatus(&status);
    if (st != 0) {
        return st;
    }
    if ((status & 0x80U) != 0U) {
        out->status = status;
        out->crc_ok = 0U;
        return 4;
    }

    return SensorAHT20_ReadAfterTrigger(out);
}

int SensorAHT20_Reset(void)
{
    if (aht20_reset_regs() != 0) {
        return 1;
    }
    HAL_Delay(10U);
    return 0;
}

int SensorAHT20_SetReg(const uint8_t *buf, uint8_t len)
{
    if (buf == 0 || len == 0U || len > 7U) {
        return 2;
    }
    if (!SensorI2C_Write(AHT20_ADDR, buf, len)) {
        return 1;
    }
    return 0;
}

int SensorAHT20_GetReg(uint8_t *buf, uint8_t len)
{
    if (buf == 0 || len == 0U || len > 5U) {
        return 2;
    }
    if (!SensorI2C_Read(AHT20_ADDR, buf, len)) {
        return 1;
    }
    return 0;
}
