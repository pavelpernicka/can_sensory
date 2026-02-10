#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_conf.h"

#include "app_config.h"
#include "app_can.h"
#include "calibration.h"
#include "sensors.h"
#include "events.h"

#include <string.h>

CAN_HandleTypeDef hcan1;
I2C_HandleTypeDef hi2c1;

static app_mag_data_t g_mag;
static app_acc_data_t g_acc;

typedef struct {
    uint16_t interval_ms;
    uint8_t enabled;
    uint32_t next_tx_ms;
} stream_cfg_t;

static stream_cfg_t g_stream[5];

static uint32_t g_next_mag_sample_ms;
static uint32_t g_next_acc_sample_ms;
static uint32_t g_last_mag_data_ms;

static uint8_t g_enter_bootloader_req;
static uint8_t g_led_pulse_active;
static uint32_t g_led_pulse_deadline_ms;

typedef struct {
    uint32_t magic;
    uint32_t size;
    uint32_t crc32;
    uint32_t reserved;
} app_bl_meta_t;

static void SystemClock_Config(void);
static void Error_Handler(void);
static void App_SetBootloaderStayMagic(void);
static void App_UpdateCalibStreamConfig(void);
static void App_UpdateCalibHmcConfig(void);
static void App_LoadStreamConfigFromCalib(uint32_t now_ms);
static uint8_t App_LoadDeviceIdFromBootMeta(void);

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void HardFault_Handler(void)
{
    App_SetBootloaderStayMagic();
    NVIC_SystemReset();
    while (1) {
    }
}

static int time_due(uint32_t now, uint32_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

static void schedule_next(uint32_t *deadline, uint32_t interval_ms, uint32_t now)
{
    if (interval_ms == 0U) {
        *deadline = now + 1U;
        return;
    }
    *deadline += interval_ms;
    if ((int32_t)(now - *deadline) > (int32_t)interval_ms) {
        *deadline = now + interval_ms;
    }
}

static void Led_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio.Pin = APP_LED_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(APP_LED_GPIO_PORT, &gpio);
}

static void Led_Set(uint8_t on)
{
    GPIO_PinState st = on ? GPIO_PIN_SET : GPIO_PIN_RESET;
#if APP_LED_ACTIVE_LOW
    st = on ? GPIO_PIN_RESET : GPIO_PIN_SET;
#endif
    HAL_GPIO_WritePin(APP_LED_GPIO_PORT, APP_LED_PIN, st);
}

static void Led_BlinkOnce(void)
{
    Led_Set(1U);
    g_led_pulse_active = 1U;
    g_led_pulse_deadline_ms = HAL_GetTick() + 40U;
}

static void Led_Service(uint32_t now_ms)
{
    if (g_led_pulse_active && (int32_t)(now_ms - g_led_pulse_deadline_ms) >= 0) {
        g_led_pulse_active = 0U;
        Led_Set(0U);
    }
}

static void CanStandby_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio.Pin = APP_CAN_S_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(APP_CAN_S_GPIO_PORT, &gpio);

    HAL_GPIO_WritePin(APP_CAN_S_GPIO_PORT, APP_CAN_S_PIN, APP_CAN_S_NORMAL_LEVEL);
}

static void App_SendPong(void)
{
    uint8_t frame[8] = {'P', 'O', 'N', 'G', 0, 0, 0, 0};
    frame[4] = APP_CAN_GetDeviceId();
    frame[5] = APP_PROTO_VERSION;
    frame[6] = 0x5A;
    APP_CAN_SendFrame(frame, 8);
}

static uint8_t Sensors_Bitfield(void)
{
    const app_sensor_status_t *st = Sensors_GetStatus();
    uint8_t bits = 0U;

    if (st->hmc_present) bits |= 1U << 0;
    if (st->lis_present) bits |= 1U << 1;
    if (st->aht_present) bits |= 1U << 2;
    return bits;
}

static uint8_t Streams_Bitfield(void)
{
    uint8_t bits = 0U;

    if (g_stream[APP_STREAM_MAG].enabled) bits |= 1U << 0;
    if (g_stream[APP_STREAM_ACC].enabled) bits |= 1U << 1;
    if (g_stream[APP_STREAM_ENV].enabled) bits |= 1U << 2;
    if (g_stream[APP_STREAM_EVENT].enabled) bits |= 1U << 3;
    return bits;
}

static uint8_t App_StreamEnableMask(void)
{
    uint8_t mask = 0U;
    if (g_stream[APP_STREAM_MAG].enabled) mask |= 1U << 0;
    if (g_stream[APP_STREAM_ACC].enabled) mask |= 1U << 1;
    if (g_stream[APP_STREAM_ENV].enabled) mask |= 1U << 2;
    if (g_stream[APP_STREAM_EVENT].enabled) mask |= 1U << 3;
    return mask;
}

static void App_SendStartup(void)
{
    uint8_t frame[8] = {0};

    frame[0] = 0x00;
    frame[1] = APP_FRAME_STARTUP;
    frame[2] = APP_CAN_GetDeviceId();
    frame[3] = APP_PROTO_VERSION;
    frame[4] = Sensors_Bitfield();
    frame[5] = Streams_Bitfield();
    frame[6] = (uint8_t)(RCC->CSR & 0xFFU);
    frame[7] = 0;
    APP_CAN_SendFrame(frame, 8);
}

static void App_SendStatusFrame(void)
{
    uint8_t frame[8] = {0};

    frame[0] = 0x00;
    frame[1] = APP_FRAME_STATUS;
    frame[2] = Sensors_Bitfield();
    frame[3] = Streams_Bitfield();
    frame[4] = (uint8_t)(g_stream[APP_STREAM_MAG].interval_ms & 0xFFU);
    frame[5] = (uint8_t)(g_stream[APP_STREAM_ACC].interval_ms & 0xFFU);
    frame[6] = (uint8_t)(g_stream[APP_STREAM_ENV].interval_ms & 0xFFU);
    frame[7] = (uint8_t)(g_stream[APP_STREAM_EVENT].interval_ms & 0xFFU);
    APP_CAN_SendFrame(frame, 8);
}

static void App_SendInterval(uint8_t sid)
{
    uint8_t frame[8] = {0};

    frame[0] = 0x00;
    frame[1] = APP_FRAME_INTERVAL;
    frame[2] = sid;
    frame[3] = g_stream[sid].enabled;
    frame[4] = (uint8_t)(g_stream[sid].interval_ms & 0xFFU);
    frame[5] = (uint8_t)(g_stream[sid].interval_ms >> 8);
    frame[6] = APP_CAN_GetDeviceId();
    frame[7] = APP_PROTO_VERSION;

    APP_CAN_SendFrame(frame, 8);
}

static void App_SendMag(void)
{
    uint8_t frame[8] = {0};

    frame[0] = 0x00;
    frame[1] = APP_FRAME_MAG;
    frame[2] = (uint8_t)(g_mag.x & 0xFF);
    frame[3] = (uint8_t)((uint16_t)g_mag.x >> 8);
    frame[4] = (uint8_t)(g_mag.y & 0xFF);
    frame[5] = (uint8_t)((uint16_t)g_mag.y >> 8);
    frame[6] = (uint8_t)(g_mag.z & 0xFF);
    frame[7] = (uint8_t)((uint16_t)g_mag.z >> 8);

    APP_CAN_SendFrame(frame, 8);
}

static void App_SendAcc(void)
{
    uint8_t frame[8] = {0};

    frame[0] = 0x00;
    frame[1] = APP_FRAME_ACC;
    frame[2] = (uint8_t)(g_acc.x & 0xFF);
    frame[3] = (uint8_t)((uint16_t)g_acc.x >> 8);
    frame[4] = (uint8_t)(g_acc.y & 0xFF);
    frame[5] = (uint8_t)((uint16_t)g_acc.y >> 8);
    frame[6] = (uint8_t)(g_acc.z & 0xFF);
    frame[7] = (uint8_t)((uint16_t)g_acc.z >> 8);

    APP_CAN_SendFrame(frame, 8);
}

static void App_SendEnv(void)
{
    const app_env_data_t *env = Sensors_GetEnv();
    uint8_t frame[8] = {0};

    if (!env->valid) {
        return;
    }

    frame[0] = 0x00;
    frame[1] = APP_FRAME_ENV;
    frame[2] = (uint8_t)(env->temp_centi_c & 0xFF);
    frame[3] = (uint8_t)((uint16_t)env->temp_centi_c >> 8);
    frame[4] = (uint8_t)(env->rh_centi_pct & 0xFF);
    frame[5] = (uint8_t)(env->rh_centi_pct >> 8);
    frame[6] = env->valid;
    frame[7] = 0;

    APP_CAN_SendFrame(frame, 8);
}

static app_status_t App_MapSensorError(int err)
{
    switch (err) {
    case 0:
        return APP_STATUS_OK;
    case 2:
        return APP_STATUS_ERR_RANGE;
    case 3:
        return APP_STATUS_ERR_SENSOR;
    case 4:
        return APP_STATUS_ERR_STATE;
    case 5:
        return APP_STATUS_ERR_SENSOR;
    default:
        return APP_STATUS_ERR_GENERIC;
    }
}

static void App_SendAht20Meas(const app_aht20_diag_t *diag)
{
    uint8_t frame[8] = {0};

    frame[0] = 0x00;
    frame[1] = APP_FRAME_AHT20_MEAS;
    frame[2] = (uint8_t)(diag->temp_centi_c & 0xFF);
    frame[3] = (uint8_t)(((uint16_t)diag->temp_centi_c >> 8) & 0xFFU);
    frame[4] = (uint8_t)(diag->rh_centi_pct & 0xFFU);
    frame[5] = (uint8_t)((diag->rh_centi_pct >> 8) & 0xFFU);
    frame[6] = diag->status;
    frame[7] = diag->crc_ok;
    APP_CAN_SendFrame(frame, 8);
}

static void App_SendAht20Raw(const app_aht20_diag_t *diag)
{
    uint8_t frame[8] = {0};

    frame[0] = 0x00;
    frame[1] = APP_FRAME_AHT20_RAW;
    frame[2] = (uint8_t)(diag->raw_hum & 0xFFU);
    frame[3] = (uint8_t)((diag->raw_hum >> 8) & 0xFFU);
    frame[4] = (uint8_t)((diag->raw_hum >> 16) & 0x0FU);
    frame[5] = (uint8_t)(diag->raw_temp & 0xFFU);
    frame[6] = (uint8_t)((diag->raw_temp >> 8) & 0xFFU);
    frame[7] = (uint8_t)((diag->raw_temp >> 16) & 0x0FU);
    APP_CAN_SendFrame(frame, 8);
}

static void App_SendAht20Status(uint8_t status, uint8_t present, uint8_t valid, uint8_t crc_ok)
{
    uint8_t frame[8] = {0};

    frame[0] = 0x00;
    frame[1] = APP_FRAME_AHT20_STATUS;
    frame[2] = status;
    frame[3] = present;
    frame[4] = valid;
    frame[5] = crc_ok;
    frame[6] = 0;
    frame[7] = 0;
    APP_CAN_SendFrame(frame, 8);
}

static void App_SendAht20Reg(const uint8_t *buf, uint8_t len)
{
    uint8_t frame[8] = {0};

    if (len > 5U) {
        len = 5U;
    }

    frame[0] = 0x00;
    frame[1] = APP_FRAME_AHT20_REG;
    frame[2] = len;
    for (uint8_t i = 0; i < len; ++i) {
        frame[3U + i] = buf[i];
    }
    APP_CAN_SendFrame(frame, 8);
}

static void App_SendHmcConfig(void)
{
    uint8_t frame[8] = {0};
    app_hmc_cfg_t cfg;

    Sensors_HMC_GetConfig(&cfg);

    frame[0] = 0x00;
    frame[1] = APP_FRAME_HMC_CFG;
    frame[2] = cfg.range;
    frame[3] = cfg.data_rate;
    frame[4] = cfg.samples;
    frame[5] = cfg.mode;
    frame[6] = (uint8_t)(cfg.mg_per_digit_centi & 0xFFU);
    frame[7] = (uint8_t)(cfg.mg_per_digit_centi >> 8);
    APP_CAN_SendFrame(frame, 8);
}

static app_status_t App_MapCalibError(int err)
{
    switch (err) {
    case 0:
        return APP_STATUS_OK;
    case 1:
    case 2:
    case 3:
    case 4:
        return APP_STATUS_ERR_RANGE;
    default:
        return APP_STATUS_ERR_GENERIC;
    }
}

static void App_ApplyCalibrationRuntime(void)
{
    const app_calibration_t *cal = Calib_Get();
    Sensors_ApplyCalibration(cal);
    Events_ApplyCalibration(cal);
}

static void App_SendCalibValue(uint8_t field)
{
    uint8_t frame[8] = {0};
    int16_t value = 0;

    if (Calib_GetField(field, &value) != 0) {
        return;
    }

    frame[0] = 0x00;
    frame[1] = APP_FRAME_CALIB_VALUE;
    frame[2] = field;
    frame[3] = (uint8_t)(value & 0xFFU);
    frame[4] = (uint8_t)(((uint16_t)value >> 8) & 0xFFU);
    frame[5] = 0;
    frame[6] = APP_CAN_GetDeviceId();
    frame[7] = APP_PROTO_VERSION;
    APP_CAN_SendFrame(frame, 8);
}

static void App_SendCalibInfo(uint8_t op, uint8_t result)
{
    uint8_t frame[8] = {0};

    frame[0] = 0x00;
    frame[1] = APP_FRAME_CALIB_INFO;
    frame[2] = op;
    frame[3] = result;
    frame[4] = APP_CAN_GetDeviceId();
    frame[5] = APP_PROTO_VERSION;
    frame[6] = 0;
    frame[7] = 0;
    APP_CAN_SendFrame(frame, 8);
}

static void App_SendCalibAll(void)
{
    for (uint8_t field = APP_CAL_FIELD_FIRST; field <= APP_CAL_FIELD_LAST; ++field) {
        App_SendCalibValue(field);
    }
}

static void App_SendEvent(const app_event_t *ev)
{
    uint8_t frame[8] = {0};

    frame[0] = 0x00;
    frame[1] = APP_FRAME_EVENT;
    frame[2] = ev->type;
    frame[3] = ev->p0;
    frame[4] = ev->p1;
    frame[5] = ev->p2;
    frame[6] = (uint8_t)(ev->p3 & 0xFFU);
    frame[7] = (uint8_t)(ev->p3 >> 8);

    APP_CAN_SendFrame(frame, 8);
}

static void App_SendEventState(void)
{
    uint8_t frame[8] = {0};
    uint8_t sector = 0;
    uint8_t elevation = 0;

    Events_GetSectorState(&sector, &elevation);

    frame[0] = 0x00;
    frame[1] = APP_FRAME_EVENT_STATE;
    frame[2] = sector;
    frame[3] = elevation;
    frame[4] = 0;
    frame[5] = 0;
    frame[6] = 0;
    frame[7] = 0;

    APP_CAN_SendFrame(frame, 8);
}

static void App_RequestBootloader(void)
{
    App_SetBootloaderStayMagic();
    HAL_Delay(5);
    NVIC_SystemReset();
}

static void App_SetBootloaderStayMagic(void)
{
    volatile uint32_t *magic = (volatile uint32_t *)APP_BL_STAY_MAGIC_ADDR;
    *magic = APP_BL_STAY_MAGIC_VALUE;
    __DSB();
    __ISB();
}

static void App_InitStreamDefaults(uint32_t now_ms)
{
    memset(g_stream, 0, sizeof(g_stream));

    g_stream[APP_STREAM_MAG].interval_ms = APP_TX_INTERVAL_MAG_DEFAULT_MS;
    g_stream[APP_STREAM_ACC].interval_ms = APP_TX_INTERVAL_ACC_DEFAULT_MS;
    g_stream[APP_STREAM_ENV].interval_ms = APP_TX_INTERVAL_ENV_DEFAULT_MS;
    g_stream[APP_STREAM_EVENT].interval_ms = APP_TX_INTERVAL_EVT_DEFAULT_MS;

    g_stream[APP_STREAM_MAG].enabled = 1U;
    g_stream[APP_STREAM_ACC].enabled = 1U;
    g_stream[APP_STREAM_ENV].enabled = 1U;
    g_stream[APP_STREAM_EVENT].enabled = 1U;

    for (uint8_t i = 1; i <= 4; ++i) {
        g_stream[i].next_tx_ms = now_ms + g_stream[i].interval_ms;
    }
}

static void App_UpdateCalibStreamConfig(void)
{
    Calib_SetStreamConfig(
        g_stream[APP_STREAM_MAG].interval_ms,
        g_stream[APP_STREAM_ACC].interval_ms,
        g_stream[APP_STREAM_ENV].interval_ms,
        g_stream[APP_STREAM_EVENT].interval_ms,
        App_StreamEnableMask()
    );
}

static void App_UpdateCalibHmcConfig(void)
{
    app_hmc_cfg_t cfg;

    Sensors_HMC_GetConfig(&cfg);
    Calib_SetHmcConfig(cfg.range, cfg.data_rate, cfg.samples, cfg.mode);
}

static void App_LoadStreamConfigFromCalib(uint32_t now_ms)
{
    uint16_t mag_ms = APP_TX_INTERVAL_MAG_DEFAULT_MS;
    uint16_t acc_ms = APP_TX_INTERVAL_ACC_DEFAULT_MS;
    uint16_t env_ms = APP_TX_INTERVAL_ENV_DEFAULT_MS;
    uint16_t evt_ms = APP_TX_INTERVAL_EVT_DEFAULT_MS;
    uint8_t enable_mask = 0x0FU;

    Calib_GetStreamConfig(&mag_ms, &acc_ms, &env_ms, &evt_ms, &enable_mask);

    g_stream[APP_STREAM_MAG].interval_ms = mag_ms;
    g_stream[APP_STREAM_ACC].interval_ms = acc_ms;
    g_stream[APP_STREAM_ENV].interval_ms = env_ms;
    g_stream[APP_STREAM_EVENT].interval_ms = evt_ms;

    g_stream[APP_STREAM_MAG].enabled = (enable_mask & (1U << 0)) ? 1U : 0U;
    g_stream[APP_STREAM_ACC].enabled = (enable_mask & (1U << 1)) ? 1U : 0U;
    g_stream[APP_STREAM_ENV].enabled = (enable_mask & (1U << 2)) ? 1U : 0U;
    g_stream[APP_STREAM_EVENT].enabled = (enable_mask & (1U << 3)) ? 1U : 0U;

    for (uint8_t i = APP_STREAM_MAG; i <= APP_STREAM_EVENT; ++i) {
        g_stream[i].next_tx_ms = now_ms + g_stream[i].interval_ms;
    }
}

static uint8_t App_LoadDeviceIdFromBootMeta(void)
{
    const app_bl_meta_t *meta = (const app_bl_meta_t *)APP_BL_META_ADDR;
    uint32_t reserved;
    uint8_t id;

    if (meta->magic != APP_BL_META_MAGIC) {
        return APP_DEVICE_ID;
    }

    reserved = meta->reserved;
    if ((reserved & APP_META_RESERVED_DEVICE_ID_TAG_MASK) != APP_META_RESERVED_DEVICE_ID_TAG) {
        return APP_DEVICE_ID;
    }

    id = (uint8_t)(reserved & APP_META_RESERVED_DEVICE_ID_MASK);
    if (id > APP_DEVICE_ID_MAX) {
        return APP_DEVICE_ID;
    }

    return id;
}

static void App_HandleCommand(const uint8_t *data, uint8_t len)
{
    uint8_t sid;
    uint16_t interval;
    int st;
    app_aht20_diag_t diag;
    uint8_t aht_status = 0U;
    uint8_t reg_buf[5] = {0};
    int16_t val16 = 0;
    int16_t ex = 0;
    int16_t ey = 0;
    int16_t ez = 0;
    const app_sensor_status_t *sensor_st = Sensors_GetStatus();
    const app_env_data_t *env = Sensors_GetEnv();

    if (len == 0U) {
        return;
    }

    switch ((app_cmd_t)data[0]) {
    case APP_CMD_PING:
        APP_CAN_SendStatus(APP_STATUS_OK, 0x01);
        App_SendPong();
        break;

    case APP_CMD_ENTER_BOOTLOADER:
        APP_CAN_SendStatus(APP_STATUS_OK, 0x40);
        g_enter_bootloader_req = 1U;
        break;

    case APP_CMD_HMC_SET_CFG:
        if (len < 5U) {
            APP_CAN_SendStatus(APP_STATUS_ERR_RANGE, 0x6E);
            break;
        }
        st = Sensors_HMC_SetConfig(data[1], data[2], data[3], data[4]);
        if (st != 0) {
            APP_CAN_SendStatus(App_MapSensorError(st), 0x6E);
            break;
        }
        Calib_SetHmcConfig(data[1], data[2], data[3], data[4]);
        APP_CAN_SendStatus(APP_STATUS_OK, 0x6E);
        App_SendHmcConfig();
        break;

    case APP_CMD_HMC_GET_CFG:
        APP_CAN_SendStatus(APP_STATUS_OK, 0x6F);
        App_SendHmcConfig();
        break;

    case APP_CMD_SET_INTERVAL:
        if (len < 4U) {
            APP_CAN_SendStatus(APP_STATUS_ERR_RANGE, 0x70);
            break;
        }
        sid = data[1];
        if (sid < APP_STREAM_MAG || sid > APP_STREAM_EVENT) {
            APP_CAN_SendStatus(APP_STATUS_ERR_RANGE, sid);
            break;
        }
        interval = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
        if (interval > 60000U) {
            APP_CAN_SendStatus(APP_STATUS_ERR_RANGE, sid);
            break;
        }
        g_stream[sid].interval_ms = interval;
        g_stream[sid].next_tx_ms = HAL_GetTick() + interval;
        App_UpdateCalibStreamConfig();
        APP_CAN_SendStatus(APP_STATUS_OK, sid);
        App_SendInterval(sid);
        break;

    case APP_CMD_GET_INTERVAL:
        if (len >= 2U) {
            sid = data[1];
            if (sid == 0U) {
                for (sid = APP_STREAM_MAG; sid <= APP_STREAM_EVENT; ++sid) {
                    App_SendInterval(sid);
                }
            } else if (sid >= APP_STREAM_MAG && sid <= APP_STREAM_EVENT) {
                App_SendInterval(sid);
            } else {
                APP_CAN_SendStatus(APP_STATUS_ERR_RANGE, sid);
            }
        } else {
            for (sid = APP_STREAM_MAG; sid <= APP_STREAM_EVENT; ++sid) {
                App_SendInterval(sid);
            }
        }
        break;

    case APP_CMD_SET_STREAM_ENABLE:
        if (len < 3U) {
            APP_CAN_SendStatus(APP_STATUS_ERR_RANGE, 0x72);
            break;
        }
        sid = data[1];
        if (sid < APP_STREAM_MAG || sid > APP_STREAM_EVENT) {
            APP_CAN_SendStatus(APP_STATUS_ERR_RANGE, sid);
            break;
        }
        g_stream[sid].enabled = data[2] ? 1U : 0U;
        App_UpdateCalibStreamConfig();
        APP_CAN_SendStatus(APP_STATUS_OK, sid);
        App_SendInterval(sid);
        break;

    case APP_CMD_GET_STATUS:
        APP_CAN_SendStatus(APP_STATUS_OK, 0x73);
        App_SendStatusFrame();
        break;

    case APP_CMD_AHT20_READ:
        st = Sensors_AHT20_Read(&diag);
        if (st != 0) {
            APP_CAN_SendStatus(App_MapSensorError(st), (uint8_t)st);
            break;
        }
        APP_CAN_SendStatus(APP_STATUS_OK, 0x74);
        App_SendAht20Meas(&diag);
        App_SendAht20Raw(&diag);
        break;

    case APP_CMD_AHT20_GET_STATUS:
        st = Sensors_AHT20_GetStatus(&aht_status);
        if (st != 0) {
            APP_CAN_SendStatus(App_MapSensorError(st), (uint8_t)st);
            break;
        }
        Sensors_AHT20_GetLast(&diag);
        APP_CAN_SendStatus(APP_STATUS_OK, 0x75);
        App_SendAht20Status(aht_status, sensor_st->aht_present, env->valid, diag.crc_ok);
        break;

    case APP_CMD_AHT20_RESET:
        st = Sensors_AHT20_Reset();
        if (st != 0) {
            APP_CAN_SendStatus(App_MapSensorError(st), (uint8_t)st);
            break;
        }
        st = Sensors_AHT20_GetStatus(&aht_status);
        if (st != 0) {
            APP_CAN_SendStatus(App_MapSensorError(st), (uint8_t)st);
            break;
        }
        Sensors_AHT20_GetLast(&diag);
        APP_CAN_SendStatus(APP_STATUS_OK, 0x76);
        App_SendAht20Status(aht_status, sensor_st->aht_present, env->valid, diag.crc_ok);
        break;

    case APP_CMD_AHT20_SET_REG:
        if (len < 2U || len > 6U) {
            APP_CAN_SendStatus(APP_STATUS_ERR_RANGE, 0x77);
            break;
        }
        st = Sensors_AHT20_SetReg(&data[1], (uint8_t)(len - 1U));
        if (st != 0) {
            APP_CAN_SendStatus(App_MapSensorError(st), (uint8_t)st);
            break;
        }
        APP_CAN_SendStatus(APP_STATUS_OK, 0x77);
        App_SendAht20Reg(&data[1], (uint8_t)(len - 1U));
        break;

    case APP_CMD_AHT20_GET_REG:
        if (len < 2U || data[1] == 0U || data[1] > 5U) {
            APP_CAN_SendStatus(APP_STATUS_ERR_RANGE, 0x78);
            break;
        }
        st = Sensors_AHT20_GetReg(reg_buf, data[1]);
        if (st != 0) {
            APP_CAN_SendStatus(App_MapSensorError(st), (uint8_t)st);
            break;
        }
        APP_CAN_SendStatus(APP_STATUS_OK, 0x78);
        App_SendAht20Reg(reg_buf, data[1]);
        break;

    case APP_CMD_CALIB_GET:
        sid = (len >= 2U) ? data[1] : 0U;
        if (sid == 0U) {
            APP_CAN_SendStatus(APP_STATUS_OK, 0x79);
            App_SendCalibAll();
            break;
        }
        if (sid < APP_CAL_FIELD_FIRST || sid > APP_CAL_FIELD_LAST) {
            APP_CAN_SendStatus(APP_STATUS_ERR_RANGE, sid);
            break;
        }
        APP_CAN_SendStatus(APP_STATUS_OK, sid);
        App_SendCalibValue(sid);
        break;

    case APP_CMD_CALIB_SET:
        if (len < 4U) {
            APP_CAN_SendStatus(APP_STATUS_ERR_RANGE, 0x7A);
            break;
        }
        sid = data[1];
        val16 = (int16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8));
        st = Calib_SetField(sid, val16);
        if (st != 0) {
            APP_CAN_SendStatus(App_MapCalibError(st), sid);
            break;
        }
        App_ApplyCalibrationRuntime();
        APP_CAN_SendStatus(APP_STATUS_OK, sid);
        App_SendCalibValue(sid);
        break;

    case APP_CMD_CALIB_SAVE:
        App_UpdateCalibStreamConfig();
        App_UpdateCalibHmcConfig();
        st = Calib_SaveToFlash();
        if (st != 0) {
            APP_CAN_SendStatus(App_MapCalibError(st), (uint8_t)st);
            break;
        }
        APP_CAN_SendStatus(APP_STATUS_OK, 0x7B);
        App_SendCalibInfo(0x7B, 0U);
        break;

    case APP_CMD_CALIB_LOAD:
        st = Calib_LoadFromFlash();
        if (st != 0) {
            APP_CAN_SendStatus(App_MapCalibError(st), (uint8_t)st);
            break;
        }
        App_ApplyCalibrationRuntime();
        App_LoadStreamConfigFromCalib(HAL_GetTick());
        APP_CAN_SendStatus(APP_STATUS_OK, 0x7C);
        App_SendCalibInfo(0x7C, 0U);
        App_SendCalibAll();
        for (sid = APP_STREAM_MAG; sid <= APP_STREAM_EVENT; ++sid) {
            App_SendInterval(sid);
        }
        App_SendHmcConfig();
        break;

    case APP_CMD_CALIB_RESET:
        Calib_ResetToDefaults();
        App_ApplyCalibrationRuntime();
        App_LoadStreamConfigFromCalib(HAL_GetTick());
        APP_CAN_SendStatus(APP_STATUS_OK, 0x7D);
        App_SendCalibInfo(0x7D, 0U);
        App_SendCalibAll();
        for (sid = APP_STREAM_MAG; sid <= APP_STREAM_EVENT; ++sid) {
            App_SendInterval(sid);
        }
        App_SendHmcConfig();
        break;

    case APP_CMD_CALIB_CAPTURE_EARTH:
        if (!Sensors_CaptureEarthField(&ex, &ey, &ez)) {
            APP_CAN_SendStatus(APP_STATUS_ERR_SENSOR, 0x7E);
            break;
        }
        Calib_SetEarth(ex, ey, ez, 1U);
        App_ApplyCalibrationRuntime();
        APP_CAN_SendStatus(APP_STATUS_OK, 0x7E);
        App_SendCalibInfo(0x7E, 0U);
        App_SendCalibValue(APP_CAL_FIELD_EARTH_X);
        App_SendCalibValue(APP_CAL_FIELD_EARTH_Y);
        App_SendCalibValue(APP_CAL_FIELD_EARTH_Z);
        App_SendCalibValue(APP_CAL_FIELD_EARTH_VALID);
        break;

    default:
        APP_CAN_SendStatus(APP_STATUS_ERR_GENERIC, 0xFF);
        break;
    }
}

int main(void)
{
    uint8_t rx_data[8];
    uint8_t rx_len;
    app_event_t ev;
    uint32_t now;

    HAL_Init();
    SystemClock_Config();
    Led_Init();
    Led_Set(0);
    g_led_pulse_active = 0U;
    g_led_pulse_deadline_ms = 0U;
    CanStandby_Init();

    Calib_Init();

    APP_CAN_SetDeviceId(App_LoadDeviceIdFromBootMeta());
    APP_CAN_Init(&hcan1);
    App_SendStartup();

    Sensors_Init(&hi2c1);
    Events_Init();
    App_ApplyCalibrationRuntime();

    now = HAL_GetTick();
    App_InitStreamDefaults(now);
    App_LoadStreamConfigFromCalib(now);

    g_next_mag_sample_ms = now + APP_MAG_SAMPLE_PERIOD_MS;
    g_next_acc_sample_ms = now + APP_ACC_SAMPLE_PERIOD_MS;
    g_last_mag_data_ms = now;

    App_SendStartup();

    while (1) {
        now = HAL_GetTick();
        Led_Service(now);

        while (APP_CAN_TryRecv(rx_data, &rx_len)) {
            App_HandleCommand(rx_data, rx_len);
            Led_BlinkOnce();
        }

        if (time_due(now, g_next_mag_sample_ms)) {
            if (Sensors_ReadMag(&g_mag) && g_mag.valid) {
                g_last_mag_data_ms = now;
                /* Event detection runs on fast sampled magnetic vector. */
                Events_ProcessMagSample((float)g_mag.x, (float)g_mag.y, (float)(-g_mag.z), now);
            }
            schedule_next(&g_next_mag_sample_ms, APP_MAG_SAMPLE_PERIOD_MS, now);
        }

        if (time_due(now, g_next_acc_sample_ms)) {
            Sensors_ReadAcc(&g_acc);
            schedule_next(&g_next_acc_sample_ms, APP_ACC_SAMPLE_PERIOD_MS, now);
        }

        Sensors_ServiceEnv(now);

        if ((now - g_last_mag_data_ms) > 10000U) {
            Events_PostNoData(now);
        }

        while (Events_Pop(&ev)) {
            if (g_stream[APP_STREAM_EVENT].enabled) {
                App_SendEvent(&ev);
            }
        }

        if (g_stream[APP_STREAM_MAG].enabled && g_stream[APP_STREAM_MAG].interval_ms > 0U && time_due(now, g_stream[APP_STREAM_MAG].next_tx_ms)) {
            if (g_mag.valid) {
                App_SendMag();
            }
            schedule_next(&g_stream[APP_STREAM_MAG].next_tx_ms, g_stream[APP_STREAM_MAG].interval_ms, now);
        }

        if (g_stream[APP_STREAM_ACC].enabled && g_stream[APP_STREAM_ACC].interval_ms > 0U && time_due(now, g_stream[APP_STREAM_ACC].next_tx_ms)) {
            if (g_acc.valid) {
                App_SendAcc();
            }
            schedule_next(&g_stream[APP_STREAM_ACC].next_tx_ms, g_stream[APP_STREAM_ACC].interval_ms, now);
        }

        if (g_stream[APP_STREAM_ENV].enabled && g_stream[APP_STREAM_ENV].interval_ms > 0U && time_due(now, g_stream[APP_STREAM_ENV].next_tx_ms)) {
            App_SendEnv();
            schedule_next(&g_stream[APP_STREAM_ENV].next_tx_ms, g_stream[APP_STREAM_ENV].interval_ms, now);
        }

        if (g_stream[APP_STREAM_EVENT].enabled && g_stream[APP_STREAM_EVENT].interval_ms > 0U && time_due(now, g_stream[APP_STREAM_EVENT].next_tx_ms)) {
            App_SendEventState();
            schedule_next(&g_stream[APP_STREAM_EVENT].next_tx_ms, g_stream[APP_STREAM_EVENT].interval_ms, now);
        }

        if (g_enter_bootloader_req) {
            App_RequestBootloader();
        }
    }
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        Error_Handler();
    }

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000U);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}

static void Error_Handler(void)
{
    while (1) {
        HAL_GPIO_TogglePin(APP_LED_GPIO_PORT, APP_LED_PIN);
        HAL_Delay(100);
    }
}
