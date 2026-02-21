#pragma once

#include "stm32l4xx_hal.h"
#include <stdint.h>

// Pinout
#define APP_LED_GPIO_PORT             GPIOA
#define APP_LED_PIN                   GPIO_PIN_15
#define APP_LED_ACTIVE_LOW            0

#define APP_CAN_S_GPIO_PORT           GPIOA
#define APP_CAN_S_PIN                 GPIO_PIN_5
#define APP_CAN_S_NORMAL_LEVEL        GPIO_PIN_RESET

#define APP_CAN_INSTANCE              CAN1
#define APP_CAN_BITRATE_PRESCALER     2U
#define APP_CAN_BITRATE_BS1           CAN_BS1_13TQ
#define APP_CAN_BITRATE_BS2           CAN_BS2_2TQ
#define APP_CAN_BITRATE_SJW           CAN_SJW_1TQ

#define APP_I2C_INSTANCE              I2C1
#define APP_I2C_SCL_GPIO_PORT         GPIOB
#define APP_I2C_SCL_PIN               GPIO_PIN_6
#define APP_I2C_SDA_GPIO_PORT         GPIOB
#define APP_I2C_SDA_PIN               GPIO_PIN_7
#define APP_I2C_GPIO_AF               GPIO_AF4_I2C1
#define APP_I2C_TIMING                0x0020098EU
#define APP_I2C_TIMEOUT_MS            50U

#define APP_WS2812_GPIO_PORT          GPIOA
#define APP_WS2812_PIN                GPIO_PIN_7
#define APP_WS2812_STRIP_LEN          16U

// Addressing
#define APP_DEVICE_ID                 0x01U // fallback
#define APP_DEVICE_ID_MAX             0x7FU
#define APP_CAN_BASE_CMD_ID           0x600U
#define APP_CAN_BASE_STATUS_ID        0x580U
#define APP_CAN_CMD_ID                (APP_CAN_BASE_CMD_ID + APP_DEVICE_ID)
#define APP_CAN_STATUS_ID             (APP_CAN_BASE_STATUS_ID + APP_DEVICE_ID)
#define APP_PROTO_VERSION             1U

// Metadata from bootloader
#define APP_BL_META_ADDR                       ((uint32_t)0x0801F800UL)
#define APP_BL_META_MAGIC                      0xB00710ADUL
#define APP_META_RESERVED_DEVICE_ID_TAG        0xA5D10000UL
#define APP_META_RESERVED_DEVICE_ID_TAG_MASK   0xFFFFFF00UL
#define APP_META_RESERVED_DEVICE_ID_MASK       0x000000FFUL

// App-to-bootloader handoff
#define APP_BL_STAY_MAGIC_ADDR        ((uint32_t)0x10000000UL) /* SRAM2 */
#define APP_BL_STAY_MAGIC_VALUE       ((uint32_t)0xB007B007UL)
#define APP_CALIB_FLASH_ADDR          ((uint32_t)0x0801F000UL)
#define APP_CALIB_FLASH_PAGE_SIZE     0x800U

// Sampling defaults
#define APP_HMC_DEFAULT_RANGE         7U   /* 8.1 gauss range */
#define APP_HMC_DEFAULT_DATA_RATE     6U   /* 75 Hz */
#define APP_HMC_DEFAULT_SAMPLES       0U   /* 1 sample */
#define APP_HMC_DEFAULT_MODE          0U   /* continuous */

#define APP_MAG_SAMPLE_PERIOD_MS      10U
#define APP_ACC_SAMPLE_PERIOD_MS      20U
#define APP_ENV_TRIGGER_PERIOD_MS     1000U
#define APP_ENV_CONVERSION_MS         90U

#define APP_TX_INTERVAL_MAG_DEFAULT_MS   200U
#define APP_TX_INTERVAL_ACC_DEFAULT_MS   200U
#define APP_TX_INTERVAL_ENV_DEFAULT_MS   1000U
#define APP_TX_INTERVAL_EVT_DEFAULT_MS   250U

// Protocol
typedef enum {
    APP_STATUS_OK = 0x00,
    APP_STATUS_ERR_GENERIC = 0x01,
    APP_STATUS_ERR_RANGE = 0x02,
    APP_STATUS_ERR_STATE = 0x03,
    APP_STATUS_ERR_SENSOR = 0x04
} app_status_t;

typedef enum {
    APP_CMD_PING = 0x01,
    APP_CMD_ENTER_BOOTLOADER = 0x40,
    APP_CMD_HMC_SET_CFG = 0x6E,
    APP_CMD_HMC_GET_CFG = 0x6F,
    APP_CMD_SET_INTERVAL = 0x70,
    APP_CMD_GET_INTERVAL = 0x71,
    APP_CMD_SET_STREAM_ENABLE = 0x72,
    APP_CMD_GET_STATUS = 0x73,
    APP_CMD_AHT20_READ = 0x74,
    APP_CMD_AHT20_GET_STATUS = 0x75,
    APP_CMD_AHT20_RESET = 0x76,
    APP_CMD_AHT20_SET_REG = 0x77,
    APP_CMD_AHT20_GET_REG = 0x78,
    APP_CMD_WS_SET_POWER = 0x50,
    APP_CMD_WS_SET_BRIGHTNESS = 0x51,
    APP_CMD_WS_SET_COLOR = 0x52,
    APP_CMD_WS_SET_ALL = 0x53,
    APP_CMD_WS_GET_STATE = 0x54,
    APP_CMD_CALIB_GET = 0x79,
    APP_CMD_CALIB_SET = 0x7A,
    APP_CMD_CALIB_SAVE = 0x7B,
    APP_CMD_CALIB_LOAD = 0x7C,
    APP_CMD_CALIB_RESET = 0x7D,
    APP_CMD_CALIB_CAPTURE_EARTH = 0x7E
} app_cmd_t;

typedef enum {
    APP_STREAM_MAG = 1,
    APP_STREAM_ACC = 2,
    APP_STREAM_ENV = 3,
    APP_STREAM_EVENT = 4
} app_stream_id_t;

typedef enum {
    APP_FRAME_PONG = 0x01,
    APP_FRAME_STARTUP = 0x02,
    APP_FRAME_MAG = 0x10,
    APP_FRAME_ACC = 0x11,
    APP_FRAME_ENV = 0x12,
    APP_FRAME_EVENT = 0x20,
    APP_FRAME_INTERVAL = 0x30,
    APP_FRAME_STATUS = 0x31,
    APP_FRAME_EVENT_STATE = 0x32,
    APP_FRAME_AHT20_MEAS = 0x40,
    APP_FRAME_AHT20_RAW = 0x41,
    APP_FRAME_AHT20_STATUS = 0x42,
    APP_FRAME_AHT20_REG = 0x43,
    APP_FRAME_CALIB_VALUE = 0x44,
    APP_FRAME_CALIB_INFO = 0x45,
    APP_FRAME_HMC_CFG = 0x46,
    APP_FRAME_WS_STATE = 0x47
} app_frame_t;

typedef enum {
    APP_EVENT_SECTOR_ACTIVATED = 1,
    APP_EVENT_SECTOR_CHANGED = 2,
    APP_EVENT_INTENSITY_CHANGE = 3,
    APP_EVENT_SECTION_DEACTIVATED = 4,
    APP_EVENT_SESSION_STARTED = 5,
    APP_EVENT_SESSION_ENDED = 6,
    APP_EVENT_PASSING_SECTOR_CHANGE = 7,
    APP_EVENT_POSSIBLE_MECHANICAL_FAILURE = 8,
    APP_EVENT_ERROR_NO_DATA = 9
} app_event_type_t;
