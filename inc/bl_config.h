#pragma once

#include "stm32l4xx_hal.h"
#include <stdint.h>

// Board pinout
#define BL_LED_GPIO_PORT      GPIOA
#define BL_LED_PIN            GPIO_PIN_15
#define BL_LED_ACTIVE_LOW     0

#define BL_CAN_S_GPIO_PORT    GPIOA // CAN at PA11 (RX), PA12 (TX)
#define BL_CAN_S_PIN          GPIO_PIN_5
#define BL_CAN_S_NORMAL_LEVEL GPIO_PIN_RESET

// LED settings
#define BL_START_BLINK_COUNT         3U
#define BL_START_BLINK_DELAY_MS      280U
#define BL_JUMP_BLINK_COUNT          2U
#define BL_JUMP_BLINK_DELAY_MS       220U
#define BL_ACTIVITY_BLINK_COUNT      1U
#define BL_ACTIVITY_BLINK_DELAY_MS   30U
#define BL_PING_BLINK_COUNT          3U
#define BL_PING_BLINK_DELAY_MS       60U

// Bootloader settings
#define BL_AUTORUN_WAIT_MS           3000U
#define BL_FORCE_STAY_IN_BOOTLOADER 0

// Communication
#define BL_DEVICE_ID          0x05
#define BL_DEVICE_ID_MAX      0x7F

#define BL_CAN_BASE_CMD_ID    0x600
#define BL_CAN_BASE_STATUS_ID 0x580

#define BL_CAN_CMD_ID         (BL_CAN_BASE_CMD_ID    + BL_DEVICE_ID)
#define BL_CAN_STATUS_ID      (BL_CAN_BASE_STATUS_ID + BL_DEVICE_ID)
#define BL_PROTO_VERSION      2U

#define BL_STAY_MAGIC_ADDR    ((uint32_t)0x10000000UL) /* SRAM2 */
#define BL_STAY_MAGIC_VALUE   ((uint32_t)0xB007B007UL)

// I2C
#define BL_I2C_INSTANCE             I2C1
#define BL_I2C_SCL_GPIO_PORT        GPIOB
#define BL_I2C_SCL_PIN              GPIO_PIN_6
#define BL_I2C_SDA_GPIO_PORT        GPIOB
#define BL_I2C_SDA_PIN              GPIO_PIN_7
#define BL_I2C_GPIO_AF              GPIO_AF4_I2C1
#define BL_I2C_TIMING               0x0020098EU
#define BL_I2C_TIMEOUT_MS           50U
#define BL_I2C_SCAN_FIRST_ADDR      0x08U
#define BL_I2C_SCAN_LAST_ADDR       0x77U
#define BL_I2C_MAX_TX               48U
#define BL_I2C_MAX_RX               32U

// Flash layout
#define FLASH_START_ADDR      0x08000000UL
#define FLASH_TOTAL_SIZE      0x00020000UL     // 128 kB

// Bootloader: 0x08000000 - 0x08003FFF (16 kB)
#define BL_FLASH_START        FLASH_START_ADDR
#define BL_FLASH_SIZE         (16U * 1024U)
#define BL_FLASH_END          (BL_FLASH_START + BL_FLASH_SIZE)

// Application: 0x08004000 - end of flash region
#define APP_FLASH_START       (FLASH_START_ADDR + BL_FLASH_SIZE)
#define META_FLASH_START      (FLASH_START_ADDR + FLASH_TOTAL_SIZE - FLASH_PAGE_SIZE)
#define APP_FLASH_END         (META_FLASH_START)

#define APP_MAX_SIZE          (APP_FLASH_END - APP_FLASH_START)

#define BL_META_MAGIC         0xB00710ADUL

typedef struct {
    uint32_t magic;
    uint32_t size;
    uint32_t crc32;
    uint32_t reserved;
} bl_meta_t;

#define BL_META_ADDR          (META_FLASH_START)
#define BL_META_RESERVED_DEVICE_ID_TAG       0xA5D10000UL
#define BL_META_RESERVED_DEVICE_ID_TAG_MASK  0xFFFFFF00UL
#define BL_META_RESERVED_DEVICE_ID_MASK      0x000000FFUL
#define BL_META_RESERVED_ENCODE_DEVICE_ID(id) \
    (BL_META_RESERVED_DEVICE_ID_TAG | ((uint32_t)(id) & BL_META_RESERVED_DEVICE_ID_MASK))
#define BL_META_RESERVED_HAS_DEVICE_ID(v) \
    (((uint32_t)(v) & BL_META_RESERVED_DEVICE_ID_TAG_MASK) == BL_META_RESERVED_DEVICE_ID_TAG)
#define BL_META_RESERVED_GET_DEVICE_ID(v) \
    ((uint8_t)((uint32_t)(v) & BL_META_RESERVED_DEVICE_ID_MASK))

// Protocol
typedef enum {
    BL_CMD_PING     = 0x01,  // ping
    BL_CMD_CHECK    = 0x02,  // FW validity check + metadata
    BL_CMD_START    = 0x10,  // Start update - FW size
    BL_CMD_DATA     = 0x20,  // Data chunk
    BL_CMD_END      = 0x30,  // End update - CRC
    BL_CMD_BOOT_APP = 0x40,  // Jump to app
    BL_CMD_BOOT_STATUS = 0x41, // Last error before jump
    BL_CMD_I2C_BUF_CLEAR  = 0x50, // I2C xfer TX buffer reset
    BL_CMD_I2C_BUF_APPEND = 0x51, // append data to TX buffer
    BL_CMD_I2C_XFER       = 0x52, // I2C write/read cmd
    BL_CMD_I2C_SCAN       = 0x53  // i2cdetect
} bl_cmd_t;

typedef enum {
    BL_STATUS_OK          = 0x00,
    BL_STATUS_ERR_GENERIC = 0x01,
    BL_STATUS_ERR_RANGE   = 0x02,
    BL_STATUS_ERR_STATE   = 0x03,
    BL_STATUS_ERR_CRC     = 0x04
} bl_status_t;

typedef enum {
    BL_BOOTERR_NONE        = 0x00,
    BL_BOOTERR_APP_INVALID = 0xE1,
    BL_BOOTERR_VECTOR_EMPTY= 0xE2,
    BL_BOOTERR_STACK_ALIGN = 0xE3,
    BL_BOOTERR_STACK_RANGE = 0xE4,
    BL_BOOTERR_ENTRY_RANGE = 0xE5,
    BL_BOOTERR_RETURNED    = 0xE6
} bl_boot_error_t;
