#include "bl_can.h"

static CAN_HandleTypeDef *g_hcan;
static int stay_in_bl = 0;

uint8_t BL_BootToAppRequest = 0;
static uint8_t bl_last_boot_error = (uint8_t)BL_BOOTERR_NONE;

// Stav updatu
static uint8_t  bl_updating       = 0;
static uint32_t bl_expected_size  = 0;
static uint32_t bl_received_bytes = 0;
static uint32_t bl_crc32_state    = 0;
static uint32_t bl_write_addr     = APP_FLASH_START;
static uint8_t  bl_flash_staging[8];
static uint8_t  bl_flash_staging_len = 0;

static I2C_HandleTypeDef g_hi2c;
static uint8_t bl_i2c_ready = 0;
static uint8_t bl_i2c_tx_buf[BL_I2C_MAX_TX];
static uint8_t bl_i2c_tx_len = 0;
static uint8_t bl_i2c_rx_buf[BL_I2C_MAX_RX];

enum {
    BL_FRAME_CHECK_SUMMARY = 0x20,
    BL_FRAME_CHECK_CRC     = 0x21,
    BL_FRAME_I2C_SCAN      = 0x60,
    BL_FRAME_I2C_RXDATA    = 0x61
};

static void BL_CAN_SendStartupMsg(void);
static HAL_StatusTypeDef BL_FlashPushBytes(const uint8_t *data, uint32_t len);
static HAL_StatusTypeDef BL_FlashFlushTail(void);
static void BL_CAN_SendStatus(bl_status_t st, uint8_t extra);

static void BL_CAN_SendFrame(const uint8_t *data, uint8_t dlc)
{
    CAN_TxHeaderTypeDef tx = {0};
    uint8_t payload[8] = {0};
    uint32_t mbox;
    uint32_t start;

    if (dlc > 8U) {
        dlc = 8U;
    }

    for (uint8_t i = 0; i < dlc; ++i) {
        payload[i] = data[i];
    }

    tx.StdId = BL_CAN_STATUS_ID;
    tx.IDE   = CAN_ID_STD;
    tx.RTR   = CAN_RTR_DATA;
    tx.DLC   = dlc;

    start = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(g_hcan) == 0U) {
        if ((HAL_GetTick() - start) > 20U) {
            return;
        }
    }

    if (HAL_CAN_AddTxMessage(g_hcan, &tx, payload, &mbox) != HAL_OK) {
        start = HAL_GetTick();
        while (HAL_CAN_GetTxMailboxesFreeLevel(g_hcan) == 0U) {
            if ((HAL_GetTick() - start) > 20U) {
                return;
            }
        }
        HAL_CAN_AddTxMessage(g_hcan, &tx, payload, &mbox);
    }
}

static void BL_CAN_SendChunked(uint8_t subtype, const uint8_t *data, uint8_t total_len)
{
    uint8_t frame[8] = {0};
    frame[0] = (uint8_t)BL_STATUS_OK;
    frame[1] = subtype;
    frame[3] = total_len;

    if (total_len == 0U) {
        BL_CAN_SendFrame(frame, 8);
        return;
    }

    for (uint8_t off = 0; off < total_len; off += 4U) {
        uint8_t chunk = (uint8_t)((total_len - off > 4U) ? 4U : (total_len - off));
        frame[2] = off;
        frame[4] = 0;
        frame[5] = 0;
        frame[6] = 0;
        frame[7] = 0;
        for (uint8_t i = 0; i < chunk; ++i) {
            frame[4U + i] = data[off + i];
        }
        BL_CAN_SendFrame(frame, 8);
    }
}

static HAL_StatusTypeDef BL_FlashPushBytes(const uint8_t *data, uint32_t len)
{
    HAL_StatusTypeDef st = HAL_OK;

    while (len > 0U && st == HAL_OK) {
        bl_flash_staging[bl_flash_staging_len++] = *data++;
        len--;

        if (bl_flash_staging_len == 8U) {
            st = Flash_ProgramBytes(bl_write_addr, bl_flash_staging, 8U);
            if (st != HAL_OK) {
                return st;
            }
            bl_write_addr += 8U;
            bl_flash_staging_len = 0U;
        }
    }

    return st;
}

static HAL_StatusTypeDef BL_FlashFlushTail(void)
{
    if (bl_flash_staging_len == 0U) {
        return HAL_OK;
    }

    for (uint8_t i = bl_flash_staging_len; i < 8U; ++i) {
        bl_flash_staging[i] = 0xFFU;
    }

    HAL_StatusTypeDef st = Flash_ProgramBytes(bl_write_addr, bl_flash_staging, 8U);
    if (st == HAL_OK) {
        bl_write_addr += 8U;
        bl_flash_staging_len = 0U;
    }

    return st;
}

static void BL_I2C_Init(void)
{
    g_hi2c.Instance = BL_I2C_INSTANCE;
    g_hi2c.Init.Timing = BL_I2C_TIMING;
    g_hi2c.Init.OwnAddress1 = 0U;
    g_hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    g_hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    g_hi2c.Init.OwnAddress2 = 0U;
    g_hi2c.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    g_hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    g_hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&g_hi2c) != HAL_OK) {
        bl_i2c_ready = 0;
        return;
    }
    if (HAL_I2CEx_ConfigAnalogFilter(&g_hi2c, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
        bl_i2c_ready = 0;
        return;
    }

    bl_i2c_tx_len = 0U;
    bl_i2c_ready = 1;
}

static HAL_StatusTypeDef BL_I2C_DoTransfer(uint8_t addr7, uint8_t rx_len)
{
    HAL_StatusTypeDef st = HAL_OK;
    uint16_t addr = (uint16_t)(addr7 << 1);

    if (bl_i2c_tx_len > 0U) {
        st = HAL_I2C_Master_Transmit(&g_hi2c, addr, bl_i2c_tx_buf, bl_i2c_tx_len, BL_I2C_TIMEOUT_MS);
    }
    if (st == HAL_OK && rx_len > 0U) {
        st = HAL_I2C_Master_Receive(&g_hi2c, addr, bl_i2c_rx_buf, rx_len, BL_I2C_TIMEOUT_MS);
    }

    bl_i2c_tx_len = 0U;
    return st;
}

static void BL_I2C_DoScan(uint8_t first, uint8_t last)
{
    uint8_t found[16] = {0};
    const uint32_t scan_timeout_ms = 3U;

    for (uint8_t addr = first; addr <= last; ++addr) {
        if (HAL_I2C_IsDeviceReady(&g_hi2c, (uint16_t)(addr << 1), 1U, scan_timeout_ms) == HAL_OK) {
            found[addr >> 3] |= (uint8_t)(1U << (addr & 0x7U));
        }
        if (addr == 0x7FU) {
            break;
        }
    }

    BL_CAN_SendChunked(BL_FRAME_I2C_SCAN, found, sizeof(found));
}

/* Inicializace CAN1 na PA11/PA12, 500 kbit/s */
void BL_CAN_Init(CAN_HandleTypeDef *hcan)
{
    g_hcan = hcan;

    g_hcan->Instance = CAN1;
    g_hcan->Init.Prescaler = 2;              // 16 MHz / (2 * (1+13+2)) = 500 kbit/s
    g_hcan->Init.Mode = CAN_MODE_NORMAL;
    g_hcan->Init.SyncJumpWidth = CAN_SJW_1TQ;
    g_hcan->Init.TimeSeg1 = CAN_BS1_13TQ;
    g_hcan->Init.TimeSeg2 = CAN_BS2_2TQ;
    g_hcan->Init.TimeTriggeredMode = DISABLE;
    g_hcan->Init.AutoBusOff = DISABLE;
    g_hcan->Init.AutoWakeUp = DISABLE;
    g_hcan->Init.AutoRetransmission = ENABLE;
    g_hcan->Init.ReceiveFifoLocked = DISABLE;
    g_hcan->Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(g_hcan) != HAL_OK) {
        // když to spadne, necháme to řešit Error_Handler v mainu
    }

    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh     = (BL_CAN_CMD_ID << 5);
    filter.FilterIdLow      = 0x0000;
    filter.FilterMaskIdHigh = 0xFFE0;  // maska na celé 11bit ID
    filter.FilterMaskIdLow  = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;

    HAL_CAN_ConfigFilter(g_hcan, &filter);
    HAL_CAN_Start(g_hcan);

    BL_I2C_Init();
    bl_last_boot_error = (uint8_t)BL_BOOTERR_NONE;
    BL_CAN_SendStartupMsg();
}

int BL_CAN_StayInBootloaderRequested(void)
{
    return stay_in_bl;
}

void BL_CAN_ForceStayInBootloader(void)
{
    stay_in_bl = 1;
}

void BL_CAN_SetLastBootError(uint8_t code)
{
    bl_last_boot_error = code;
}

uint8_t BL_CAN_GetLastBootError(void)
{
    return bl_last_boot_error;
}

void BL_CAN_ReportBootError(uint8_t code)
{
    bl_last_boot_error = code;
    BL_CAN_SendStatus(BL_STATUS_ERR_STATE, code);
}

static void BL_CAN_SendStatus(bl_status_t st, uint8_t extra)
{
    uint8_t data[8] = {0};
    data[0] = (uint8_t)st;
    data[1] = extra;
    BL_CAN_SendFrame(data, 8);
}

static void BL_CAN_SendPingMsg(void)
{
    uint8_t pong[8] = {'P', 'O', 'N', 'G', 0, 0, 0, 0};
    pong[4] = (uint8_t)BL_DEVICE_ID;
    pong[5] = (uint8_t)BL_PROTO_VERSION;
    pong[6] = (uint8_t)stay_in_bl;
    pong[7] = 0xA5;
    BL_CAN_SendFrame(pong, 8);
}

static void BL_CAN_SendStartupMsg(void)
{
    uint8_t msg[8] = {'B', 'L', 'S', 'T', 0, 0, 0, 0};
    uint8_t flags = 0U;

    msg[4] = (uint8_t)BL_DEVICE_ID;
    msg[5] = (uint8_t)BL_PROTO_VERSION;

    if (Flash_IsAppValid(0)) {
        flags |= (1U << 0); // valid app present
    }
    if (bl_i2c_ready) {
        flags |= (1U << 1); // i2c bridge ready
    }
    if (BL_FORCE_STAY_IN_BOOTLOADER) {
        flags |= (1U << 2); // forced stay in bootloader
    }

    msg[6] = flags;
    msg[7] = (uint8_t)(RCC->CSR & 0xFFU); // reset cause bits (low byte)

    BL_CAN_SendFrame(msg, 8);
}

static void BL_CAN_SendCheckInfo(void)
{
    bl_meta_t meta = {0};
    uint8_t valid = (uint8_t)Flash_IsAppValid(&meta);
    uint32_t size = valid ? meta.size : 0U;
    uint32_t crc  = valid ? meta.crc32 : 0U;

    uint8_t frame0[8] = {0};
    frame0[0] = (uint8_t)BL_STATUS_OK;
    frame0[1] = BL_FRAME_CHECK_SUMMARY;
    frame0[2] = valid;
    frame0[3] = bl_updating;
    frame0[4] = (uint8_t)(size);
    frame0[5] = (uint8_t)(size >> 8);
    frame0[6] = (uint8_t)(size >> 16);
    frame0[7] = (uint8_t)(size >> 24);
    BL_CAN_SendFrame(frame0, 8);

    uint8_t frame1[8] = {0};
    frame1[0] = (uint8_t)BL_STATUS_OK;
    frame1[1] = BL_FRAME_CHECK_CRC;
    frame1[2] = (uint8_t)(crc);
    frame1[3] = (uint8_t)(crc >> 8);
    frame1[4] = (uint8_t)(crc >> 16);
    frame1[5] = (uint8_t)(crc >> 24);
    frame1[6] = (uint8_t)BL_DEVICE_ID;
    frame1[7] = (uint8_t)BL_PROTO_VERSION;
    BL_CAN_SendFrame(frame1, 8);
}

static uint32_t BL_HandleCmd(uint8_t *data, uint8_t len)
{
    if (len == 0)
        return BL_CAN_EVENT_NONE;

    bl_cmd_t cmd = (bl_cmd_t)data[0];

    switch (cmd) {
    case BL_CMD_PING:
        // Byte1 == 0x42 stay in bootloader
        if (len > 1 && data[1] == 0x42) {
            stay_in_bl = 1;
        }
        BL_CAN_SendStatus(BL_STATUS_OK, 0x01);
        BL_CAN_SendPingMsg();
        return BL_CAN_EVENT_ACTIVITY | BL_CAN_EVENT_PING;

    case BL_CMD_CHECK:
        BL_CAN_SendCheckInfo();
        return BL_CAN_EVENT_ACTIVITY;

    case BL_CMD_START: {
        if (len < 5) {
            BL_CAN_SendStatus(BL_STATUS_ERR_GENERIC, 0);
            return BL_CAN_EVENT_ACTIVITY;
        }
        uint32_t size =
            ((uint32_t)data[1])       |
            ((uint32_t)data[2] << 8)  |
            ((uint32_t)data[3] << 16) |
            ((uint32_t)data[4] << 24);

        if (size == 0 || size > APP_MAX_SIZE) {
            BL_CAN_SendStatus(BL_STATUS_ERR_RANGE, 0);
            return BL_CAN_EVENT_ACTIVITY;
        }

        if (Flash_EraseAppArea() != HAL_OK) {
            BL_CAN_SendStatus(BL_STATUS_ERR_GENERIC, 1);
            return BL_CAN_EVENT_ACTIVITY;
        }

        bl_updating       = 1;
        bl_expected_size  = size;
        bl_received_bytes = 0;
        bl_write_addr     = APP_FLASH_START;
        bl_flash_staging_len = 0U;
        crc32_reset(&bl_crc32_state);

        BL_CAN_SendStatus(BL_STATUS_OK, 0);
        return BL_CAN_EVENT_ACTIVITY;
    }

    case BL_CMD_DATA: {
        if (!bl_updating) {
            BL_CAN_SendStatus(BL_STATUS_ERR_STATE, 0);
            return BL_CAN_EVENT_ACTIVITY;
        }

        if (bl_received_bytes >= bl_expected_size) {
            BL_CAN_SendStatus(BL_STATUS_ERR_RANGE, 0);
            return BL_CAN_EVENT_ACTIVITY;
        }

        uint32_t remaining = bl_expected_size - bl_received_bytes;
        uint32_t payload_len = (len - 1 <= remaining) ? (len - 1) : remaining;

        if (BL_FlashPushBytes(&data[1], payload_len) != HAL_OK) {
            BL_CAN_SendStatus(BL_STATUS_ERR_GENERIC, 2);
            bl_updating = 0;
            return BL_CAN_EVENT_ACTIVITY;
        }

        crc32_update(&bl_crc32_state, &data[1], payload_len);

        bl_received_bytes += payload_len;

        BL_CAN_SendStatus(BL_STATUS_OK, 0);
        return BL_CAN_EVENT_ACTIVITY;
    }

    case BL_CMD_END: {
        if (!bl_updating) {
            BL_CAN_SendStatus(BL_STATUS_ERR_STATE, 0);
            return BL_CAN_EVENT_ACTIVITY;
        }
        bl_updating = 0;

        if (len < 5) {
            BL_CAN_SendStatus(BL_STATUS_ERR_GENERIC, 0);
            return BL_CAN_EVENT_ACTIVITY;
        }

        uint32_t crc_host =
            ((uint32_t)data[1])       |
            ((uint32_t)data[2] << 8)  |
            ((uint32_t)data[3] << 16) |
            ((uint32_t)data[4] << 24);

        uint32_t crc_dev = crc32_value(&bl_crc32_state);

        if (crc_host != crc_dev || bl_received_bytes != bl_expected_size) {
            BL_CAN_SendStatus(BL_STATUS_ERR_CRC, 0);
            return BL_CAN_EVENT_ACTIVITY;
        }

        if (BL_FlashFlushTail() != HAL_OK) {
            BL_CAN_SendStatus(BL_STATUS_ERR_GENERIC, 2);
            return BL_CAN_EVENT_ACTIVITY;
        }

        bl_meta_t meta = {
            .magic  = BL_META_MAGIC,
            .size   = bl_received_bytes,
            .crc32  = crc_dev,
            .reserved = BL_META_RESERVED_ENCODE_DEVICE_ID(BL_DEVICE_ID)
        };

        if (Flash_WriteMeta(&meta) != HAL_OK) {
            BL_CAN_SendStatus(BL_STATUS_ERR_GENERIC, 3);
            return BL_CAN_EVENT_ACTIVITY;
        }

        BL_CAN_SendStatus(BL_STATUS_OK, 0);
        return BL_CAN_EVENT_ACTIVITY;
    }

    case BL_CMD_BOOT_APP:
        BL_CAN_SetLastBootError((uint8_t)BL_BOOTERR_NONE);
        BL_BootToAppRequest = 1;
        BL_CAN_SendStatus(BL_STATUS_OK, 0x40);
        return BL_CAN_EVENT_ACTIVITY;

    case BL_CMD_BOOT_STATUS:
        BL_CAN_SendStatus(BL_STATUS_OK, BL_CAN_GetLastBootError());
        return BL_CAN_EVENT_ACTIVITY;

    case BL_CMD_I2C_BUF_CLEAR:
        bl_i2c_tx_len = 0U;
        BL_CAN_SendStatus(BL_STATUS_OK, 0);
        return BL_CAN_EVENT_ACTIVITY;

    case BL_CMD_I2C_BUF_APPEND: {
        if (len <= 1U) {
            BL_CAN_SendStatus(BL_STATUS_ERR_GENERIC, 0);
            return BL_CAN_EVENT_ACTIVITY;
        }
        if (!bl_i2c_ready) {
            BL_CAN_SendStatus(BL_STATUS_ERR_STATE, 0xE0);
            return BL_CAN_EVENT_ACTIVITY;
        }
        uint8_t add = (uint8_t)(len - 1U);
        if ((uint16_t)bl_i2c_tx_len + add > BL_I2C_MAX_TX) {
            BL_CAN_SendStatus(BL_STATUS_ERR_RANGE, BL_I2C_MAX_TX);
            return BL_CAN_EVENT_ACTIVITY;
        }
        for (uint8_t i = 0; i < add; ++i) {
            bl_i2c_tx_buf[bl_i2c_tx_len + i] = data[1U + i];
        }
        bl_i2c_tx_len = (uint8_t)(bl_i2c_tx_len + add);
        BL_CAN_SendStatus(BL_STATUS_OK, bl_i2c_tx_len);
        return BL_CAN_EVENT_ACTIVITY;
    }

    case BL_CMD_I2C_XFER: {
        if (len < 3U) {
            BL_CAN_SendStatus(BL_STATUS_ERR_GENERIC, 0);
            return BL_CAN_EVENT_ACTIVITY;
        }
        if (!bl_i2c_ready) {
            BL_CAN_SendStatus(BL_STATUS_ERR_STATE, 0xE0);
            return BL_CAN_EVENT_ACTIVITY;
        }
        uint8_t addr7 = (uint8_t)(data[1] & 0x7FU);
        uint8_t rx_len = data[2];
        if (addr7 > 0x7FU || rx_len > BL_I2C_MAX_RX) {
            BL_CAN_SendStatus(BL_STATUS_ERR_RANGE, 0);
            return BL_CAN_EVENT_ACTIVITY;
        }

        HAL_StatusTypeDef st = BL_I2C_DoTransfer(addr7, rx_len);
        if (st != HAL_OK) {
            BL_CAN_SendStatus(BL_STATUS_ERR_GENERIC, (uint8_t)(HAL_I2C_GetError(&g_hi2c) & 0xFFU));
            return BL_CAN_EVENT_ACTIVITY;
        }

        BL_CAN_SendChunked(BL_FRAME_I2C_RXDATA, bl_i2c_rx_buf, rx_len);
        return BL_CAN_EVENT_ACTIVITY;
    }

    case BL_CMD_I2C_SCAN: {
        if (!bl_i2c_ready) {
            BL_CAN_SendStatus(BL_STATUS_ERR_STATE, 0xE0);
            return BL_CAN_EVENT_ACTIVITY;
        }

        uint8_t first = BL_I2C_SCAN_FIRST_ADDR;
        uint8_t last  = BL_I2C_SCAN_LAST_ADDR;
        if (len >= 3U) {
            first = data[1];
            last  = data[2];
        }
        if (first > 0x7FU || last > 0x7FU || first > last) {
            BL_CAN_SendStatus(BL_STATUS_ERR_RANGE, 0);
            return BL_CAN_EVENT_ACTIVITY;
        }

        BL_I2C_DoScan(first, last);
        return BL_CAN_EVENT_ACTIVITY;
    }

    default:
        BL_CAN_SendStatus(BL_STATUS_ERR_GENERIC, 0xFF);
        return BL_CAN_EVENT_ACTIVITY;
    }

    return BL_CAN_EVENT_NONE;
}

uint32_t BL_CAN_Poll(void)
{
    uint32_t events = BL_CAN_EVENT_NONE;

    if (HAL_CAN_GetRxFifoFillLevel(g_hcan, CAN_RX_FIFO0) > 0) {
        CAN_RxHeaderTypeDef rx;
        uint8_t data[8];

        HAL_CAN_GetRxMessage(g_hcan, CAN_RX_FIFO0, &rx, data);
        if (rx.StdId == BL_CAN_CMD_ID && rx.RTR == CAN_RTR_DATA) {
            events |= BL_HandleCmd(data, rx.DLC);
        }
    }

    return events;
}
