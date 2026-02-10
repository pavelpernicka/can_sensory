#include "app_can.h"

static CAN_HandleTypeDef *g_hcan;
static uint8_t g_device_id = APP_DEVICE_ID;
static uint16_t g_cmd_id = (uint16_t)(APP_CAN_BASE_CMD_ID + APP_DEVICE_ID);
static uint16_t g_status_id = (uint16_t)(APP_CAN_BASE_STATUS_ID + APP_DEVICE_ID);

void APP_CAN_SetDeviceId(uint8_t device_id)
{
    if (device_id > APP_DEVICE_ID_MAX) {
        device_id = APP_DEVICE_ID;
    }

    g_device_id = device_id;
    g_cmd_id = (uint16_t)(APP_CAN_BASE_CMD_ID + device_id);
    g_status_id = (uint16_t)(APP_CAN_BASE_STATUS_ID + device_id);
}

uint8_t APP_CAN_GetDeviceId(void)
{
    return g_device_id;
}

void APP_CAN_Init(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef filter = {0};

    g_hcan = hcan;

    g_hcan->Instance = APP_CAN_INSTANCE;
    g_hcan->Init.Prescaler = APP_CAN_BITRATE_PRESCALER;
    g_hcan->Init.Mode = CAN_MODE_NORMAL;
    g_hcan->Init.SyncJumpWidth = APP_CAN_BITRATE_SJW;
    g_hcan->Init.TimeSeg1 = APP_CAN_BITRATE_BS1;
    g_hcan->Init.TimeSeg2 = APP_CAN_BITRATE_BS2;
    g_hcan->Init.TimeTriggeredMode = DISABLE;
    g_hcan->Init.AutoBusOff = DISABLE;
    g_hcan->Init.AutoWakeUp = DISABLE;
    g_hcan->Init.AutoRetransmission = ENABLE;
    g_hcan->Init.ReceiveFifoLocked = DISABLE;
    g_hcan->Init.TransmitFifoPriority = DISABLE;

    HAL_CAN_Init(g_hcan);

    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = (uint16_t)(g_cmd_id << 5);
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0xFFE0;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;

    HAL_CAN_ConfigFilter(g_hcan, &filter);
    HAL_CAN_Start(g_hcan);
}

void APP_CAN_SendFrame(const uint8_t *data, uint8_t dlc)
{
    CAN_TxHeaderTypeDef tx = {0};
    uint8_t payload[8] = {0};
    uint32_t mbox;
    uint32_t spins;

    if (dlc > 8U) {
        dlc = 8U;
    }

    for (uint8_t i = 0; i < dlc; ++i) {
        payload[i] = data[i];
    }

    tx.StdId = g_status_id;
    tx.IDE = CAN_ID_STD;
    tx.RTR = CAN_RTR_DATA;
    tx.DLC = dlc;

    /* Do not depend on SysTick/HAL_GetTick here: bounded spin is fail-safe. */
    spins = 200000U;
    while (HAL_CAN_GetTxMailboxesFreeLevel(g_hcan) == 0U) {
        if (spins-- == 0U) {
            return;
        }
    }

    if (HAL_CAN_AddTxMessage(g_hcan, &tx, payload, &mbox) != HAL_OK) {
        spins = 200000U;
        while (HAL_CAN_GetTxMailboxesFreeLevel(g_hcan) == 0U) {
            if (spins-- == 0U) {
                return;
            }
        }
        HAL_CAN_AddTxMessage(g_hcan, &tx, payload, &mbox);
    }
}

void APP_CAN_SendStatus(app_status_t status, uint8_t extra)
{
    uint8_t data[8] = {0};
    data[0] = (uint8_t)status;
    data[1] = extra;
    APP_CAN_SendFrame(data, 8);
}

int APP_CAN_TryRecv(uint8_t *data, uint8_t *len)
{
    CAN_RxHeaderTypeDef rx;

    if (HAL_CAN_GetRxFifoFillLevel(g_hcan, CAN_RX_FIFO0) == 0U) {
        return 0;
    }

    HAL_CAN_GetRxMessage(g_hcan, CAN_RX_FIFO0, &rx, data);
    if (rx.StdId != g_cmd_id || rx.RTR != CAN_RTR_DATA) {
        return 0;
    }

    *len = rx.DLC;
    return 1;
}
