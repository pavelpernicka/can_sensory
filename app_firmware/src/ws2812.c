#include "ws2812.h"

#include <string.h>

#define WS2812_SPI_INSTANCE            SPI1
#define WS2812_SPI_GPIO_AF             GPIO_AF5_SPI1
#define WS2812_SYM_0                   0x18U  /* 11000 */
#define WS2812_SYM_1                   0x1CU  /* 11100 */
#define WS2812_RESET_BYTES             32U
#define WS2812_BYTES_PER_LED           15U    /* 24 bits * 5 encoded bits / 8 */
#define WS2812_TX_MAX_BYTES            ((APP_WS2812_STRIP_LEN * WS2812_BYTES_PER_LED) + WS2812_RESET_BYTES)

typedef struct {
    uint8_t enabled;
    uint8_t brightness;
    uint8_t r;
    uint8_t g;
    uint8_t b;
} ws2812_runtime_t;

static ws2812_runtime_t g_ws;
static SPI_HandleTypeDef g_hspi;
static uint8_t g_spi_ready;
static uint8_t g_tx_buf[WS2812_TX_MAX_BYTES];

static void WS2812_PackSym5(uint8_t sym, uint16_t *bit_pos)
{
    for (uint8_t mask = 0x10U; mask != 0U; mask >>= 1U) {
        uint16_t p = *bit_pos;
        uint16_t byte_i = (uint16_t)(p >> 3);
        uint8_t bit_i = (uint8_t)(7U - (p & 7U));
        if (sym & mask) {
            g_tx_buf[byte_i] |= (uint8_t)(1U << bit_i);
        }
        *bit_pos = (uint16_t)(p + 1U);
    }
}

static void WS2812_PackByte(uint8_t value, uint16_t *bit_pos)
{
    for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1U) {
        WS2812_PackSym5((value & mask) ? WS2812_SYM_1 : WS2812_SYM_0, bit_pos);
    }
}

void WS2812_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    memset(&g_ws, 0, sizeof(g_ws));
    memset(&g_hspi, 0, sizeof(g_hspi));
    g_ws.enabled = 0U;
    g_ws.brightness = 64U;
    g_ws.r = 255U;
    g_ws.g = 255U;
    g_ws.b = 255U;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();

    gpio.Pin = APP_WS2812_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = WS2812_SPI_GPIO_AF;
    HAL_GPIO_Init(APP_WS2812_GPIO_PORT, &gpio);

    g_hspi.Instance = WS2812_SPI_INSTANCE;
    g_hspi.Init.Mode = SPI_MODE_MASTER;
    g_hspi.Init.Direction = SPI_DIRECTION_2LINES;
    g_hspi.Init.DataSize = SPI_DATASIZE_8BIT;
    g_hspi.Init.CLKPolarity = SPI_POLARITY_LOW;
    g_hspi.Init.CLKPhase = SPI_PHASE_1EDGE;
    g_hspi.Init.NSS = SPI_NSS_SOFT;
    g_hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4; /* 16MHz / 4 = 4MHz */
    g_hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    g_hspi.Init.TIMode = SPI_TIMODE_DISABLE;
    g_hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    g_hspi.Init.CRCPolynomial = 7U;
    g_hspi.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    g_hspi.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

    g_spi_ready = (HAL_SPI_Init(&g_hspi) == HAL_OK) ? 1U : 0U;
    WS2812_Apply();
}

void WS2812_SetEnabled(uint8_t enabled)
{
    g_ws.enabled = enabled ? 1U : 0U;
}

void WS2812_SetBrightness(uint8_t brightness)
{
    g_ws.brightness = brightness;
}

void WS2812_SetColor(uint8_t r, uint8_t g, uint8_t b)
{
    g_ws.r = r;
    g_ws.g = g;
    g_ws.b = b;
}

void WS2812_GetState(ws2812_state_t *state)
{
    if (state == NULL) {
        return;
    }

    state->enabled = g_ws.enabled;
    state->brightness = g_ws.brightness;
    state->r = g_ws.r;
    state->g = g_ws.g;
    state->b = g_ws.b;
    state->strip_len = APP_WS2812_STRIP_LEN;
}

void WS2812_Apply(void)
{
    uint16_t bit_pos = 0U;
    uint16_t tx_len;
    uint8_t r = 0U;
    uint8_t g = 0U;
    uint8_t b = 0U;

    if (!g_spi_ready) {
        return;
    }

    if (g_ws.enabled) {
        uint16_t br = (uint16_t)g_ws.brightness;
        r = (uint8_t)(((uint16_t)g_ws.r * br + 127U) / 255U);
        g = (uint8_t)(((uint16_t)g_ws.g * br + 127U) / 255U);
        b = (uint8_t)(((uint16_t)g_ws.b * br + 127U) / 255U);
    }

    memset(g_tx_buf, 0, sizeof(g_tx_buf));
    for (uint16_t i = 0; i < APP_WS2812_STRIP_LEN; ++i) {
        /* WS2812 expects GRB order. */
        WS2812_PackByte(g, &bit_pos);
        WS2812_PackByte(r, &bit_pos);
        WS2812_PackByte(b, &bit_pos);
    }

    tx_len = (uint16_t)(bit_pos >> 3);
    tx_len = (uint16_t)(tx_len + WS2812_RESET_BYTES);
    if (tx_len > sizeof(g_tx_buf)) {
        tx_len = sizeof(g_tx_buf);
    }

    HAL_SPI_Transmit(&g_hspi, g_tx_buf, tx_len, 20U);
}
