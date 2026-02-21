#include "ws2812.h"

#include <string.h>

typedef struct {
    uint8_t enabled;
    uint8_t brightness;
    uint8_t r;
    uint8_t g;
    uint8_t b;
} ws2812_runtime_t;

static ws2812_runtime_t g_ws;

static uint32_t ws_cycles_bit;
static uint32_t ws_cycles_t0h;
static uint32_t ws_cycles_t1h;

static void WS2812_EnableCycleCounter(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t WS2812_CycleNow(void)
{
    return DWT->CYCCNT;
}

static inline void WS2812_WaitUntil(uint32_t deadline)
{
    while ((int32_t)(WS2812_CycleNow() - deadline) < 0) {
    }
}

static inline void WS2812_PinSet(void)
{
    APP_WS2812_GPIO_PORT->BSRR = APP_WS2812_PIN;
}

static inline void WS2812_PinReset(void)
{
    APP_WS2812_GPIO_PORT->BSRR = (uint32_t)APP_WS2812_PIN << 16U;
}

static void WS2812_SendBit(uint8_t bit)
{
    uint32_t t0 = WS2812_CycleNow();
    uint32_t hi_deadline = t0 + (bit ? ws_cycles_t1h : ws_cycles_t0h);
    uint32_t bit_deadline = t0 + ws_cycles_bit;

    WS2812_PinSet();
    WS2812_WaitUntil(hi_deadline);
    WS2812_PinReset();
    WS2812_WaitUntil(bit_deadline);
}

static void WS2812_SendByte(uint8_t byte)
{
    for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1U) {
        WS2812_SendBit((byte & mask) ? 1U : 0U);
    }
}

static void WS2812_ResetLatch(void)
{
    uint32_t cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000U;
    uint32_t t0 = WS2812_CycleNow();
    uint32_t latch_cycles = cycles_per_us * 90U;

    WS2812_PinReset();
    WS2812_WaitUntil(t0 + latch_cycles);
}

void WS2812_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t hclk_hz;

    memset(&g_ws, 0, sizeof(g_ws));
    g_ws.enabled = 0U;
    g_ws.brightness = 64U;
    g_ws.r = 255U;
    g_ws.g = 255U;
    g_ws.b = 255U;

    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio.Pin = APP_WS2812_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(APP_WS2812_GPIO_PORT, &gpio);

    WS2812_EnableCycleCounter();

    hclk_hz = HAL_RCC_GetHCLKFreq();
    ws_cycles_bit = (hclk_hz + 400000U) / 800000U;      /* 1.25us */
    ws_cycles_t0h = (hclk_hz + 1250000U) / 2500000U;    /* 0.40us */
    ws_cycles_t1h = (hclk_hz + 625000U) / 1250000U;     /* 0.80us */

    if (ws_cycles_bit < 8U) {
        ws_cycles_bit = 8U;
    }
    if (ws_cycles_t0h < 2U) {
        ws_cycles_t0h = 2U;
    }
    if (ws_cycles_t1h <= ws_cycles_t0h) {
        ws_cycles_t1h = ws_cycles_t0h + 2U;
    }

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
    uint32_t primask = __get_PRIMASK();
    uint16_t n = APP_WS2812_STRIP_LEN;
    uint8_t r = 0U;
    uint8_t g = 0U;
    uint8_t b = 0U;

    if (g_ws.enabled) {
        uint16_t br = (uint16_t)g_ws.brightness;
        r = (uint8_t)(((uint16_t)g_ws.r * br + 127U) / 255U);
        g = (uint8_t)(((uint16_t)g_ws.g * br + 127U) / 255U);
        b = (uint8_t)(((uint16_t)g_ws.b * br + 127U) / 255U);
    }

    __disable_irq();
    for (uint16_t i = 0; i < n; ++i) {
        /* WS2812 expects GRB order. */
        WS2812_SendByte(g);
        WS2812_SendByte(r);
        WS2812_SendByte(b);
    }
    WS2812_ResetLatch();
    if (!primask) {
        __enable_irq();
    }
}
