#include "ws2812.h"

#include <string.h>

#define WS2812_SPI_INSTANCE            SPI1
#define WS2812_SPI_GPIO_AF             GPIO_AF5_SPI1
#define WS2812_SYM_0                   0x10U  /* 10000: 0.25us high @ 4MHz */
#define WS2812_SYM_1                   0x1CU  /* 11100: 0.75us high @ 4MHz */
#define WS2812_RESET_BYTES             64U    /* >80us low reset */
#define WS2812_BYTES_PER_LED           15U    /* 24 bits * 5 encoded bits / 8 */
#define WS2812_TX_MAX_BYTES            ((APP_WS2812_STRIP_LEN * WS2812_BYTES_PER_LED) + WS2812_RESET_BYTES)

typedef struct {
    uint8_t enabled;
    uint8_t strip_len;
    uint8_t brightness;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t anim_mode;
    uint8_t anim_speed;
    uint16_t anim_step;
    uint32_t next_anim_ms;
    uint8_t grad_split_idx;
    uint8_t grad_fade_px;
    uint16_t grad_c1_rgb565;
    uint16_t grad_c2_rgb565;
    uint8_t sector_mode_enabled;
    uint8_t sector_fade_speed;
    uint8_t sector_count;
    uint8_t sector_active;
    uint8_t sector_target;
    uint8_t sector_colors[8][3]; /* legacy 1..8 palette */
    ws2812_sector_zone_t zones[WS2812_MAX_ZONES];
    uint8_t sector_cur_led[APP_WS2812_STRIP_LEN][3];
    uint8_t sector_tgt_led[APP_WS2812_STRIP_LEN][3];
    uint8_t dirty;
} ws2812_runtime_t;

static ws2812_runtime_t g_ws;
static SPI_HandleTypeDef g_hspi;
static uint8_t g_spi_ready;
static uint8_t g_tx_buf[WS2812_TX_MAX_BYTES];

static uint8_t ws_scale_u8(uint8_t value, uint8_t scale)
{
    return (uint8_t)(((uint16_t)value * (uint16_t)scale + 127U) / 255U);
}

static uint8_t ws_step_to_target_u8(uint8_t cur, uint8_t tgt, uint8_t step)
{
    if (cur == tgt || step == 0U) {
        return tgt;
    }
    if (cur < tgt) {
        uint8_t d = (uint8_t)(tgt - cur);
        return (d <= step) ? tgt : (uint8_t)(cur + step);
    }
    {
        uint8_t d = (uint8_t)(cur - tgt);
        return (d <= step) ? tgt : (uint8_t)(cur - step);
    }
}

static uint8_t ws_lerp_u8(uint8_t a, uint8_t b, uint16_t t, uint16_t t_max)
{
    uint32_t av = (uint32_t)a;
    uint32_t bv = (uint32_t)b;
    if (t_max == 0U) {
        return b;
    }
    return (uint8_t)((((t_max - t) * av) + (t * bv) + (t_max / 2U)) / t_max);
}

static void ws_rgb565_to_rgb888(uint16_t c, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t r5 = (uint8_t)((c >> 11) & 0x1FU);
    uint8_t g6 = (uint8_t)((c >> 5) & 0x3FU);
    uint8_t b5 = (uint8_t)(c & 0x1FU);
    *r = (uint8_t)((r5 * 255U + 15U) / 31U);
    *g = (uint8_t)((g6 * 255U + 31U) / 63U);
    *b = (uint8_t)((b5 * 255U + 15U) / 31U);
}

static uint16_t ws_map_speed_ms(uint8_t speed, uint16_t slow_ms, uint16_t fast_ms)
{
    uint16_t span;
    if (slow_ms <= fast_ms) {
        return fast_ms;
    }
    span = (uint16_t)(slow_ms - fast_ms);
    return (uint16_t)(slow_ms - (((uint32_t)span * (uint32_t)speed) / 255U));
}

static uint8_t ws_active_len(void)
{
    if (g_ws.strip_len == 0U || g_ws.strip_len > APP_WS2812_STRIP_LEN) {
        return APP_WS2812_STRIP_LEN;
    }
    return g_ws.strip_len;
}

static void ws_hue_to_rgb(uint8_t hue, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t region = (uint8_t)(hue / 43U);
    uint8_t rem = (uint8_t)((hue - (region * 43U)) * 6U);
    uint8_t q = (uint8_t)(255U - rem);
    uint8_t t = rem;

    switch (region) {
    case 0: *r = 255U; *g = t;    *b = 0U;   break;
    case 1: *r = q;    *g = 255U; *b = 0U;   break;
    case 2: *r = 0U;   *g = 255U; *b = t;    break;
    case 3: *r = 0U;   *g = q;    *b = 255U; break;
    case 4: *r = t;    *g = 0U;   *b = 255U; break;
    default:*r = 255U; *g = 0U;   *b = q;    break;
    }
}

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

static void WS2812_PackPixel(uint8_t r, uint8_t g, uint8_t b, uint16_t *bit_pos)
{
    /* WS2812 expects GRB order. */
    WS2812_PackByte(g, bit_pos);
    WS2812_PackByte(r, bit_pos);
    WS2812_PackByte(b, bit_pos);
}

static void WS2812_TransmitPacked(uint16_t bit_pos)
{
    uint16_t tx_len;

    tx_len = (uint16_t)(bit_pos >> 3);
    tx_len = (uint16_t)(tx_len + WS2812_RESET_BYTES);
    if (tx_len > sizeof(g_tx_buf)) {
        tx_len = sizeof(g_tx_buf);
    }

    HAL_SPI_Transmit(&g_hspi, g_tx_buf, tx_len, 20U);
}

static void WS2812_RenderSolid(uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t bit_pos = 0U;
    uint8_t n = ws_active_len();

    memset(g_tx_buf, 0, sizeof(g_tx_buf));
    for (uint16_t i = 0; i < APP_WS2812_STRIP_LEN; ++i) {
        if (i < n) {
            WS2812_PackPixel(r, g, b, &bit_pos);
        } else {
            WS2812_PackPixel(0U, 0U, 0U, &bit_pos);
        }
    }
    WS2812_TransmitPacked(bit_pos);
}

static void WS2812_RenderStatic(void)
{
    uint8_t r = 0U;
    uint8_t g = 0U;
    uint8_t b = 0U;

    if (g_ws.enabled) {
        r = ws_scale_u8(g_ws.r, g_ws.brightness);
        g = ws_scale_u8(g_ws.g, g_ws.brightness);
        b = ws_scale_u8(g_ws.b, g_ws.brightness);
    }

    WS2812_RenderSolid(r, g, b);
}

static void WS2812_RenderBlink(void)
{
    uint16_t bit_pos = 0U;
    uint8_t n = ws_active_len();
    uint8_t on = (uint8_t)(g_ws.anim_step & 1U);
    uint8_t r = on ? ws_scale_u8(g_ws.r, g_ws.brightness) : 0U;
    uint8_t g = on ? ws_scale_u8(g_ws.g, g_ws.brightness) : 0U;
    uint8_t b = on ? ws_scale_u8(g_ws.b, g_ws.brightness) : 0U;

    memset(g_tx_buf, 0, sizeof(g_tx_buf));
    for (uint16_t i = 0; i < APP_WS2812_STRIP_LEN; ++i) {
        if (i < n) {
            WS2812_PackPixel(r, g, b, &bit_pos);
        } else {
            WS2812_PackPixel(0U, 0U, 0U, &bit_pos);
        }
    }
    WS2812_TransmitPacked(bit_pos);
}

static void WS2812_RenderBreathe(void)
{
    uint16_t bit_pos = 0U;
    uint8_t n = ws_active_len();
    uint16_t phase = (uint16_t)(g_ws.anim_step % 512U);
    uint8_t level = (phase < 256U) ? (uint8_t)phase : (uint8_t)(511U - phase);
    uint8_t gain = ws_scale_u8(level, g_ws.brightness);
    uint8_t r = ws_scale_u8(g_ws.r, gain);
    uint8_t g = ws_scale_u8(g_ws.g, gain);
    uint8_t b = ws_scale_u8(g_ws.b, gain);

    memset(g_tx_buf, 0, sizeof(g_tx_buf));
    for (uint16_t i = 0; i < APP_WS2812_STRIP_LEN; ++i) {
        if (i < n) {
            WS2812_PackPixel(r, g, b, &bit_pos);
        } else {
            WS2812_PackPixel(0U, 0U, 0U, &bit_pos);
        }
    }
    WS2812_TransmitPacked(bit_pos);
}

static void WS2812_RenderRainbow(void)
{
    uint16_t bit_pos = 0U;
    uint8_t n = ws_active_len();
    uint8_t base = (uint8_t)g_ws.anim_step;

    memset(g_tx_buf, 0, sizeof(g_tx_buf));
    for (uint16_t i = 0; i < APP_WS2812_STRIP_LEN; ++i) {
        if (i >= n) {
            WS2812_PackPixel(0U, 0U, 0U, &bit_pos);
            continue;
        }
        uint8_t r, g, b;
        ws_hue_to_rgb((uint8_t)(base + (uint8_t)(i * 4U)), &r, &g, &b);
        r = ws_scale_u8(r, g_ws.brightness);
        g = ws_scale_u8(g, g_ws.brightness);
        b = ws_scale_u8(b, g_ws.brightness);
        WS2812_PackPixel(r, g, b, &bit_pos);
    }
    WS2812_TransmitPacked(bit_pos);
}

static void WS2812_RenderWipe(void)
{
    uint16_t bit_pos = 0U;
    uint8_t n = ws_active_len();
    uint16_t n_on = (uint16_t)(g_ws.anim_step % ((uint16_t)n + 1U));
    uint8_t r = ws_scale_u8(g_ws.r, g_ws.brightness);
    uint8_t g = ws_scale_u8(g_ws.g, g_ws.brightness);
    uint8_t b = ws_scale_u8(g_ws.b, g_ws.brightness);

    memset(g_tx_buf, 0, sizeof(g_tx_buf));
    for (uint16_t i = 0; i < APP_WS2812_STRIP_LEN; ++i) {
        if (i < n && i < n_on) {
            WS2812_PackPixel(r, g, b, &bit_pos);
        } else {
            WS2812_PackPixel(0U, 0U, 0U, &bit_pos);
        }
    }
    WS2812_TransmitPacked(bit_pos);
}

static void WS2812_RenderGradient(void)
{
    uint16_t bit_pos = 0U;
    uint8_t n = ws_active_len();
    uint8_t stop_pos[WS2812_MAX_ZONES];
    uint8_t stop_r[WS2812_MAX_ZONES];
    uint8_t stop_g[WS2812_MAX_ZONES];
    uint8_t stop_b[WS2812_MAX_ZONES];
    uint8_t stop_count = 0U;

    for (uint8_t i = 0U; i < WS2812_MAX_ZONES; ++i) {
        uint8_t pos = g_ws.zones[i].pos_led;
        uint8_t r, g, b;
        if (pos == 0U || pos > n) {
            continue;
        }
        ws_rgb565_to_rgb888(g_ws.zones[i].color_rgb565, &r, &g, &b);
        if (stop_count < WS2812_MAX_ZONES) {
            stop_pos[stop_count] = pos;
            stop_r[stop_count] = r;
            stop_g[stop_count] = g;
            stop_b[stop_count] = b;
            ++stop_count;
        }
    }

    /* Sort by position. */
    for (uint8_t i = 1U; i < stop_count; ++i) {
        uint8_t p = stop_pos[i];
        uint8_t r = stop_r[i];
        uint8_t g = stop_g[i];
        uint8_t b = stop_b[i];
        uint8_t j = i;
        while (j > 0U && stop_pos[j - 1U] > p) {
            stop_pos[j] = stop_pos[j - 1U];
            stop_r[j] = stop_r[j - 1U];
            stop_g[j] = stop_g[j - 1U];
            stop_b[j] = stop_b[j - 1U];
            --j;
        }
        stop_pos[j] = p;
        stop_r[j] = r;
        stop_g[j] = g;
        stop_b[j] = b;
    }

    /* Merge duplicate positions (last wins). */
    if (stop_count > 1U) {
        uint8_t w = 0U;
        for (uint8_t i = 0U; i < stop_count; ++i) {
            if (w > 0U && stop_pos[w - 1U] == stop_pos[i]) {
                stop_r[w - 1U] = stop_r[i];
                stop_g[w - 1U] = stop_g[i];
                stop_b[w - 1U] = stop_b[i];
                continue;
            }
            stop_pos[w] = stop_pos[i];
            stop_r[w] = stop_r[i];
            stop_g[w] = stop_g[i];
            stop_b[w] = stop_b[i];
            ++w;
        }
        stop_count = w;
    }

    memset(g_tx_buf, 0, sizeof(g_tx_buf));
    for (uint16_t i = 0; i < APP_WS2812_STRIP_LEN; ++i) {
        uint8_t r = 0U, g = 0U, b = 0U;
        if (i >= n || !g_ws.enabled) {
            WS2812_PackPixel(0U, 0U, 0U, &bit_pos);
            continue;
        }

        if (stop_count == 0U) {
            r = g_ws.r;
            g = g_ws.g;
            b = g_ws.b;
        } else if (stop_count == 1U) {
            r = stop_r[0];
            g = stop_g[0];
            b = stop_b[0];
        } else {
            uint16_t p = (uint16_t)i + 1U;
            uint8_t seg = 0U;
            for (uint8_t s = 0U; s < stop_count; ++s) {
                uint16_t a = stop_pos[s];
                uint16_t bpos = (s + 1U < stop_count) ? stop_pos[s + 1U] : (uint16_t)(stop_pos[0] + n);
                uint16_t p2 = (p < a) ? (uint16_t)(p + n) : p;
                if (p2 >= a && p2 <= bpos) {
                    seg = s;
                    break;
                }
            }

            {
                uint16_t a = stop_pos[seg];
                uint16_t bpos = (seg + 1U < stop_count) ? stop_pos[seg + 1U] : (uint16_t)(stop_pos[0] + n);
                uint16_t p2 = (p < a) ? (uint16_t)(p + n) : p;
                uint16_t t = (uint16_t)(p2 - a);
                uint16_t tmax = (uint16_t)(bpos - a);
                uint8_t nseg = (uint8_t)((seg + 1U) % stop_count);
                r = ws_lerp_u8(stop_r[seg], stop_r[nseg], t, tmax);
                g = ws_lerp_u8(stop_g[seg], stop_g[nseg], t, tmax);
                b = ws_lerp_u8(stop_b[seg], stop_b[nseg], t, tmax);
            }
        }
        r = ws_scale_u8(r, g_ws.brightness);
        g = ws_scale_u8(g, g_ws.brightness);
        b = ws_scale_u8(b, g_ws.brightness);
        WS2812_PackPixel(r, g, b, &bit_pos);
    }
    WS2812_TransmitPacked(bit_pos);
}

static void WS2812_UpdateSectorTarget(void)
{
    uint8_t n = ws_active_len();
    uint8_t r = 0U, g = 0U, b = 0U;

    memset(g_ws.sector_tgt_led, 0, sizeof(g_ws.sector_tgt_led));
    g_ws.sector_target = 0U;

    if (!g_ws.sector_mode_enabled || g_ws.sector_active == 0U) {
        return;
    }
    if (g_ws.sector_active <= 8U) {
        uint8_t idx = (uint8_t)(g_ws.sector_active - 1U);
        r = g_ws.sector_colors[idx][0];
        g = g_ws.sector_colors[idx][1];
        b = g_ws.sector_colors[idx][2];
    }
    for (uint16_t led = 0U; led < n; ++led) {
        g_ws.sector_tgt_led[led][0] = r;
        g_ws.sector_tgt_led[led][1] = g;
        g_ws.sector_tgt_led[led][2] = b;
    }
    g_ws.sector_target = g_ws.sector_active;
}

static void WS2812_RenderSectorFollow(void)
{
    uint16_t bit_pos = 0U;
    uint8_t n = ws_active_len();

    memset(g_tx_buf, 0, sizeof(g_tx_buf));
    for (uint16_t i = 0; i < APP_WS2812_STRIP_LEN; ++i) {
        if (i >= n) {
            WS2812_PackPixel(0U, 0U, 0U, &bit_pos);
            continue;
        }
        uint8_t r = ws_scale_u8(g_ws.sector_cur_led[i][0], g_ws.brightness);
        uint8_t g = ws_scale_u8(g_ws.sector_cur_led[i][1], g_ws.brightness);
        uint8_t b = ws_scale_u8(g_ws.sector_cur_led[i][2], g_ws.brightness);
        if (!g_ws.enabled) {
            r = g = b = 0U;
        }
        WS2812_PackPixel(r, g, b, &bit_pos);
    }
    WS2812_TransmitPacked(bit_pos);
}

void WS2812_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    memset(&g_ws, 0, sizeof(g_ws));
    memset(&g_hspi, 0, sizeof(g_hspi));
    g_ws.enabled = 0U;
    g_ws.strip_len = APP_WS2812_STRIP_LEN;
    g_ws.brightness = 64U;
    g_ws.r = 255U;
    g_ws.g = 255U;
    g_ws.b = 255U;
    g_ws.anim_mode = WS2812_ANIM_STATIC;
    g_ws.anim_speed = 120U;
    g_ws.anim_step = 0U;
    g_ws.next_anim_ms = 0U;
    g_ws.grad_split_idx = (uint8_t)(APP_WS2812_STRIP_LEN / 2U);
    g_ws.grad_fade_px = 4U;
    g_ws.grad_c1_rgb565 = 0x001FU; /* blue */
    g_ws.grad_c2_rgb565 = 0xF800U; /* red */
    g_ws.sector_mode_enabled = 0U;
    g_ws.sector_fade_speed = 128U;
    g_ws.sector_count = 16U;
    g_ws.sector_active = 0U;
    g_ws.sector_target = 0U;
    memset(g_ws.sector_cur_led, 0, sizeof(g_ws.sector_cur_led));
    memset(g_ws.sector_tgt_led, 0, sizeof(g_ws.sector_tgt_led));
    for (uint8_t i = 0; i < 8U; ++i) {
        uint8_t r, g, b;
        ws_hue_to_rgb((uint8_t)(i * 32U), &r, &g, &b);
        g_ws.sector_colors[i][0] = r;
        g_ws.sector_colors[i][1] = g;
        g_ws.sector_colors[i][2] = b;
    }
    memset(g_ws.zones, 0, sizeof(g_ws.zones));
    g_ws.zones[0].idx = 1U;
    g_ws.zones[0].pos_led = 1U;
    g_ws.zones[0].color_rgb565 = 0xF800U; /* red */
    g_ws.zones[1].idx = 2U;
    g_ws.zones[1].pos_led = (uint8_t)(APP_WS2812_STRIP_LEN / 3U);
    g_ws.zones[1].color_rgb565 = 0x07E0U; /* green */
    g_ws.zones[2].idx = 3U;
    g_ws.zones[2].pos_led = (uint8_t)((2U * APP_WS2812_STRIP_LEN) / 3U);
    g_ws.zones[2].color_rgb565 = 0x001FU; /* blue */
    g_ws.dirty = 1U;

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
    HAL_Delay(2);
    WS2812_Apply();
}

void WS2812_SetEnabled(uint8_t enabled)
{
    g_ws.enabled = enabled ? 1U : 0U;
    g_ws.dirty = 1U;
}

void WS2812_SetBrightness(uint8_t brightness)
{
    g_ws.brightness = brightness;
    g_ws.dirty = 1U;
}

void WS2812_SetColor(uint8_t r, uint8_t g, uint8_t b)
{
    g_ws.r = r;
    g_ws.g = g;
    g_ws.b = b;
    g_ws.dirty = 1U;
}

void WS2812_SetAnim(uint8_t mode, uint8_t speed)
{
    if (mode > WS2812_ANIM_SECTOR_FOLLOW) {
        mode = WS2812_ANIM_STATIC;
    }
    g_ws.anim_mode = mode;
    g_ws.anim_speed = speed;
    g_ws.anim_step = 0U;
    g_ws.next_anim_ms = HAL_GetTick();
    g_ws.sector_mode_enabled = (mode == WS2812_ANIM_SECTOR_FOLLOW) ? 1U : 0U;
    WS2812_UpdateSectorTarget();
    g_ws.dirty = 1U;
}

void WS2812_GetAnim(ws2812_anim_t *anim)
{
    if (anim == NULL) {
        return;
    }
    anim->mode = g_ws.anim_mode;
    anim->speed = g_ws.anim_speed;
}

void WS2812_SetGradient(uint8_t split_idx, uint8_t fade_px, uint16_t c1_rgb565, uint16_t c2_rgb565)
{
    if (split_idx == 0U) {
        split_idx = 1U;
    }
    if (split_idx > APP_WS2812_STRIP_LEN) {
        split_idx = APP_WS2812_STRIP_LEN;
    }
    g_ws.grad_split_idx = split_idx;
    g_ws.grad_fade_px = fade_px;
    g_ws.grad_c1_rgb565 = c1_rgb565;
    g_ws.grad_c2_rgb565 = c2_rgb565;

    /* Backward compatibility: map 2-color gradient command to first two gradient stops. */
    g_ws.zones[0].idx = 1U;
    g_ws.zones[0].pos_led = 1U;
    g_ws.zones[0].color_rgb565 = c1_rgb565;
    g_ws.zones[1].idx = 2U;
    g_ws.zones[1].pos_led = split_idx;
    g_ws.zones[1].color_rgb565 = c2_rgb565;
    for (uint8_t i = 2U; i < WS2812_MAX_ZONES; ++i) {
        g_ws.zones[i].idx = (uint8_t)(i + 1U);
        g_ws.zones[i].pos_led = 0U;
        g_ws.zones[i].color_rgb565 = 0U;
    }
    g_ws.dirty = 1U;
}

void WS2812_GetGradient(ws2812_gradient_t *cfg)
{
    if (cfg == NULL) {
        return;
    }
    cfg->split_idx = g_ws.grad_split_idx;
    cfg->fade_px = g_ws.grad_fade_px;
    cfg->color1_rgb565 = g_ws.grad_c1_rgb565;
    cfg->color2_rgb565 = g_ws.grad_c2_rgb565;
}

void WS2812_SetSectorMode(uint8_t enabled, uint8_t fade_speed, uint8_t sector_count)
{
    if (sector_count == 0U) {
        sector_count = 1U;
    }
    if (sector_count > WS2812_MAX_SECTORS) {
        sector_count = WS2812_MAX_SECTORS;
    }

    g_ws.sector_mode_enabled = enabled ? 1U : 0U;
    g_ws.sector_fade_speed = fade_speed;
    g_ws.sector_count = sector_count;
    g_ws.anim_mode = g_ws.sector_mode_enabled ? WS2812_ANIM_SECTOR_FOLLOW : WS2812_ANIM_STATIC;
    g_ws.next_anim_ms = HAL_GetTick();
    WS2812_UpdateSectorTarget();
    g_ws.dirty = 1U;
}

void WS2812_GetSectorMode(ws2812_sector_mode_t *cfg)
{
    if (cfg == NULL) {
        return;
    }
    cfg->enabled = g_ws.sector_mode_enabled;
    cfg->fade_speed = g_ws.sector_fade_speed;
    cfg->sector_count = g_ws.sector_count;
    cfg->active_sector = g_ws.sector_active;
    cfg->target_sector = g_ws.sector_target;
    cfg->max_zones = WS2812_MAX_ZONES;
}

void WS2812_SetSectorColor(uint8_t idx, uint8_t r, uint8_t g, uint8_t b)
{
    if (idx == 0U || idx > 8U) {
        return;
    }
    g_ws.sector_colors[idx - 1U][0] = r;
    g_ws.sector_colors[idx - 1U][1] = g;
    g_ws.sector_colors[idx - 1U][2] = b;
    WS2812_UpdateSectorTarget();
    g_ws.dirty = 1U;
}

void WS2812_GetSectorColor(uint8_t idx, ws2812_sector_color_t *cfg)
{
    if (cfg == NULL) {
        return;
    }
    if (idx == 0U || idx > 8U) {
        cfg->idx = 0U;
        cfg->r = 0U;
        cfg->g = 0U;
        cfg->b = 0U;
        return;
    }
    cfg->idx = idx;
    cfg->r = g_ws.sector_colors[idx - 1U][0];
    cfg->g = g_ws.sector_colors[idx - 1U][1];
    cfg->b = g_ws.sector_colors[idx - 1U][2];
}

void WS2812_SetSectorZone(uint8_t idx, uint8_t pos_led, uint16_t color_rgb565)
{
    ws2812_sector_zone_t *z;
    if (idx == 0U || idx > WS2812_MAX_ZONES) {
        return;
    }
    z = &g_ws.zones[idx - 1U];
    z->idx = idx;
    z->pos_led = pos_led;
    z->color_rgb565 = color_rgb565;

    if (pos_led > APP_WS2812_STRIP_LEN) {
        z->pos_led = 0U;
        z->color_rgb565 = 0U;
    }
    g_ws.dirty = 1U;
}

void WS2812_GetSectorZone(uint8_t idx, ws2812_sector_zone_t *cfg)
{
    if (cfg == NULL) {
        return;
    }
    cfg->idx = idx;
    cfg->pos_led = 0U;
    cfg->color_rgb565 = 0U;
    if (idx == 0U || idx > WS2812_MAX_ZONES) {
        return;
    }
    *cfg = g_ws.zones[idx - 1U];
}

void WS2812_SetActiveSector(uint8_t sector)
{
    if (g_ws.sector_active == sector) {
        return;
    }
    g_ws.sector_active = sector;
    WS2812_UpdateSectorTarget();
    g_ws.dirty = 1U;
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
    state->strip_len = g_ws.strip_len;
}

void WS2812_SetStripLength(uint8_t strip_len)
{
    if (strip_len == 0U) {
        strip_len = 1U;
    }
    if (strip_len > APP_WS2812_STRIP_LEN) {
        strip_len = APP_WS2812_STRIP_LEN;
    }
    g_ws.strip_len = strip_len;
    WS2812_UpdateSectorTarget();
    g_ws.dirty = 1U;
}

void WS2812_GetStripLength(uint8_t *strip_len, uint8_t *max_strip_len)
{
    if (strip_len != NULL) {
        *strip_len = g_ws.strip_len;
    }
    if (max_strip_len != NULL) {
        *max_strip_len = APP_WS2812_STRIP_LEN;
    }
}

void WS2812_Apply(void)
{
    if (!g_spi_ready) {
        return;
    }

    if (g_ws.anim_mode == WS2812_ANIM_STATIC) {
        WS2812_RenderStatic();
    } else if (!g_ws.enabled) {
        WS2812_RenderStatic();
    } else {
        /* Apply current animation frame without advancing timebase. */
        switch (g_ws.anim_mode) {
        case WS2812_ANIM_BLINK:   WS2812_RenderBlink();   break;
        case WS2812_ANIM_BREATHE: WS2812_RenderBreathe(); break;
        case WS2812_ANIM_RAINBOW: WS2812_RenderRainbow(); break;
        case WS2812_ANIM_WIPE:    WS2812_RenderWipe();    break;
        case WS2812_ANIM_GRADIENT:WS2812_RenderGradient();break;
        case WS2812_ANIM_SECTOR_FOLLOW: WS2812_RenderSectorFollow(); break;
        default:                  WS2812_RenderStatic();  break;
        }
    }
    g_ws.dirty = 0U;
}

void WS2812_Service(uint32_t now_ms)
{
    uint16_t interval_ms;

    if (!g_spi_ready) {
        return;
    }
    if (g_ws.dirty) {
        WS2812_Apply();
    }
    if (!g_ws.enabled) {
        return;
    }
    if (g_ws.anim_mode == WS2812_ANIM_STATIC) {
        return;
    }
    if (g_ws.anim_mode == WS2812_ANIM_GRADIENT) {
        return;
    }
    if ((int32_t)(now_ms - g_ws.next_anim_ms) < 0) {
        return;
    }

    switch (g_ws.anim_mode) {
    case WS2812_ANIM_BLINK:
        interval_ms = ws_map_speed_ms(g_ws.anim_speed, 900U, 80U);
        g_ws.anim_step = (uint16_t)(g_ws.anim_step + 1U);
        WS2812_RenderBlink();
        break;
    case WS2812_ANIM_BREATHE:
        interval_ms = ws_map_speed_ms(g_ws.anim_speed, 20U, 4U);
        g_ws.anim_step = (uint16_t)((g_ws.anim_step + 4U) & 0x1FFU);
        WS2812_RenderBreathe();
        break;
    case WS2812_ANIM_RAINBOW:
        interval_ms = ws_map_speed_ms(g_ws.anim_speed, 90U, 8U);
        g_ws.anim_step = (uint16_t)(g_ws.anim_step + 1U);
        WS2812_RenderRainbow();
        break;
    case WS2812_ANIM_WIPE:
        {
        uint8_t n = ws_active_len();
        interval_ms = ws_map_speed_ms(g_ws.anim_speed, 160U, 20U);
        g_ws.anim_step = (uint16_t)(g_ws.anim_step + 1U);
        if (g_ws.anim_step > ((uint16_t)n + 2U)) {
            g_ws.anim_step = 0U;
        }
        WS2812_RenderWipe();
        break;
        }
    case WS2812_ANIM_SECTOR_FOLLOW: {
        uint8_t step = (uint8_t)(1U + (g_ws.sector_fade_speed / 24U));
        interval_ms = ws_map_speed_ms(g_ws.sector_fade_speed, 40U, 4U);
        for (uint16_t led = 0; led < APP_WS2812_STRIP_LEN; ++led) {
            g_ws.sector_cur_led[led][0] = ws_step_to_target_u8(g_ws.sector_cur_led[led][0], g_ws.sector_tgt_led[led][0], step);
            g_ws.sector_cur_led[led][1] = ws_step_to_target_u8(g_ws.sector_cur_led[led][1], g_ws.sector_tgt_led[led][1], step);
            g_ws.sector_cur_led[led][2] = ws_step_to_target_u8(g_ws.sector_cur_led[led][2], g_ws.sector_tgt_led[led][2], step);
        }
        WS2812_RenderSectorFollow();
        break;
    }
    default:
        interval_ms = 100U;
        WS2812_RenderStatic();
        break;
    }

    g_ws.next_anim_ms = now_ms + interval_ms;
}
