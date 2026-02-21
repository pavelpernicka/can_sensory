#pragma once

#include "app_config.h"

typedef struct {
    uint8_t enabled;
    uint8_t brightness;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint16_t strip_len;
} ws2812_state_t;

typedef enum {
    WS2812_ANIM_STATIC = 0,
    WS2812_ANIM_BLINK = 1,
    WS2812_ANIM_BREATHE = 2,
    WS2812_ANIM_RAINBOW = 3,
    WS2812_ANIM_WIPE = 4,
    WS2812_ANIM_GRADIENT = 5,
} ws2812_anim_mode_t;

typedef struct {
    uint8_t mode;
    uint8_t speed;
} ws2812_anim_t;

typedef struct {
    uint8_t split_idx;
    uint8_t fade_px;
    uint16_t color1_rgb565;
    uint16_t color2_rgb565;
} ws2812_gradient_t;

void WS2812_Init(void);
void WS2812_SetEnabled(uint8_t enabled);
void WS2812_SetBrightness(uint8_t brightness);
void WS2812_SetColor(uint8_t r, uint8_t g, uint8_t b);
void WS2812_Apply(void);
void WS2812_GetState(ws2812_state_t *state);
void WS2812_SetAnim(uint8_t mode, uint8_t speed);
void WS2812_GetAnim(ws2812_anim_t *anim);
void WS2812_SetGradient(uint8_t split_idx, uint8_t fade_px, uint16_t c1_rgb565, uint16_t c2_rgb565);
void WS2812_GetGradient(ws2812_gradient_t *cfg);
void WS2812_Service(uint32_t now_ms);
