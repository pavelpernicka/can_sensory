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

void WS2812_Init(void);
void WS2812_SetEnabled(uint8_t enabled);
void WS2812_SetBrightness(uint8_t brightness);
void WS2812_SetColor(uint8_t r, uint8_t g, uint8_t b);
void WS2812_Apply(void);
void WS2812_GetState(ws2812_state_t *state);
