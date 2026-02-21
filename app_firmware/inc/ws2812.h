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
    WS2812_ANIM_SECTOR_FOLLOW = 6,
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

#define WS2812_MAX_SECTORS 64U
#define WS2812_MAX_ZONES   32U

typedef struct {
    uint8_t enabled;
    uint8_t fade_speed;
    uint8_t sector_count;
    uint8_t active_sector;
    uint8_t target_sector;
    uint8_t max_zones;
} ws2812_sector_mode_t;

typedef struct {
    uint8_t idx;
    uint8_t r;
    uint8_t g;
    uint8_t b;
} ws2812_sector_color_t;

typedef struct {
    uint8_t idx;
    uint8_t start_led; /* 1-based index, 0 = disabled */
    uint8_t end_led;   /* 1-based index */
    uint8_t sector;    /* event sector id */
    uint16_t color_rgb565;
} ws2812_sector_zone_t;

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
void WS2812_SetSectorMode(uint8_t enabled, uint8_t fade_speed, uint8_t sector_count);
void WS2812_GetSectorMode(ws2812_sector_mode_t *cfg);
void WS2812_SetSectorColor(uint8_t idx, uint8_t r, uint8_t g, uint8_t b);
void WS2812_GetSectorColor(uint8_t idx, ws2812_sector_color_t *cfg);
void WS2812_SetSectorZone(uint8_t idx, uint8_t start_led, uint8_t end_led, uint8_t sector, uint16_t color_rgb565);
void WS2812_GetSectorZone(uint8_t idx, ws2812_sector_zone_t *cfg);
void WS2812_SetActiveSector(uint8_t sector);
void WS2812_Service(uint32_t now_ms);
