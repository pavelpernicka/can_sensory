#include "events.h"

#include <math.h>
#include <string.h>

#define EVENT_QUEUE_CAPACITY 16U
#define EVENT_BUFFER_SIZE 5U
#define APP_NUM_SECTORS 6U

typedef struct {
    float center_x;
    float center_y;
    float center_z;
    float rotate_xy_deg;
    float rotate_xz_deg;
    float rotate_yz_deg;
    float keepout_rad;
    float z_limit;
    float data_radius;
    uint8_t num_sectors;
    float change_threshold;
    uint32_t deactivation_timeout_ms;
    uint32_t session_timeout_ms;
} event_cfg_t;

static event_cfg_t g_cfg;

typedef struct {
    app_event_t q[EVENT_QUEUE_CAPACITY];
    uint8_t head;
    uint8_t tail;

    uint8_t sector_buf[EVENT_BUFFER_SIZE];
    float elevation_buf[EVENT_BUFFER_SIZE];
    uint8_t buf_len;
    uint8_t buf_pos;

    uint8_t last_sector;
    float last_elevation;
    uint32_t last_event_ms;
    uint32_t last_nonzero_ms;
    uint8_t session_active;

    uint32_t last_sector_event_ms[APP_NUM_SECTORS + 1U];
    uint8_t deactivated_mask;

    uint8_t last_state_elevation;
    uint32_t last_no_data_ms;
} event_state_t;

static event_state_t g_state;

static uint8_t clamp_u8(int32_t v)
{
    if (v < 0) {
        return 0U;
    }
    if (v > 255) {
        return 255U;
    }
    return (uint8_t)v;
}

static void queue_event(uint8_t type, uint8_t p0, uint8_t p1, uint8_t p2, uint16_t p3)
{
    uint8_t next = (uint8_t)((g_state.head + 1U) % EVENT_QUEUE_CAPACITY);
    app_event_t *e;

    if (next == g_state.tail) {
        return;
    }

    e = &g_state.q[g_state.head];
    e->type = type;
    e->p0 = p0;
    e->p1 = p1;
    e->p2 = p2;
    e->p3 = p3;
    g_state.head = next;
}

static void event_cfg_set_defaults(void)
{
    g_cfg.center_x = 0.0f;
    g_cfg.center_y = 0.0f;
    g_cfg.center_z = 0.0f;
    g_cfg.rotate_xy_deg = 0.0f;
    g_cfg.rotate_xz_deg = 0.0f;
    g_cfg.rotate_yz_deg = 0.0f;
    g_cfg.keepout_rad = 1000.0f;
    g_cfg.z_limit = 150.0f;
    g_cfg.data_radius = 3000.0f;
    g_cfg.num_sectors = APP_NUM_SECTORS;
    g_cfg.change_threshold = 3.0f;
    g_cfg.deactivation_timeout_ms = 5000U;
    g_cfg.session_timeout_ms = 10000U;
}

static void rotate_3d(float x, float y, float z, float *xr, float *yr, float *zr)
{
    float rad_xy = g_cfg.rotate_xy_deg * (float)M_PI / 180.0f;
    float rad_xz = g_cfg.rotate_xz_deg * (float)M_PI / 180.0f;
    float rad_yz = g_cfg.rotate_yz_deg * (float)M_PI / 180.0f;

    float x1 = x * cosf(rad_xy) - y * sinf(rad_xy);
    float y1 = x * sinf(rad_xy) + y * cosf(rad_xy);
    float z1 = z;

    float x2 = x1 * cosf(rad_xz) - z1 * sinf(rad_xz);
    float z2 = x1 * sinf(rad_xz) + z1 * cosf(rad_xz);
    float y2 = y1;

    float y3 = y2 * cosf(rad_yz) - z2 * sinf(rad_yz);
    float z3 = y2 * sinf(rad_yz) + z2 * cosf(rad_yz);

    *xr = x2;
    *yr = y3;
    *zr = z3;
}

static void get_sector(float x, float y, float z, uint8_t *sector_out, uint8_t *elev_out)
{
    float xr;
    float yr;
    float zr;
    float dx;
    float dy;
    float distance;
    float azimuth;
    float elevation;

    z -= g_cfg.center_z;
    rotate_3d(x, y, z, &xr, &yr, &zr);

    dx = xr - g_cfg.center_x;
    dy = yr - g_cfg.center_y;
    distance = sqrtf((dx * dx) + (dy * dy));
    if (distance <= g_cfg.keepout_rad || zr < g_cfg.z_limit) {
        *sector_out = 0U;
        *elev_out = 0U;
        return;
    }

    azimuth = (atan2f(dy, dx) * 180.0f / (float)M_PI);
    while (azimuth < 0.0f) {
        azimuth += 360.0f;
    }
    while (azimuth >= 360.0f) {
        azimuth -= 360.0f;
    }

    *sector_out = (uint8_t)(azimuth / (360.0f / g_cfg.num_sectors)) + 1U;
    elevation = zr - g_cfg.z_limit;
    if (elevation < 0.0f) {
        elevation = 0.0f;
    }
    if (elevation > 255.0f) {
        elevation = 255.0f;
    }
    *elev_out = (uint8_t)elevation;
}

void Events_Init(void)
{
    memset(&g_state, 0, sizeof(g_state));
    event_cfg_set_defaults();
    g_state.last_event_ms = HAL_GetTick();
    g_state.last_nonzero_ms = g_state.last_event_ms;
}

void Events_ProcessMagSample(float x, float y, float z, uint32_t now_ms)
{
    uint8_t sector;
    uint8_t elevation_u8;
    float elevation_avg = 0.0f;
    float speed;
    float dt_sec;
    int32_t sector_diff;

    get_sector(x, y, z, &sector, &elevation_u8);

    g_state.sector_buf[g_state.buf_pos] = sector;
    g_state.elevation_buf[g_state.buf_pos] = (float)elevation_u8;
    g_state.buf_pos = (uint8_t)((g_state.buf_pos + 1U) % EVENT_BUFFER_SIZE);
    if (g_state.buf_len < EVENT_BUFFER_SIZE) {
        g_state.buf_len++;
    }

    if (g_state.buf_len < EVENT_BUFFER_SIZE) {
        g_state.last_event_ms = now_ms;
        g_state.last_sector = sector;
        g_state.last_elevation = (float)elevation_u8;
        g_state.last_state_elevation = elevation_u8;
        return;
    }

    for (uint8_t i = 0; i < EVENT_BUFFER_SIZE; ++i) {
        elevation_avg += g_state.elevation_buf[i];
    }
    elevation_avg /= (float)EVENT_BUFFER_SIZE;

    dt_sec = (float)(now_ms - g_state.last_event_ms) / 1000.0f;
    if (dt_sec < 0.001f) {
        dt_sec = 0.001f;
    }
    speed = fabsf(elevation_avg - g_state.last_elevation) / dt_sec;

    if (sector != g_state.last_sector) {
        if (g_state.last_sector == 0U) {
            queue_event(APP_EVENT_SECTOR_ACTIVATED, sector, (uint8_t)elevation_avg, clamp_u8((int32_t)speed), (uint16_t)(now_ms & 0xFFFFU));
            if (!g_state.session_active) {
                queue_event(APP_EVENT_SESSION_STARTED, 0U, 0U, 0U, (uint16_t)(now_ms & 0xFFFFU));
                g_state.session_active = 1U;
            }
        } else if (sector != 0U) {
            sector_diff = (int32_t)g_state.last_sector - (int32_t)sector;
            if (sector_diff < 0) {
                sector_diff = -sector_diff;
            }
            if (sector_diff == 1 && (now_ms - g_state.last_event_ms) < 20U) {
                queue_event(APP_EVENT_PASSING_SECTOR_CHANGE, sector, 0U, 0U, (uint16_t)(now_ms & 0xFFFFU));
            } else {
                queue_event(APP_EVENT_SECTOR_CHANGED, g_state.last_sector, sector, 0U, (uint16_t)(now_ms & 0xFFFFU));
            }
        }

        if (sector <= APP_NUM_SECTORS) {
            g_state.deactivated_mask &= (uint8_t)~(1U << sector);
            g_state.last_sector_event_ms[sector] = now_ms;
        }
    } else if (fabsf(elevation_avg - g_state.last_elevation) > g_cfg.change_threshold) {
        if ((g_state.deactivated_mask & (1U << sector)) == 0U) {
            queue_event(APP_EVENT_INTENSITY_CHANGE, sector, (uint8_t)elevation_avg, clamp_u8((int32_t)speed), (uint16_t)(now_ms & 0xFFFFU));
            if (sector <= APP_NUM_SECTORS) {
                g_state.last_sector_event_ms[sector] = now_ms;
            }
        }
    }

    if (g_state.last_sector != 0U) {
        g_state.last_nonzero_ms = now_ms;
    }

    if (g_state.last_sector != 0U && g_state.last_sector <= APP_NUM_SECTORS) {
        uint32_t sec_last_ms = g_state.last_sector_event_ms[g_state.last_sector];
        if (sec_last_ms != 0U && (now_ms - sec_last_ms) > g_cfg.deactivation_timeout_ms) {
            queue_event(APP_EVENT_SECTION_DEACTIVATED, g_state.last_sector, 0U, 0U, (uint16_t)(now_ms & 0xFFFFU));
            if (g_state.session_active) {
                queue_event(APP_EVENT_SESSION_ENDED, 0U, 0U, 0U, (uint16_t)(now_ms & 0xFFFFU));
                g_state.session_active = 0U;
            }
            g_state.deactivated_mask |= (uint8_t)(1U << g_state.last_sector);
            g_state.last_sector_event_ms[g_state.last_sector] = 0U;
        }
    }

    if (g_state.last_sector != 0U && (now_ms - g_state.last_event_ms) > g_cfg.session_timeout_ms) {
        queue_event(APP_EVENT_POSSIBLE_MECHANICAL_FAILURE, g_state.last_sector, 0U, 0U, (uint16_t)(now_ms & 0xFFFFU));
    } else if (g_state.last_sector == 0U && (now_ms - g_state.last_nonzero_ms) > g_cfg.session_timeout_ms) {
        if (g_state.session_active) {
            queue_event(APP_EVENT_SESSION_ENDED, 0U, 0U, 0U, (uint16_t)(now_ms & 0xFFFFU));
            g_state.session_active = 0U;
        }
    }

    g_state.last_sector = sector;
    g_state.last_elevation = elevation_avg;
    g_state.last_state_elevation = (uint8_t)elevation_avg;
    g_state.last_event_ms = now_ms;
}

void Events_PostNoData(uint32_t now_ms)
{
    if ((now_ms - g_state.last_no_data_ms) < g_cfg.session_timeout_ms) {
        return;
    }
    queue_event(APP_EVENT_ERROR_NO_DATA, 0U, 0U, 0U, (uint16_t)(now_ms & 0xFFFFU));
    g_state.last_no_data_ms = now_ms;
}

int Events_Pop(app_event_t *out)
{
    if (out == 0) {
        return 0;
    }
    if (g_state.head == g_state.tail) {
        return 0;
    }

    *out = g_state.q[g_state.tail];
    g_state.tail = (uint8_t)((g_state.tail + 1U) % EVENT_QUEUE_CAPACITY);
    return 1;
}

void Events_GetSectorState(uint8_t *sector, uint8_t *elevation)
{
    if (sector != 0) {
        *sector = g_state.last_sector;
    }
    if (elevation != 0) {
        *elevation = g_state.last_state_elevation;
    }
}

void Events_ApplyCalibration(const app_calibration_t *cal)
{
    if (cal == 0) {
        return;
    }

    g_cfg.center_x = (float)cal->center_x_mg;
    g_cfg.center_y = (float)cal->center_y_mg;
    g_cfg.center_z = (float)cal->center_z_mg;
    g_cfg.rotate_xy_deg = (float)cal->rotate_xy_cdeg / 100.0f;
    g_cfg.rotate_xz_deg = (float)cal->rotate_xz_cdeg / 100.0f;
    g_cfg.rotate_yz_deg = (float)cal->rotate_yz_cdeg / 100.0f;
    g_cfg.keepout_rad = (float)cal->keepout_rad_mg;
    g_cfg.z_limit = (float)cal->z_limit_mg;
    g_cfg.data_radius = (float)cal->data_radius_mg;
}
