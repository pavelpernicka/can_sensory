#include "event_detector.h"

#include <math.h>
#include <string.h>

static uint8_t sanitize_sector_count(uint8_t n)
{
    if (n < EVENT_DETECTOR_MIN_SECTORS || n > EVENT_DETECTOR_MAX_SECTORS) {
        return EVENT_DETECTOR_DEFAULT_SECTORS;
    }
    return n;
}

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

static uint8_t append_event(app_event_t *out_events, uint8_t out_capacity, uint8_t out_count,
                            uint8_t type, uint8_t p0, uint8_t p1, uint8_t p2, uint16_t p3)
{
    if (out_count >= out_capacity || out_events == 0) {
        return out_count;
    }

    out_events[out_count].type = type;
    out_events[out_count].p0 = p0;
    out_events[out_count].p1 = p1;
    out_events[out_count].p2 = p2;
    out_events[out_count].p3 = p3;
    return (uint8_t)(out_count + 1U);
}

static void rotate_3d(const event_detector_t *det, float x, float y, float z, float *xr, float *yr, float *zr)
{
    float rad_xy = det->rotate_xy_deg * (float)M_PI / 180.0f;
    float rad_xz = det->rotate_xz_deg * (float)M_PI / 180.0f;
    float rad_yz = det->rotate_yz_deg * (float)M_PI / 180.0f;

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

static void get_sector(const event_detector_t *det, float x, float y, float z, uint8_t *sector_out, uint8_t *elev_out)
{
    float xr;
    float yr;
    float zr;
    float dx;
    float dy;
    float distance;
    float azimuth;
    float normalized;
    float span;
    float curve;
    float elevation;

    z -= det->center_z;
    rotate_3d(det, x, y, z, &xr, &yr, &zr);

    dx = xr - det->center_x;
    dy = yr - det->center_y;
    distance = sqrtf((dx * dx) + (dy * dy));
    if (distance <= det->keepout_rad || zr < det->z_limit) {
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

    *sector_out = (uint8_t)(azimuth / (360.0f / (float)det->num_sectors)) + 1U;
    span = det->z_max - det->z_limit;
    if (span < 1.0f) {
        span = 1.0f;
    }
    normalized = (zr - det->z_limit) / span;
    if (normalized < 0.0f) {
        normalized = 0.0f;
    } else if (normalized > 1.0f) {
        normalized = 1.0f;
    }

    curve = det->elev_curve;
    if (curve < 0.01f) {
        curve = 0.01f;
    }
    elevation = powf(normalized, curve) * 255.0f;
    *elev_out = clamp_u8((int32_t)(elevation + 0.5f));
}

void EventDetector_Init(event_detector_t *det, uint32_t now_ms)
{
    if (det == 0) {
        return;
    }

    memset(det, 0, sizeof(*det));
    det->center_x = 0.0f;
    det->center_y = 0.0f;
    det->center_z = 0.0f;
    det->rotate_xy_deg = 0.0f;
    det->rotate_xz_deg = 0.0f;
    det->rotate_yz_deg = 0.0f;
    det->keepout_rad = 1000.0f;
    det->z_limit = 150.0f;
    det->z_max = 405.0f;
    det->elev_curve = 1.0f;
    det->data_radius = 3000.0f;
    det->num_sectors = EVENT_DETECTOR_DEFAULT_SECTORS;
    det->change_threshold = 3.0f;
    det->deactivation_timeout_ms = 5000U;
    det->session_timeout_ms = 10000U;
    det->last_event_ms = now_ms;
    det->last_nonzero_ms = now_ms;
}

void EventDetector_ApplyCalibration(event_detector_t *det, const app_calibration_t *cal)
{
    if (det == 0 || cal == 0) {
        return;
    }

    det->center_x = (float)cal->center_x_mg;
    det->center_y = (float)cal->center_y_mg;
    det->center_z = (float)cal->center_z_mg;
    det->rotate_xy_deg = (float)cal->rotate_xy_cdeg / 100.0f;
    det->rotate_xz_deg = (float)cal->rotate_xz_cdeg / 100.0f;
    det->rotate_yz_deg = (float)cal->rotate_yz_cdeg / 100.0f;
    det->keepout_rad = (float)cal->keepout_rad_mg;
    det->z_limit = (float)cal->z_limit_mg;
    det->z_max = (float)cal->z_max_mg;
    det->elev_curve = (float)cal->elev_curve_centi / 100.0f;
    det->data_radius = (float)cal->data_radius_mg;
    det->num_sectors = sanitize_sector_count(cal->num_sectors);
}

uint8_t EventDetector_ProcessMagSample(event_detector_t *det, float x, float y, float z, uint32_t now_ms,
                                       app_event_t *out_events, uint8_t out_capacity)
{
    uint8_t out_count = 0U;
    uint8_t sector;
    uint8_t elevation_u8;
    float elevation_avg = 0.0f;
    float speed;
    float dt_sec;
    int32_t sector_diff;
    int32_t wrap_diff;

    if (det == 0) {
        return 0U;
    }

    get_sector(det, x, y, z, &sector, &elevation_u8);

    det->sector_buf[det->buf_pos] = sector;
    det->elevation_buf[det->buf_pos] = (float)elevation_u8;
    det->buf_pos = (uint8_t)((det->buf_pos + 1U) % EVENT_DETECTOR_BUFFER_SIZE);
    if (det->buf_len < EVENT_DETECTOR_BUFFER_SIZE) {
        det->buf_len++;
    }

    if (det->buf_len < EVENT_DETECTOR_BUFFER_SIZE) {
        det->last_event_ms = now_ms;
        det->last_sector = sector;
        det->last_elevation = (float)elevation_u8;
        det->last_state_elevation = elevation_u8;
        return 0U;
    }

    for (uint8_t i = 0; i < EVENT_DETECTOR_BUFFER_SIZE; ++i) {
        elevation_avg += det->elevation_buf[i];
    }
    elevation_avg /= (float)EVENT_DETECTOR_BUFFER_SIZE;

    dt_sec = (float)(now_ms - det->last_event_ms) / 1000.0f;
    if (dt_sec < 0.001f) {
        dt_sec = 0.001f;
    }
    speed = fabsf(elevation_avg - det->last_elevation) / dt_sec;

    if (sector != det->last_sector) {
        if (det->last_sector == 0U) {
            out_count = append_event(out_events, out_capacity, out_count,
                                     APP_EVENT_SECTOR_ACTIVATED, sector, (uint8_t)elevation_avg,
                                     clamp_u8((int32_t)speed), (uint16_t)(now_ms & 0xFFFFU));
            if (!det->session_active) {
                out_count = append_event(out_events, out_capacity, out_count,
                                         APP_EVENT_SESSION_STARTED, 0U, 0U, 0U,
                                         (uint16_t)(now_ms & 0xFFFFU));
                det->session_active = 1U;
            }
        } else if (sector != 0U) {
            sector_diff = (int32_t)det->last_sector - (int32_t)sector;
            if (sector_diff < 0) {
                sector_diff = -sector_diff;
            }
            wrap_diff = (int32_t)det->num_sectors - sector_diff;
            if ((sector_diff == 1 || wrap_diff == 1) && (now_ms - det->last_event_ms) < 20U) {
                out_count = append_event(out_events, out_capacity, out_count,
                                         APP_EVENT_PASSING_SECTOR_CHANGE, sector, 0U, 0U,
                                         (uint16_t)(now_ms & 0xFFFFU));
            } else {
                out_count = append_event(out_events, out_capacity, out_count,
                                         APP_EVENT_SECTOR_CHANGED, det->last_sector, sector, 0U,
                                         (uint16_t)(now_ms & 0xFFFFU));
            }
        }

        if (sector >= EVENT_DETECTOR_MIN_SECTORS && sector <= det->num_sectors) {
            det->deactivated_mask &= ~(1UL << sector);
            det->last_sector_event_ms[sector] = now_ms;
        }
    } else if (sector != 0U && fabsf(elevation_avg - det->last_elevation) > det->change_threshold) {
        if ((det->deactivated_mask & (1UL << sector)) == 0U) {
            out_count = append_event(out_events, out_capacity, out_count,
                                     APP_EVENT_INTENSITY_CHANGE, sector, (uint8_t)elevation_avg,
                                     clamp_u8((int32_t)speed), (uint16_t)(now_ms & 0xFFFFU));
            if (sector <= det->num_sectors) {
                det->last_sector_event_ms[sector] = now_ms;
            }
        }
    }

    if (det->last_sector != 0U) {
        det->last_nonzero_ms = now_ms;
    }

    if (det->last_sector != 0U && det->last_sector <= det->num_sectors) {
        uint32_t sec_last_ms = det->last_sector_event_ms[det->last_sector];
        if (sec_last_ms != 0U && (now_ms - sec_last_ms) > det->deactivation_timeout_ms) {
            out_count = append_event(out_events, out_capacity, out_count,
                                     APP_EVENT_SECTION_DEACTIVATED, det->last_sector, 0U, 0U,
                                     (uint16_t)(now_ms & 0xFFFFU));
            if (det->session_active) {
                out_count = append_event(out_events, out_capacity, out_count,
                                         APP_EVENT_SESSION_ENDED, 0U, 0U, 0U,
                                         (uint16_t)(now_ms & 0xFFFFU));
                det->session_active = 0U;
            }
            det->deactivated_mask |= (1UL << det->last_sector);
            det->last_sector_event_ms[det->last_sector] = 0U;
        }
    }

    if (det->last_sector != 0U && (now_ms - det->last_event_ms) > det->session_timeout_ms) {
        out_count = append_event(out_events, out_capacity, out_count,
                                 APP_EVENT_POSSIBLE_MECHANICAL_FAILURE, det->last_sector, 0U, 0U,
                                 (uint16_t)(now_ms & 0xFFFFU));
    } else if (det->last_sector == 0U && (now_ms - det->last_nonzero_ms) > det->session_timeout_ms) {
        if (det->session_active) {
            out_count = append_event(out_events, out_capacity, out_count,
                                     APP_EVENT_SESSION_ENDED, 0U, 0U, 0U,
                                     (uint16_t)(now_ms & 0xFFFFU));
            det->session_active = 0U;
        }
    }

    det->last_sector = sector;
    det->last_elevation = elevation_avg;
    det->last_state_elevation = (uint8_t)elevation_avg;
    det->last_event_ms = now_ms;
    return out_count;
}

uint8_t EventDetector_PostNoData(event_detector_t *det, uint32_t now_ms, app_event_t *out_events,
                                 uint8_t out_capacity)
{
    if (det == 0) {
        return 0U;
    }
    if ((now_ms - det->last_no_data_ms) < det->session_timeout_ms) {
        return 0U;
    }
    det->last_no_data_ms = now_ms;
    return append_event(out_events, out_capacity, 0U, APP_EVENT_ERROR_NO_DATA, 0U, 0U, 0U,
                        (uint16_t)(now_ms & 0xFFFFU));
}

void EventDetector_GetSectorState(const event_detector_t *det, uint8_t *sector, uint8_t *elevation)
{
    if (det == 0) {
        return;
    }
    if (sector != 0) {
        *sector = det->last_sector;
    }
    if (elevation != 0) {
        *elevation = det->last_state_elevation;
    }
}
