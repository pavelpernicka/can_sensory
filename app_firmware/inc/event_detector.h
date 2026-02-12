#pragma once

#include "events.h"

#define EVENT_DETECTOR_BUFFER_SIZE         5U
#define EVENT_DETECTOR_MIN_SECTORS         1U
#define EVENT_DETECTOR_MAX_SECTORS         16U
#define EVENT_DETECTOR_DEFAULT_SECTORS     6U
#define EVENT_DETECTOR_MAX_EVENTS_PER_STEP 4U

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

    uint8_t sector_buf[EVENT_DETECTOR_BUFFER_SIZE];
    float elevation_buf[EVENT_DETECTOR_BUFFER_SIZE];
    uint8_t buf_len;
    uint8_t buf_pos;

    uint8_t last_sector;
    float last_elevation;
    uint32_t last_event_ms;
    uint32_t last_nonzero_ms;
    uint8_t session_active;

    uint32_t last_sector_event_ms[EVENT_DETECTOR_MAX_SECTORS + 1U];
    uint32_t deactivated_mask;

    uint8_t last_state_elevation;
    uint32_t last_no_data_ms;
} event_detector_t;

void EventDetector_Init(event_detector_t *det, uint32_t now_ms);
void EventDetector_ApplyCalibration(event_detector_t *det, const app_calibration_t *cal);
uint8_t EventDetector_ProcessMagSample(event_detector_t *det, float x, float y, float z, uint32_t now_ms,
                                       app_event_t *out_events, uint8_t out_capacity);
uint8_t EventDetector_PostNoData(event_detector_t *det, uint32_t now_ms, app_event_t *out_events,
                                 uint8_t out_capacity);
void EventDetector_GetSectorState(const event_detector_t *det, uint8_t *sector, uint8_t *elevation);
