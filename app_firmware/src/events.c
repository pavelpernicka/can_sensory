#include "events.h"

#include "event_detector.h"

#include <string.h>

#define EVENT_QUEUE_CAPACITY 16U

typedef struct {
    app_event_t q[EVENT_QUEUE_CAPACITY];
    uint8_t head;
    uint8_t tail;
} event_queue_t;

static event_queue_t g_queue;
static event_detector_t g_detector;

static void queue_push(const app_event_t *e)
{
    uint8_t next;

    if (e == 0) {
        return;
    }

    next = (uint8_t)((g_queue.head + 1U) % EVENT_QUEUE_CAPACITY);
    if (next == g_queue.tail) {
        return;
    }

    g_queue.q[g_queue.head] = *e;
    g_queue.head = next;
}

void Events_Init(void)
{
    memset(&g_queue, 0, sizeof(g_queue));
    EventDetector_Init(&g_detector, HAL_GetTick());
}

void Events_ProcessMagSample(float x, float y, float z, uint32_t now_ms)
{
    app_event_t out[EVENT_DETECTOR_MAX_EVENTS_PER_STEP];
    uint8_t count = EventDetector_ProcessMagSample(&g_detector, x, y, z, now_ms, out,
                                                   EVENT_DETECTOR_MAX_EVENTS_PER_STEP);

    for (uint8_t i = 0; i < count; ++i) {
        queue_push(&out[i]);
    }
}

void Events_PostNoData(uint32_t now_ms)
{
    app_event_t out[1];
    uint8_t count = EventDetector_PostNoData(&g_detector, now_ms, out, 1U);

    if (count != 0U) {
        queue_push(&out[0]);
    }
}

int Events_Pop(app_event_t *out)
{
    if (out == 0) {
        return 0;
    }
    if (g_queue.head == g_queue.tail) {
        return 0;
    }

    *out = g_queue.q[g_queue.tail];
    g_queue.tail = (uint8_t)((g_queue.tail + 1U) % EVENT_QUEUE_CAPACITY);
    return 1;
}

void Events_GetSectorState(uint8_t *sector, uint8_t *elevation)
{
    EventDetector_GetSectorState(&g_detector, sector, elevation);
}

void Events_ApplyCalibration(const app_calibration_t *cal)
{
    EventDetector_ApplyCalibration(&g_detector, cal);
}
