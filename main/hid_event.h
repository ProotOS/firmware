#pragma once

#include <stdint.h>
#include "hid_report_parser.h"

#define HID_EVENT_MAX_PREV_REPORT_SIZE 64

typedef enum {
    HID_EVENT_BUTTON_DOWN,
    HID_EVENT_BUTTON_UP,
    HID_EVENT_AXIS_CHANGE,
} hid_event_type_t;

typedef struct {
    hid_event_type_t type;
    uint8_t  device_index;   // Which connected remote (0-based)
    uint16_t usage_page;     // e.g. 0x09=Button, 0x07=Keyboard, 0x0C=Consumer
    uint16_t usage;          // Specific usage (Button 1, Key code, Consumer control code)
    int32_t  value;          // Axis value, or 1=pressed / 0=released for buttons
} hid_event_t;

typedef void (*hid_event_callback_t)(const hid_event_t *event);

/**
 * Set the callback that receives decoded HID events.
 */
void hid_event_set_callback(hid_event_callback_t cb);

/**
 * State tracker for one device. Stores previous report data
 * so we can detect changes and emit press/release events.
 */
typedef struct {
    uint8_t prev_data[HID_MAX_REPORTS][HID_EVENT_MAX_PREV_REPORT_SIZE];
    bool    has_prev[HID_MAX_REPORTS];
} hid_event_state_t;

/**
 * Process an incoming HID input report and emit events for any changes.
 *
 * @param map           Parsed report map for this device
 * @param state         Per-device event state (previous report data)
 * @param device_index  Device index (passed through to events)
 * @param report_id     Report ID (0 if device doesn't use report IDs)
 * @param data          Raw report data (excluding the report ID byte)
 * @param data_len      Length of data
 */
/**
 * Emit a HID event directly (for raw/unparsed reports).
 */
void hid_event_emit(hid_event_type_t type, uint8_t device_index,
                    uint16_t usage_page, uint16_t usage, int32_t value);

void hid_event_process_report(const hid_report_map_t *map,
                              hid_event_state_t *state,
                              uint8_t device_index,
                              uint8_t report_id,
                              const uint8_t *data,
                              size_t data_len);
