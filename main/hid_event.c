#include "hid_event.h"
#include <string.h>
#include "esp_log.h"

static const char *TAG = "HID_Event";
static hid_event_callback_t s_callback = NULL;

void hid_event_set_callback(hid_event_callback_t cb)
{
    s_callback = cb;
}

static void emit(hid_event_type_t type, uint8_t device_index,
                 uint16_t usage_page, uint16_t usage, int32_t value)
{
    if (!s_callback) return;
    hid_event_t ev = {
        .type = type,
        .device_index = device_index,
        .usage_page = usage_page,
        .usage = usage,
        .value = value,
    };
    s_callback(&ev);
}

void hid_event_emit(hid_event_type_t type, uint8_t device_index,
                    uint16_t usage_page, uint16_t usage, int32_t value)
{
    emit(type, device_index, usage_page, usage, value);
}

// Find which report index in the map matches this report_id
static int find_report_index(const hid_report_map_t *map, uint8_t report_id)
{
    for (int i = 0; i < map->report_count; i++) {
        if (map->reports[i].report_id == report_id) return i;
    }
    return -1;
}

static void process_button_field(const hid_field_t *field,
                                 const uint8_t *data, size_t data_len,
                                 const uint8_t *prev_data, size_t prev_len,
                                 bool has_prev, uint8_t device_index)
{
    // Variable buttons: each bit is a separate button
    // Usage range maps 1:1 to each count index
    for (uint8_t i = 0; i < field->count; i++) {
        int32_t cur = hid_extract_field_value(data, data_len, field, i);
        int32_t prev = has_prev ? hid_extract_field_value(prev_data, prev_len, field, i) : 0;

        if (cur != prev) {
            uint16_t usage = field->usage_min + i;
            if (cur) {
                emit(HID_EVENT_BUTTON_DOWN, device_index, field->usage_page, usage, 1);
            } else {
                emit(HID_EVENT_BUTTON_UP, device_index, field->usage_page, usage, 0);
            }
        }
    }
}

static void process_array_field(const hid_field_t *field,
                                const uint8_t *data, size_t data_len,
                                const uint8_t *prev_data, size_t prev_len,
                                bool has_prev, uint8_t device_index)
{
    // Array fields (like keyboard keycodes): each element contains a usage value.
    // Compare old and new arrays to find pressed/released keys.

    // Collect current usages
    int32_t cur_usages[16];
    uint8_t cur_count = field->count < 16 ? field->count : 16;
    for (uint8_t i = 0; i < cur_count; i++) {
        cur_usages[i] = hid_extract_field_value(data, data_len, field, i);
    }

    // Collect previous usages
    int32_t prev_usages[16] = {0};
    uint8_t prev_count = cur_count;
    if (has_prev) {
        for (uint8_t i = 0; i < prev_count; i++) {
            prev_usages[i] = hid_extract_field_value(prev_data, prev_len, field, i);
        }
    }

    // Keys in current but not in previous → key down
    for (uint8_t i = 0; i < cur_count; i++) {
        if (cur_usages[i] == 0) continue; // 0 = no key
        bool found = false;
        for (uint8_t j = 0; j < prev_count; j++) {
            if (cur_usages[i] == prev_usages[j]) { found = true; break; }
        }
        if (!found) {
            emit(HID_EVENT_BUTTON_DOWN, device_index, field->usage_page,
                 (uint16_t)cur_usages[i], 1);
        }
    }

    // Keys in previous but not in current → key up
    if (has_prev) {
        for (uint8_t i = 0; i < prev_count; i++) {
            if (prev_usages[i] == 0) continue;
            bool found = false;
            for (uint8_t j = 0; j < cur_count; j++) {
                if (prev_usages[i] == cur_usages[j]) { found = true; break; }
            }
            if (!found) {
                emit(HID_EVENT_BUTTON_UP, device_index, field->usage_page,
                     (uint16_t)prev_usages[i], 0);
            }
        }
    }
}

static void process_axis_field(const hid_field_t *field,
                               const uint8_t *data, size_t data_len,
                               const uint8_t *prev_data, size_t prev_len,
                               bool has_prev, uint8_t device_index)
{
    for (uint8_t i = 0; i < field->count; i++) {
        int32_t cur = hid_extract_field_value(data, data_len, field, i);
        int32_t prev = has_prev ? hid_extract_field_value(prev_data, prev_len, field, i) : 0;

        if (cur != prev) {
            uint16_t usage = (field->usage_min != field->usage_max)
                ? field->usage_min + i
                : field->usage;
            emit(HID_EVENT_AXIS_CHANGE, device_index, field->usage_page, usage, cur);
        }
    }
}

void hid_event_process_report(const hid_report_map_t *map,
                              hid_event_state_t *state,
                              uint8_t device_index,
                              uint8_t report_id,
                              const uint8_t *data,
                              size_t data_len)
{
    if (!map || !state || !data || data_len == 0) return;

    int ri = find_report_index(map, report_id);
    if (ri < 0) {
        ESP_LOGD(TAG, "Unknown report ID %d, ignoring", report_id);
        return;
    }

    const hid_report_layout_t *layout = &map->reports[ri];
    bool has_prev = state->has_prev[ri];
    const uint8_t *prev_data = state->prev_data[ri];
    size_t prev_len = has_prev ? data_len : 0;

    for (int f = 0; f < layout->field_count; f++) {
        const hid_field_t *field = &layout->fields[f];

        // Skip constant (padding) fields
        if (field->flags & HID_FIELD_CONSTANT) continue;

        bool is_button_page = (field->usage_page == 0x09); // Button page
        bool is_variable = (field->flags & HID_FIELD_VARIABLE);

        if (is_button_page && is_variable) {
            // Variable buttons (each bit = one button)
            process_button_field(field, data, data_len, prev_data, prev_len,
                                 has_prev, device_index);
        } else if (!is_variable) {
            // Array field (e.g. keyboard keycodes, consumer control selectors)
            process_array_field(field, data, data_len, prev_data, prev_len,
                                has_prev, device_index);
        } else if (field->usage_page == 0x01 || (field->flags & HID_FIELD_RELATIVE)) {
            // Generic Desktop axes or any relative data
            process_axis_field(field, data, data_len, prev_data, prev_len,
                               has_prev, device_index);
        } else {
            // Other variable fields — treat as buttons if 1-bit, axes otherwise
            if (field->bit_size == 1) {
                process_button_field(field, data, data_len, prev_data, prev_len,
                                     has_prev, device_index);
            } else {
                process_axis_field(field, data, data_len, prev_data, prev_len,
                                   has_prev, device_index);
            }
        }
    }

    // Save current as previous
    size_t copy_len = data_len < HID_EVENT_MAX_PREV_REPORT_SIZE
                    ? data_len : HID_EVENT_MAX_PREV_REPORT_SIZE;
    memcpy(state->prev_data[ri], data, copy_len);
    state->has_prev[ri] = true;
}
