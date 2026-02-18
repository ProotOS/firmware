#include "hid_report_parser.h"
#include <string.h>
#include "esp_log.h"

static const char *TAG = "HID_Parser";

// HID item types (bits 3:2 of prefix byte)
#define ITEM_TYPE_MAIN    0
#define ITEM_TYPE_GLOBAL  1
#define ITEM_TYPE_LOCAL   2

// Main item tags
#define MAIN_INPUT          0x08
#define MAIN_OUTPUT         0x09
#define MAIN_FEATURE        0x0B
#define MAIN_COLLECTION     0x0A
#define MAIN_END_COLLECTION 0x0C

// Global item tags
#define GLOBAL_USAGE_PAGE    0x00
#define GLOBAL_LOGICAL_MIN   0x01
#define GLOBAL_LOGICAL_MAX   0x02
#define GLOBAL_PHYSICAL_MIN  0x03
#define GLOBAL_PHYSICAL_MAX  0x04
#define GLOBAL_REPORT_SIZE   0x07
#define GLOBAL_REPORT_ID     0x08
#define GLOBAL_REPORT_COUNT  0x09

// Local item tags
#define LOCAL_USAGE          0x00
#define LOCAL_USAGE_MIN      0x01
#define LOCAL_USAGE_MAX      0x02

// Parser state for global items (persists across Main items)
typedef struct {
    uint16_t usage_page;
    int32_t  logical_min;
    int32_t  logical_max;
    uint8_t  report_size;
    uint8_t  report_count;
    uint8_t  report_id;
} hid_global_state_t;

// Parser state for local items (resets after each Main item)
#define MAX_LOCAL_USAGES 16
typedef struct {
    uint16_t usages[MAX_LOCAL_USAGES];
    uint8_t  usage_count;
    uint16_t usage_min;
    uint16_t usage_max;
    bool     has_usage_range;
} hid_local_state_t;

static int32_t parse_item_signed(const uint8_t *data, uint8_t size)
{
    switch (size) {
    case 1: return (int8_t)data[0];
    case 2: return (int16_t)(data[0] | (data[1] << 8));
    case 4: return (int32_t)(data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
    default: return 0;
    }
}

static uint32_t parse_item_unsigned(const uint8_t *data, uint8_t size)
{
    switch (size) {
    case 1: return data[0];
    case 2: return data[0] | (data[1] << 8);
    case 4: return data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
    default: return 0;
    }
}

static hid_report_layout_t *find_or_create_report(hid_report_map_t *map, uint8_t report_id)
{
    for (int i = 0; i < map->report_count; i++) {
        if (map->reports[i].report_id == report_id) {
            return &map->reports[i];
        }
    }
    if (map->report_count >= HID_MAX_REPORTS) {
        return NULL;
    }
    hid_report_layout_t *r = &map->reports[map->report_count++];
    memset(r, 0, sizeof(*r));
    r->report_id = report_id;
    return r;
}

static void handle_input_item(hid_report_map_t *map,
                              const hid_global_state_t *global,
                              const hid_local_state_t *local,
                              uint8_t flags)
{
    hid_report_layout_t *report = find_or_create_report(map, global->report_id);
    if (!report) {
        ESP_LOGW(TAG, "Too many reports, skipping");
        return;
    }

    if (report->field_count >= HID_MAX_FIELDS) {
        ESP_LOGW(TAG, "Too many fields in report %d, skipping", global->report_id);
        // Still advance bit offset
        report->total_bits += global->report_size * global->report_count;
        return;
    }

    hid_field_t *field = &report->fields[report->field_count++];
    field->usage_page = global->usage_page;
    field->logical_min = global->logical_min;
    field->logical_max = global->logical_max;
    field->bit_offset = report->total_bits;
    field->bit_size = global->report_size;
    field->count = global->report_count;
    field->flags = flags & 0x07; // We care about Constant, Variable, Relative

    if (local->has_usage_range) {
        field->usage_min = local->usage_min;
        field->usage_max = local->usage_max;
        field->usage = local->usage_min;
    } else if (local->usage_count > 0) {
        field->usage = local->usages[0];
        field->usage_min = local->usages[0];
        field->usage_max = (local->usage_count > 1)
            ? local->usages[local->usage_count - 1]
            : local->usages[0];
    } else {
        field->usage = 0;
        field->usage_min = 0;
        field->usage_max = 0;
    }

    report->total_bits += global->report_size * global->report_count;

    ESP_LOGD(TAG, "  Field: page=0x%04x usage=0x%04x-0x%04x size=%d count=%d offset=%d flags=0x%02x",
             field->usage_page, field->usage_min, field->usage_max,
             field->bit_size, field->count, field->bit_offset, field->flags);
}

bool hid_parse_report_map(const uint8_t *data, size_t len, hid_report_map_t *out_map)
{
    if (!data || !len || !out_map) return false;

    memset(out_map, 0, sizeof(*out_map));
    out_map->uses_report_ids = false;

    hid_global_state_t global = {0};
    hid_local_state_t local = {0};

    size_t pos = 0;
    while (pos < len) {
        uint8_t prefix = data[pos];

        // Long item check (prefix == 0xFE)
        if (prefix == 0xFE) {
            if (pos + 2 >= len) break;
            uint8_t data_size = data[pos + 1];
            pos += 3 + data_size; // skip long item
            continue;
        }

        uint8_t item_size = prefix & 0x03;
        if (item_size == 3) item_size = 4; // size code 3 means 4 bytes
        uint8_t item_type = (prefix >> 2) & 0x03;
        uint8_t item_tag = (prefix >> 4) & 0x0F;

        if (pos + 1 + item_size > len) break;
        const uint8_t *item_data = &data[pos + 1];

        switch (item_type) {
        case ITEM_TYPE_MAIN:
            switch (item_tag) {
            case MAIN_INPUT:
                handle_input_item(out_map, &global, &local,
                                  item_size > 0 ? (uint8_t)parse_item_unsigned(item_data, item_size) : 0);
                break;
            case MAIN_OUTPUT:
            case MAIN_FEATURE:
                // Skip output/feature — we only care about input reports
                break;
            case MAIN_COLLECTION:
            case MAIN_END_COLLECTION:
                break;
            }
            // Reset local state after each main item
            memset(&local, 0, sizeof(local));
            break;

        case ITEM_TYPE_GLOBAL:
            switch (item_tag) {
            case GLOBAL_USAGE_PAGE:
                global.usage_page = (uint16_t)parse_item_unsigned(item_data, item_size);
                break;
            case GLOBAL_LOGICAL_MIN:
                global.logical_min = parse_item_signed(item_data, item_size);
                break;
            case GLOBAL_LOGICAL_MAX:
                global.logical_max = parse_item_signed(item_data, item_size);
                break;
            case GLOBAL_REPORT_SIZE:
                global.report_size = (uint8_t)parse_item_unsigned(item_data, item_size);
                break;
            case GLOBAL_REPORT_ID:
                global.report_id = (uint8_t)parse_item_unsigned(item_data, item_size);
                out_map->uses_report_ids = true;
                break;
            case GLOBAL_REPORT_COUNT:
                global.report_count = (uint8_t)parse_item_unsigned(item_data, item_size);
                break;
            }
            break;

        case ITEM_TYPE_LOCAL:
            switch (item_tag) {
            case LOCAL_USAGE:
                if (local.usage_count < MAX_LOCAL_USAGES) {
                    uint16_t usage = (uint16_t)parse_item_unsigned(item_data, item_size);
                    local.usages[local.usage_count++] = usage;
                }
                break;
            case LOCAL_USAGE_MIN:
                local.usage_min = (uint16_t)parse_item_unsigned(item_data, item_size);
                local.has_usage_range = true;
                break;
            case LOCAL_USAGE_MAX:
                local.usage_max = (uint16_t)parse_item_unsigned(item_data, item_size);
                local.has_usage_range = true;
                break;
            }
            break;
        }

        pos += 1 + item_size;
    }

    ESP_LOGI(TAG, "Parsed report map: %d input reports, uses_report_ids=%d",
             out_map->report_count, out_map->uses_report_ids);
    for (int i = 0; i < out_map->report_count; i++) {
        hid_report_layout_t *r = &out_map->reports[i];
        ESP_LOGI(TAG, "  Report ID %d: %d fields, %d bits (%d bytes)",
                 r->report_id, r->field_count, r->total_bits, (r->total_bits + 7) / 8);
    }

    return out_map->report_count > 0;
}

const hid_report_layout_t *hid_find_report(const hid_report_map_t *map, uint8_t report_id)
{
    if (!map) return NULL;
    for (int i = 0; i < map->report_count; i++) {
        if (map->reports[i].report_id == report_id) {
            return &map->reports[i];
        }
    }
    return NULL;
}

int32_t hid_extract_field_value(const uint8_t *data, size_t data_len,
                                const hid_field_t *field, uint8_t index)
{
    if (!data || !field || index >= field->count) return 0;

    uint16_t bit_pos = field->bit_offset + (index * field->bit_size);
    uint16_t byte_pos = bit_pos / 8;
    uint8_t bit_off = bit_pos % 8;
    uint8_t bits_left = field->bit_size;

    if (byte_pos >= data_len) return 0;

    // Extract up to 32 bits across byte boundaries
    uint32_t raw = 0;
    uint8_t shift = 0;
    while (bits_left > 0 && byte_pos < data_len) {
        uint8_t available = 8 - bit_off;
        uint8_t take = (bits_left < available) ? bits_left : available;
        uint8_t mask = (uint8_t)((1u << take) - 1);
        raw |= (uint32_t)((data[byte_pos] >> bit_off) & mask) << shift;
        shift += take;
        bits_left -= take;
        byte_pos++;
        bit_off = 0;
    }

    // Sign extend if logical_min is negative
    if (field->logical_min < 0 && field->bit_size < 32) {
        uint32_t sign_bit = 1u << (field->bit_size - 1);
        if (raw & sign_bit) {
            raw |= ~((1u << field->bit_size) - 1);
        }
    }

    return (int32_t)raw;
}
