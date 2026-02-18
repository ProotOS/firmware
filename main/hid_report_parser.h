#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define HID_MAX_REPORTS         8
#define HID_MAX_FIELDS          16

// HID field flags (from Input/Output/Feature main items)
#define HID_FIELD_CONSTANT      (1 << 0)  // Bit 0: 0=Data, 1=Constant
#define HID_FIELD_VARIABLE      (1 << 1)  // Bit 1: 0=Array, 1=Variable
#define HID_FIELD_RELATIVE      (1 << 2)  // Bit 2: 0=Absolute, 1=Relative

typedef struct {
    uint16_t usage_page;
    uint16_t usage;           // Single usage (if not a range)
    uint16_t usage_min;       // For button/key ranges
    uint16_t usage_max;
    int32_t  logical_min;
    int32_t  logical_max;
    uint16_t bit_offset;      // Position within the report (excluding report ID byte)
    uint8_t  bit_size;        // Size of each value in bits
    uint8_t  count;           // Number of values (Report Count)
    uint8_t  flags;           // Input item flags
} hid_field_t;

typedef struct {
    uint8_t     report_id;
    uint8_t     field_count;
    hid_field_t fields[HID_MAX_FIELDS];
    uint16_t    total_bits;   // Total size of report data in bits (excluding report ID)
} hid_report_layout_t;

typedef struct {
    uint8_t              report_count;
    hid_report_layout_t  reports[HID_MAX_REPORTS];
    bool                 uses_report_ids;  // Whether reports use report ID prefix
} hid_report_map_t;

/**
 * Parse a raw HID Report Map descriptor into structured field layouts.
 * Only INPUT reports are parsed (since we're reading from the device).
 *
 * @param data     Raw report map bytes (from Report Map characteristic 0x2A4B)
 * @param len      Length of raw data
 * @param out_map  Output: filled with parsed report layouts
 * @return true on success
 */
bool hid_parse_report_map(const uint8_t *data, size_t len, hid_report_map_t *out_map);

/**
 * Find the report layout for a given report ID.
 * If the device doesn't use report IDs, returns the first (only) report.
 *
 * @param map        Parsed report map
 * @param report_id  Report ID to find (0 if no report IDs used)
 * @return pointer to layout, or NULL if not found
 */
const hid_report_layout_t *hid_find_report(const hid_report_map_t *map, uint8_t report_id);

/**
 * Extract a field value from raw report data.
 *
 * @param data       Raw report bytes (after report ID byte if applicable)
 * @param data_len   Length of data in bytes
 * @param field      Field descriptor
 * @param index      Which value within the field (0 to field->count-1)
 * @return extracted value (sign-extended if logical_min is negative)
 */
int32_t hid_extract_field_value(const uint8_t *data, size_t data_len,
                                const hid_field_t *field, uint8_t index);
