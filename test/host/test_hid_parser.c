#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "hid_report_parser.h"

#define TEST(name) do { printf("  [PARSER] %-50s ", #name); fflush(stdout); } while(0)
#define PASS() do { printf("PASS\n"); } while(0)
#define FAIL(msg) do { printf("FAIL: %s\n", msg); return 1; } while(0)

// ─── Real HID descriptors ───────────────────────────────────────────

// Standard 3-button mouse with X/Y axes (no report ID)
static const uint8_t mouse_descriptor[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x02,       // Usage (Mouse)
    0xA1, 0x01,       // Collection (Application)
    0x09, 0x01,       //   Usage (Pointer)
    0xA1, 0x00,       //   Collection (Physical)
    0x05, 0x09,       //     Usage Page (Button)
    0x19, 0x01,       //     Usage Minimum (1)
    0x29, 0x03,       //     Usage Maximum (3)
    0x15, 0x00,       //     Logical Minimum (0)
    0x25, 0x01,       //     Logical Maximum (1)
    0x95, 0x03,       //     Report Count (3)
    0x75, 0x01,       //     Report Size (1)
    0x81, 0x02,       //     Input (Data, Variable, Absolute) -- 3 buttons
    0x95, 0x01,       //     Report Count (1)
    0x75, 0x05,       //     Report Size (5)
    0x81, 0x01,       //     Input (Constant) -- 5-bit padding
    0x05, 0x01,       //     Usage Page (Generic Desktop)
    0x09, 0x30,       //     Usage (X)
    0x09, 0x31,       //     Usage (Y)
    0x15, 0x81,       //     Logical Minimum (-127)
    0x25, 0x7F,       //     Logical Maximum (127)
    0x75, 0x08,       //     Report Size (8)
    0x95, 0x02,       //     Report Count (2)
    0x81, 0x06,       //     Input (Data, Variable, Relative) -- X,Y
    0xC0,             //   End Collection
    0xC0,             // End Collection
};

// Keyboard with report ID 1 (6KRO)
static const uint8_t keyboard_descriptor[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x06,       // Usage (Keyboard)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x01,       //   Report ID (1)
    // Modifier keys (8 bits)
    0x05, 0x07,       //   Usage Page (Keyboard/Keypad)
    0x19, 0xE0,       //   Usage Minimum (Left Control)
    0x29, 0xE7,       //   Usage Maximum (Right GUI)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x01,       //   Logical Maximum (1)
    0x75, 0x01,       //   Report Size (1)
    0x95, 0x08,       //   Report Count (8)
    0x81, 0x02,       //   Input (Data, Variable, Absolute)
    // Reserved byte
    0x95, 0x01,       //   Report Count (1)
    0x75, 0x08,       //   Report Size (8)
    0x81, 0x01,       //   Input (Constant)
    // Keycodes (6 array entries)
    0x95, 0x06,       //   Report Count (6)
    0x75, 0x08,       //   Report Size (8)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x65,       //   Logical Maximum (101)
    0x05, 0x07,       //   Usage Page (Keyboard/Keypad)
    0x19, 0x00,       //   Usage Minimum (0)
    0x29, 0x65,       //   Usage Maximum (101)
    0x81, 0x00,       //   Input (Data, Array)
    0xC0,             // End Collection
};

// AB Shutter3-style: consumer control + keyboard in separate reports
static const uint8_t multi_report_descriptor[] = {
    // Consumer Control (Report ID 1)
    0x05, 0x0C,       // Usage Page (Consumer)
    0x09, 0x01,       // Usage (Consumer Control)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x01,       //   Report ID (1)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x01,       //   Logical Maximum (1)
    0x75, 0x01,       //   Report Size (1)
    0x95, 0x02,       //   Report Count (2)
    0x09, 0xE9,       //   Usage (Volume Up)
    0x09, 0xEA,       //   Usage (Volume Down)
    0x81, 0x02,       //   Input (Data, Variable, Absolute)
    0x95, 0x06,       //   Report Count (6)
    0x81, 0x01,       //   Input (Constant) -- padding
    0xC0,             // End Collection
    // Keyboard (Report ID 2)
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x06,       // Usage (Keyboard)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x02,       //   Report ID (2)
    0x05, 0x07,       //   Usage Page (Keyboard)
    0x19, 0xE0,       //   Usage Minimum (Left Ctrl)
    0x29, 0xE7,       //   Usage Maximum (Right GUI)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x01,       //   Logical Maximum (1)
    0x75, 0x01,       //   Report Size (1)
    0x95, 0x08,       //   Report Count (8)
    0x81, 0x02,       //   Input (Data, Variable, Absolute)
    0xC0,             // End Collection
};

// ─── Tests ──────────────────────────────────────────────────────────

static int test_parse_mouse(void)
{
    TEST(parse_mouse_descriptor);
    hid_report_map_t map;
    memset(&map, 0, sizeof(map));

    if (!hid_parse_report_map(mouse_descriptor, sizeof(mouse_descriptor), &map))
        FAIL("parse returned false");

    if (map.uses_report_ids)
        FAIL("mouse should not use report IDs");

    if (map.report_count != 1)
        FAIL("expected 1 report");

    hid_report_layout_t *r = &map.reports[0];
    if (r->report_id != 0)
        FAIL("report ID should be 0");

    // Should have 3 fields: buttons, padding, X+Y
    if (r->field_count != 3)
        FAIL("expected 3 fields (buttons, padding, axes)");

    // Field 0: 3 buttons, 1 bit each, variable
    hid_field_t *f = &r->fields[0];
    if (f->usage_page != 0x09)
        FAIL("buttons usage_page should be 0x09");
    if (f->count != 3)
        FAIL("button count should be 3");
    if (f->bit_size != 1)
        FAIL("button bit_size should be 1");
    if (f->bit_offset != 0)
        FAIL("buttons should start at bit 0");
    if (!(f->flags & HID_FIELD_VARIABLE))
        FAIL("buttons should be variable");
    if (f->usage_min != 1 || f->usage_max != 3)
        FAIL("button usage range should be 1-3");

    // Field 1: 5-bit padding (constant)
    f = &r->fields[1];
    if (!(f->flags & HID_FIELD_CONSTANT))
        FAIL("padding should be constant");
    if (f->bit_offset != 3)
        FAIL("padding should start at bit 3");

    // Field 2: X,Y axes, 8 bits each, relative
    f = &r->fields[2];
    if (f->usage_page != 0x01)
        FAIL("axes usage_page should be 0x01");
    if (f->count != 2)
        FAIL("axis count should be 2");
    if (f->bit_size != 8)
        FAIL("axis bit_size should be 8");
    if (f->bit_offset != 8)
        FAIL("axes should start at bit 8");
    if (!(f->flags & HID_FIELD_RELATIVE))
        FAIL("axes should be relative");
    if (f->logical_min != -127 || f->logical_max != 127)
        FAIL("axis logical range wrong");

    // Total bits: 3 + 5 + 16 = 24 (3 bytes)
    if (r->total_bits != 24)
        FAIL("total bits should be 24");

    PASS();
    return 0;
}

static int test_parse_keyboard(void)
{
    TEST(parse_keyboard_descriptor);
    hid_report_map_t map;
    memset(&map, 0, sizeof(map));

    if (!hid_parse_report_map(keyboard_descriptor, sizeof(keyboard_descriptor), &map))
        FAIL("parse returned false");

    if (!map.uses_report_ids)
        FAIL("keyboard should use report IDs");

    if (map.report_count != 1)
        FAIL("expected 1 report");

    hid_report_layout_t *r = &map.reports[0];
    if (r->report_id != 1)
        FAIL("report ID should be 1");

    if (r->field_count != 3)
        FAIL("expected 3 fields (modifiers, reserved, keycodes)");

    // Field 0: modifier keys (8x1-bit, variable)
    hid_field_t *f = &r->fields[0];
    if (f->usage_page != 0x07)
        FAIL("modifiers usage_page should be 0x07");
    if (f->count != 8 || f->bit_size != 1)
        FAIL("modifiers: 8 x 1 bit");
    if (!(f->flags & HID_FIELD_VARIABLE))
        FAIL("modifiers should be variable");

    // Field 1: reserved (constant)
    f = &r->fields[1];
    if (!(f->flags & HID_FIELD_CONSTANT))
        FAIL("reserved should be constant");

    // Field 2: keycodes (6x8-bit array)
    f = &r->fields[2];
    if (f->usage_page != 0x07)
        FAIL("keycodes usage_page should be 0x07");
    if (f->count != 6 || f->bit_size != 8)
        FAIL("keycodes: 6 x 8 bits");
    if (f->flags & HID_FIELD_VARIABLE)
        FAIL("keycodes should be array (not variable)");

    PASS();
    return 0;
}

static int test_parse_multi_report(void)
{
    TEST(parse_multi_report_descriptor);
    hid_report_map_t map;
    memset(&map, 0, sizeof(map));

    if (!hid_parse_report_map(multi_report_descriptor, sizeof(multi_report_descriptor), &map))
        FAIL("parse returned false");

    if (!map.uses_report_ids)
        FAIL("should use report IDs");

    if (map.report_count != 2)
        FAIL("expected 2 reports");

    // Report ID 1: consumer control
    const hid_report_layout_t *r1 = hid_find_report(&map, 1);
    if (!r1) FAIL("report ID 1 not found");
    if (r1->field_count < 1)
        FAIL("consumer report should have fields");
    if (r1->fields[0].usage_page != 0x0C)
        FAIL("report 1 should be Consumer page");

    // Report ID 2: keyboard
    const hid_report_layout_t *r2 = hid_find_report(&map, 2);
    if (!r2) FAIL("report ID 2 not found");
    if (r2->fields[0].usage_page != 0x07)
        FAIL("report 2 should be Keyboard page");

    // Non-existent report
    if (hid_find_report(&map, 99) != NULL)
        FAIL("report 99 should not exist");

    PASS();
    return 0;
}

static int test_extract_field_value(void)
{
    TEST(extract_field_value);

    // Mouse report: [buttons=0x05, X=-10, Y=20]
    // buttons byte: bit0=1 (btn1), bit1=0 (btn2), bit2=1 (btn3) = 0x05
    uint8_t data[] = { 0x05, 0xF6, 0x14 }; // 0xF6 = -10 signed, 0x14 = 20

    hid_report_map_t map;
    memset(&map, 0, sizeof(map));
    hid_parse_report_map(mouse_descriptor, sizeof(mouse_descriptor), &map);

    const hid_report_layout_t *r = &map.reports[0];

    // Button field: 3 x 1-bit at offset 0
    hid_field_t *btn = &map.reports[0].fields[0];
    int32_t b1 = hid_extract_field_value(data, sizeof(data), btn, 0);
    int32_t b2 = hid_extract_field_value(data, sizeof(data), btn, 1);
    int32_t b3 = hid_extract_field_value(data, sizeof(data), btn, 2);
    if (b1 != 1) FAIL("button 1 should be 1");
    if (b2 != 0) FAIL("button 2 should be 0");
    if (b3 != 1) FAIL("button 3 should be 1");

    // Axis field: 2 x 8-bit at offset 8
    hid_field_t *axes = &map.reports[0].fields[2];
    int32_t x = hid_extract_field_value(data, sizeof(data), axes, 0);
    int32_t y = hid_extract_field_value(data, sizeof(data), axes, 1);
    if (x != -10) FAIL("X should be -10");
    if (y != 20) FAIL("Y should be 20");

    PASS();
    return 0;
}

static int test_empty_descriptor(void)
{
    TEST(parse_empty_descriptor);
    hid_report_map_t map;
    memset(&map, 0, sizeof(map));

    // NULL/0 should return false (validation)
    bool ok = hid_parse_report_map(NULL, 0, &map);
    if (ok) FAIL("should return false on NULL data");

    // Valid pointer but zero-length should also return false
    uint8_t dummy = 0;
    ok = hid_parse_report_map(&dummy, 0, &map);
    if (ok) FAIL("should return false on zero length");

    PASS();
    return 0;
}

// ─── Runner ─────────────────────────────────────────────────────────

int test_hid_parser_run(void)
{
    int failures = 0;
    printf("[HID Report Parser Tests]\n");

    failures += test_parse_mouse();
    failures += test_parse_keyboard();
    failures += test_parse_multi_report();
    failures += test_extract_field_value();
    failures += test_empty_descriptor();

    printf("\n");
    return failures;
}
