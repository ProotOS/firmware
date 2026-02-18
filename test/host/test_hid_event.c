#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "hid_report_parser.h"
#include "hid_event.h"

#define TEST(name) do { printf("  [EVENT]  %-50s ", #name); fflush(stdout); } while(0)
#define PASS() do { printf("PASS\n"); } while(0)
#define FAIL(msg) do { printf("FAIL: %s\n", msg); return 1; } while(0)

// ─── Shared test descriptors ────────────────────────────────────────

// Standard 3-button mouse (no report ID)
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

// 6KRO keyboard with report ID 1
static const uint8_t keyboard_descriptor[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x06,       // Usage (Keyboard)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x01,       //   Report ID (1)
    0x05, 0x07,       //   Usage Page (Keyboard/Keypad)
    0x19, 0xE0,       //   Usage Minimum (Left Control)
    0x29, 0xE7,       //   Usage Maximum (Right GUI)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x01,       //   Logical Maximum (1)
    0x75, 0x01,       //   Report Size (1)
    0x95, 0x08,       //   Report Count (8)
    0x81, 0x02,       //   Input (Data, Variable, Absolute)
    0x95, 0x01,       //   Report Count (1)
    0x75, 0x08,       //   Report Size (8)
    0x81, 0x01,       //   Input (Constant)
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

// ─── Event collector ────────────────────────────────────────────────

#define MAX_EVENTS 32
static hid_event_t collected[MAX_EVENTS];
static int event_count = 0;

static void event_collector(const hid_event_t *ev)
{
    if (event_count < MAX_EVENTS) {
        collected[event_count++] = *ev;
    }
}

static void reset_events(void)
{
    event_count = 0;
    memset(collected, 0, sizeof(collected));
}

// Find first event matching type + usage_page + usage
static const hid_event_t *find_event(hid_event_type_t type, uint16_t usage_page, uint16_t usage)
{
    for (int i = 0; i < event_count; i++) {
        if (collected[i].type == type &&
            collected[i].usage_page == usage_page &&
            collected[i].usage == usage)
            return &collected[i];
    }
    return NULL;
}

// ─── Tests ──────────────────────────────────────────────────────────

static int test_mouse_button_press(void)
{
    TEST(mouse_button_press_release);
    hid_report_map_t map;
    hid_event_state_t state;
    memset(&map, 0, sizeof(map));
    memset(&state, 0, sizeof(state));
    hid_parse_report_map(mouse_descriptor, sizeof(mouse_descriptor), &map);

    hid_event_set_callback(event_collector);

    // First report: button 1 pressed, no movement
    // Byte 0: buttons = 0x01 (bit0=1), Byte 1: X=0, Byte 2: Y=0
    uint8_t report1[] = { 0x01, 0x00, 0x00 };
    reset_events();
    hid_event_process_report(&map, &state, 0, 0, report1, sizeof(report1));

    // First report with no previous — button 1 goes from 0→1 → BUTTON_DOWN
    if (!find_event(HID_EVENT_BUTTON_DOWN, 0x09, 1))
        FAIL("expected BUTTON_DOWN for button 1");
    // Button 2 and 3 are 0→0, no events expected
    if (find_event(HID_EVENT_BUTTON_DOWN, 0x09, 2))
        FAIL("button 2 should not fire");
    if (find_event(HID_EVENT_BUTTON_DOWN, 0x09, 3))
        FAIL("button 3 should not fire");

    // Second report: release button 1
    uint8_t report2[] = { 0x00, 0x00, 0x00 };
    reset_events();
    hid_event_process_report(&map, &state, 0, 0, report2, sizeof(report2));

    if (!find_event(HID_EVENT_BUTTON_UP, 0x09, 1))
        FAIL("expected BUTTON_UP for button 1");
    if (find_event(HID_EVENT_BUTTON_DOWN, 0x09, 1))
        FAIL("button 1 should not fire DOWN on release");

    PASS();
    return 0;
}

static int test_mouse_axis_change(void)
{
    TEST(mouse_axis_change);
    hid_report_map_t map;
    hid_event_state_t state;
    memset(&map, 0, sizeof(map));
    memset(&state, 0, sizeof(state));
    hid_parse_report_map(mouse_descriptor, sizeof(mouse_descriptor), &map);

    hid_event_set_callback(event_collector);

    // First report: no buttons, X=10, Y=-5
    // X=10 → 0x0A, Y=-5 → 0xFB (signed 8-bit)
    uint8_t report1[] = { 0x00, 0x0A, 0xFB };
    reset_events();
    hid_event_process_report(&map, &state, 0, 0, report1, sizeof(report1));

    // X axis: usage 0x30 on page 0x01
    const hid_event_t *xev = find_event(HID_EVENT_AXIS_CHANGE, 0x01, 0x30);
    if (!xev) FAIL("expected AXIS_CHANGE for X");
    if (xev->value != 10) FAIL("X value should be 10");

    // Y axis: usage 0x31 on page 0x01
    const hid_event_t *yev = find_event(HID_EVENT_AXIS_CHANGE, 0x01, 0x31);
    if (!yev) FAIL("expected AXIS_CHANGE for Y");
    if (yev->value != -5) FAIL("Y value should be -5");

    // Second report: same X, different Y
    uint8_t report2[] = { 0x00, 0x0A, 0x03 };
    reset_events();
    hid_event_process_report(&map, &state, 0, 0, report2, sizeof(report2));

    // X unchanged → no event
    if (find_event(HID_EVENT_AXIS_CHANGE, 0x01, 0x30))
        FAIL("X unchanged, should not fire");
    // Y changed → event
    const hid_event_t *yev2 = find_event(HID_EVENT_AXIS_CHANGE, 0x01, 0x31);
    if (!yev2) FAIL("expected AXIS_CHANGE for Y (changed)");
    if (yev2->value != 3) FAIL("Y value should be 3");

    PASS();
    return 0;
}

static int test_no_callback(void)
{
    TEST(no_callback_no_crash);
    hid_report_map_t map;
    hid_event_state_t state;
    memset(&map, 0, sizeof(map));
    memset(&state, 0, sizeof(state));
    hid_parse_report_map(mouse_descriptor, sizeof(mouse_descriptor), &map);

    // Set callback to NULL — should not crash
    hid_event_set_callback(NULL);
    uint8_t report[] = { 0x07, 0x0A, 0x05 };
    hid_event_process_report(&map, &state, 0, 0, report, sizeof(report));

    // Restore callback for other tests
    hid_event_set_callback(event_collector);
    PASS();
    return 0;
}

static int test_keyboard_array_field(void)
{
    TEST(keyboard_array_key_press_release);
    hid_report_map_t map;
    hid_event_state_t state;
    memset(&map, 0, sizeof(map));
    memset(&state, 0, sizeof(state));
    hid_parse_report_map(keyboard_descriptor, sizeof(keyboard_descriptor), &map);

    hid_event_set_callback(event_collector);

    // Report layout (after report ID byte): [modifiers:1][reserved:1][keys:6]
    // Press 'A' (usage 0x04) — no modifiers
    uint8_t report1[] = { 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00 };
    reset_events();
    hid_event_process_report(&map, &state, 0, 1, report1, sizeof(report1));

    // Key 0x04 should appear as BUTTON_DOWN (array fields emit button events)
    if (!find_event(HID_EVENT_BUTTON_DOWN, 0x07, 0x04))
        FAIL("expected key down for 'A' (0x04)");

    // Now press 'A' + 'B' (0x05)
    uint8_t report2[] = { 0x00, 0x00, 0x04, 0x05, 0x00, 0x00, 0x00, 0x00 };
    reset_events();
    hid_event_process_report(&map, &state, 0, 1, report2, sizeof(report2));

    // 'A' still held — no event for it. 'B' newly pressed.
    if (find_event(HID_EVENT_BUTTON_DOWN, 0x07, 0x04))
        FAIL("'A' still held, should not fire again");
    if (!find_event(HID_EVENT_BUTTON_DOWN, 0x07, 0x05))
        FAIL("expected key down for 'B' (0x05)");

    // Release 'A', keep 'B'
    uint8_t report3[] = { 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00 };
    reset_events();
    hid_event_process_report(&map, &state, 0, 1, report3, sizeof(report3));

    if (!find_event(HID_EVENT_BUTTON_UP, 0x07, 0x04))
        FAIL("expected key up for 'A' (0x04)");
    if (find_event(HID_EVENT_BUTTON_UP, 0x07, 0x05))
        FAIL("'B' still held, should not fire up");

    // Release all
    uint8_t report4[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    reset_events();
    hid_event_process_report(&map, &state, 0, 1, report4, sizeof(report4));

    if (!find_event(HID_EVENT_BUTTON_UP, 0x07, 0x05))
        FAIL("expected key up for 'B' (0x05)");

    PASS();
    return 0;
}

static int test_modifier_keys(void)
{
    TEST(keyboard_modifier_keys);
    hid_report_map_t map;
    hid_event_state_t state;
    memset(&map, 0, sizeof(map));
    memset(&state, 0, sizeof(state));
    hid_parse_report_map(keyboard_descriptor, sizeof(keyboard_descriptor), &map);

    hid_event_set_callback(event_collector);

    // Press Left Ctrl (bit 0 of modifiers byte)
    // Modifiers field: usage_page 0x07, usage_min 0xE0
    // bit0 = Left Ctrl (0xE0), bit1 = Left Shift (0xE1), etc.
    uint8_t report1[] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    reset_events();
    hid_event_process_report(&map, &state, 0, 1, report1, sizeof(report1));

    // Modifier keys are variable buttons on usage page 0x07
    // Left Ctrl = usage 0xE0
    // The modifier field is processed as buttons (1-bit variable) but on page 0x07
    // The code treats page 0x07 variable 1-bit fields via the "other variable" → bit_size==1 → button path
    const hid_event_t *ctrl = find_event(HID_EVENT_BUTTON_DOWN, 0x07, 0xE0);
    if (!ctrl) FAIL("expected BUTTON_DOWN for Left Ctrl (0xE0)");

    // Press Left Ctrl + Left Shift
    uint8_t report2[] = { 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    reset_events();
    hid_event_process_report(&map, &state, 0, 1, report2, sizeof(report2));

    // Ctrl still held → no event. Shift newly pressed.
    if (find_event(HID_EVENT_BUTTON_DOWN, 0x07, 0xE0))
        FAIL("Ctrl still held, should not fire again");
    if (!find_event(HID_EVENT_BUTTON_DOWN, 0x07, 0xE1))
        FAIL("expected BUTTON_DOWN for Left Shift (0xE1)");

    // Release Ctrl, keep Shift
    uint8_t report3[] = { 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    reset_events();
    hid_event_process_report(&map, &state, 0, 1, report3, sizeof(report3));

    if (!find_event(HID_EVENT_BUTTON_UP, 0x07, 0xE0))
        FAIL("expected BUTTON_UP for Left Ctrl");

    PASS();
    return 0;
}

static int test_device_index_passthrough(void)
{
    TEST(device_index_passthrough);
    hid_report_map_t map;
    hid_event_state_t state;
    memset(&map, 0, sizeof(map));
    memset(&state, 0, sizeof(state));
    hid_parse_report_map(mouse_descriptor, sizeof(mouse_descriptor), &map);

    hid_event_set_callback(event_collector);

    uint8_t report[] = { 0x01, 0x00, 0x00 };
    reset_events();
    hid_event_process_report(&map, &state, 3, 0, report, sizeof(report));

    if (event_count == 0) FAIL("expected at least one event");
    if (collected[0].device_index != 3) FAIL("device_index should be 3");

    PASS();
    return 0;
}

static int test_null_inputs(void)
{
    TEST(null_inputs_no_crash);
    hid_report_map_t map;
    hid_event_state_t state;
    memset(&map, 0, sizeof(map));
    memset(&state, 0, sizeof(state));

    hid_event_set_callback(event_collector);
    reset_events();

    // All NULL/zero cases — should not crash
    hid_event_process_report(NULL, &state, 0, 0, (uint8_t[]){0}, 1);
    hid_event_process_report(&map, NULL, 0, 0, (uint8_t[]){0}, 1);
    hid_event_process_report(&map, &state, 0, 0, NULL, 1);
    hid_event_process_report(&map, &state, 0, 0, (uint8_t[]){0}, 0);

    if (event_count != 0) FAIL("no events should be emitted for null inputs");

    PASS();
    return 0;
}

static int test_unknown_report_id(void)
{
    TEST(unknown_report_id_ignored);
    hid_report_map_t map;
    hid_event_state_t state;
    memset(&map, 0, sizeof(map));
    memset(&state, 0, sizeof(state));
    hid_parse_report_map(keyboard_descriptor, sizeof(keyboard_descriptor), &map);

    hid_event_set_callback(event_collector);
    reset_events();

    // Report ID 99 doesn't exist in this descriptor
    uint8_t report[] = { 0x01, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00 };
    hid_event_process_report(&map, &state, 0, 99, report, sizeof(report));

    if (event_count != 0) FAIL("unknown report ID should produce no events");

    PASS();
    return 0;
}

// ─── Runner ─────────────────────────────────────────────────────────

int test_hid_event_run(void)
{
    int failures = 0;
    printf("[HID Event System Tests]\n");

    failures += test_mouse_button_press();
    failures += test_mouse_axis_change();
    failures += test_no_callback();
    failures += test_keyboard_array_field();
    failures += test_modifier_keys();
    failures += test_device_index_passthrough();
    failures += test_null_inputs();
    failures += test_unknown_report_id();

    printf("\n");
    return failures;
}
