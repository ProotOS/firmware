#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "hid_event.h"

// Maximum number of simultaneous BLE HID connections
#define BLE_HID_MAX_DEVICES 4

/**
 * Callback for device connect/disconnect status changes.
 * @param idx       Device slot index (0 to BLE_HID_MAX_DEVICES-1)
 * @param name      Device name (or empty string on disconnect)
 * @param connected true if connected, false if disconnected
 */
typedef void (*ble_hid_status_callback_t)(uint8_t idx, const char *name, bool connected);

/**
 * Initialize the BLE stack, security, and start scanning for HID devices.
 * Automatically connects to any named device advertising the HID service.
 * Reconnects on disconnect, and scans for additional devices up to
 * BLE_HID_MAX_DEVICES.
 *
 * @param callback  Function called for every decoded HID event
 *                  (button press/release, axis change)
 */
void ble_hid_init(hid_event_callback_t callback);

/**
 * Register a callback for device connect/disconnect notifications.
 */
void ble_hid_set_status_callback(ble_hid_status_callback_t cb);

/**
 * Get the name of a connected device by slot index.
 * Returns NULL if slot is empty.
 */
const char *ble_hid_get_device_name(uint8_t idx);

/**
 * Check if a device slot is connected.
 */
bool ble_hid_is_connected(uint8_t idx);
