#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * Initialize the display subsystem:
 *  - I2C bus (shared with touch + GPIO expander)
 *  - TCA9554 GPIO expander (LCD + touch reset)
 *  - SPD2010 QSPI LCD (412x412 round)
 *  - SPD2010 touch controller
 *  - LVGL library + display/touch drivers
 *  - Backlight PWM
 *
 * After calling, use ui_update() periodically from main loop.
 */
void ui_init(void);

/**
 * Must be called periodically (~10ms) from main loop to
 * process LVGL timers and touch input.
 */
void ui_update(void);

/**
 * Update the audio TX/RX indicators on screen.
 */
void ui_set_audio_tx(bool transmitting);
void ui_set_audio_rx(bool receiving);

/**
 * Update a BLE device entry in the device list.
 * @param idx       Device slot (0 to 3)
 * @param name      Device name (NULL to clear)
 * @param connected true if connected
 */
void ui_update_ble_device(uint8_t idx, const char *name, bool connected);

/**
 * Refresh all debug UI fields (audio level, VAD, IMU, touch, heap).
 * Call from main loop before ui_update().
 */
void ui_update_debug(void);
