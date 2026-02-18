#pragma once
// ─── Minimal ES8311 codec driver (I2C register interface) ───────────
// Provides init + volume control. Audio data flows via I2S (not here).

#include "board.h"

#if BOARD_AUDIO_TYPE == AUDIO_TYPE_ES8311

#include "esp_err.h"

/**
 * Initialize the ES8311 codec via I2C.
 * Configures for 16kHz 16-bit mono, both ADC (mic) and DAC (speaker) enabled.
 * I2C bus must already be initialized.
 */
esp_err_t es8311_init(void);

/** Set DAC (speaker) volume. 0 = mute, 255 = max. */
esp_err_t es8311_set_voice_volume(uint8_t volume);

/** Set ADC (mic) gain. 0-10 maps to ES8311 PGA gain levels. */
esp_err_t es8311_set_mic_gain(uint8_t gain);

#endif // BOARD_AUDIO_TYPE == AUDIO_TYPE_ES8311
