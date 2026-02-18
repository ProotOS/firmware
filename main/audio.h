#pragma once

#include <stdbool.h>
#include "board.h"

#if BOARD_HAS_AUDIO

/**
 * Initialize audio subsystem:
 *  - I2S mic input and speaker output
 *  - ESP-NOW broadcast audio
 *  - Espressif AFE (noise suppression + AGC)
 *
 * Requires WiFi to be initialized in STA mode before calling.
 */
void audio_init(void);

/**
 * Enable/disable push-to-talk microphone transmission.
 * When active, processed mic audio is broadcast via ESP-NOW.
 */
void audio_set_ptt(bool active);

/**
 * Returns true if audio has been received from another device
 * within the last ~200ms.
 */
bool audio_is_receiving(void);

/** Play a short beep tone through the speaker. Safe to call from any task. */
void audio_beep(void);

/** Returns latest mic input level in dB (pre-AGC). */
float audio_get_mic_level_db(void);

/** Returns true if AFE detects speech (vs silence/noise). */
bool audio_get_vad_active(void);

#else  // !BOARD_HAS_AUDIO — stubs

static inline void  audio_init(void) {}
static inline void  audio_set_ptt(bool active) { (void)active; }
static inline bool  audio_is_receiving(void) { return false; }
static inline void  audio_beep(void) {}
static inline float audio_get_mic_level_db(void) { return -96.0f; }
static inline bool  audio_get_vad_active(void) { return false; }

#endif // BOARD_HAS_AUDIO
