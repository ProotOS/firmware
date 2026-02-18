#pragma once

#include "sdkconfig.h"

// ─── Board type enums (used by #if comparisons) ─────────────────────
#define LCD_TYPE_SPD2010    1   // QSPI 412x412 round (S3 board)
#define LCD_TYPE_ST7789     2   // SPI 240x280 rect (C6 board)

#define AUDIO_TYPE_RAW_I2S  1   // Separate I2S mic + I2S speaker (S3)
#define AUDIO_TYPE_ES8311   2   // ES8311 codec on single I2S (C6)

// ─── Select board by target ─────────────────────────────────────────
#if CONFIG_IDF_TARGET_ESP32S3
  #include "boards/board_s3_round.h"
#elif CONFIG_IDF_TARGET_ESP32C6
  #include "boards/board_c6_169.h"
#else
  #error "Unsupported IDF_TARGET — add a board header for this chip"
#endif

// ─── Memory helpers ─────────────────────────────────────────────────
#include "esp_heap_caps.h"
#include <stdlib.h>

static inline void *board_malloc_prefer_spiram(size_t size)
{
#if BOARD_HAS_PSRAM
    return heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
#else
    return malloc(size);
#endif
}

static inline void *board_aligned_alloc_prefer_spiram(size_t alignment, size_t size)
{
#if BOARD_HAS_PSRAM
    return heap_caps_aligned_alloc(alignment, size, MALLOC_CAP_SPIRAM);
#else
    return heap_caps_aligned_alloc(alignment, size, MALLOC_CAP_DEFAULT);
#endif
}
