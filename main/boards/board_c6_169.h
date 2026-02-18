#pragma once
// ─── Waveshare ESP32-C6-Touch-LCD-1.69 ──────────────────────────────
// 240x280 rect SPI display (ST7789V2), ES8311 audio codec,
// QMI8658 IMU, PCF85063 RTC, 16MB flash, no PSRAM

// ─── Feature flags ──────────────────────────────────────────────────
#define BOARD_HAS_PSRAM             0
#define BOARD_HAS_TOUCH             0   // Optional CST816S variant
#define BOARD_HAS_GPIO_EXPANDER     0
#define BOARD_HAS_IMU               1   // QMI8658 (addr 0x6B)
#define BOARD_HAS_AUDIO             1   // ES8311 codec
#define BOARD_HAS_AFE               0   // No PSRAM, no AFE — raw passthrough

// ─── Display ────────────────────────────────────────────────────────
#define BOARD_LCD_TYPE              LCD_TYPE_ST7789
#define BOARD_LCD_WIDTH             240
#define BOARD_LCD_HEIGHT            280
#define BOARD_LCD_COLOR_BITS        16

// SPI pins (ST7789V2) — from Waveshare reference
#define BOARD_LCD_SPI_SCK           1
#define BOARD_LCD_SPI_MOSI          2
#define BOARD_LCD_SPI_CS            5
#define BOARD_LCD_PIN_DC            3
#define BOARD_LCD_PIN_RST           4
#define BOARD_LCD_PIN_BL            6

// ─── I2C ────────────────────────────────────────────────────────────
#define BOARD_I2C_SCL               7
#define BOARD_I2C_SDA               8
#define BOARD_I2C_NUM               0
#define BOARD_I2C_FREQ_HZ           400000

// ─── Audio (ES8311 codec) ───────────────────────────────────────────
#define BOARD_AUDIO_TYPE            AUDIO_TYPE_ES8311

// I2S pins (single I2S duplex via ES8311)
#define BOARD_I2S_NUM               0
#define BOARD_I2S_MCLK_PIN          19
#define BOARD_I2S_BCLK_PIN          20
#define BOARD_I2S_WS_PIN            22
#define BOARD_I2S_DOUT_PIN          23
#define BOARD_I2S_DIN_PIN           21

// ES8311 I2C address (from reference: ES8311_CODEC_DEFAULT_ADDR = 0x18)
#define BOARD_ES8311_ADDR           0x18
