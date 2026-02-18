#pragma once
// ─── Waveshare ESP32-S3-Touch-LCD-1.46B ─────────────────────────────
// 412x412 round QSPI display, SPD2010 touch, QMI8658 IMU,
// I2S mic + PCM5101 DAC speaker, TCA9554PWR GPIO expander

// ─── Feature flags ──────────────────────────────────────────────────
#define BOARD_HAS_PSRAM             1
#define BOARD_HAS_TOUCH             1
#define BOARD_HAS_GPIO_EXPANDER     1   // TCA9554PWR
#define BOARD_HAS_IMU               1   // QMI8658
#define BOARD_HAS_AUDIO             1
#define BOARD_HAS_AFE               1   // ESP-SR AFE (needs PSRAM)

// ─── Display ────────────────────────────────────────────────────────
#define BOARD_LCD_TYPE              LCD_TYPE_SPD2010
#define BOARD_LCD_WIDTH             412
#define BOARD_LCD_HEIGHT            412
#define BOARD_LCD_COLOR_BITS        16

// QSPI pins
#define BOARD_LCD_SPI_SCK           40
#define BOARD_LCD_SPI_D0            46
#define BOARD_LCD_SPI_D1            45
#define BOARD_LCD_SPI_D2            42
#define BOARD_LCD_SPI_D3            41
#define BOARD_LCD_SPI_CS            21
#define BOARD_LCD_PIN_BL            5

// ─── I2C ────────────────────────────────────────────────────────────
#define BOARD_I2C_SCL               10
#define BOARD_I2C_SDA               11
#define BOARD_I2C_NUM               0
#define BOARD_I2C_FREQ_HZ           400000

// ─── Touch ──────────────────────────────────────────────────────────
#define BOARD_TOUCH_ADDR            0x53    // SPD2010
#define BOARD_TOUCH_INT_PIN         4

// ─── Audio (raw I2S: separate mic + speaker) ────────────────────────
#define BOARD_AUDIO_TYPE            AUDIO_TYPE_RAW_I2S

// Mic I2S (I2S_NUM_0 RX)
#define BOARD_MIC_I2S_NUM           0
#define BOARD_MIC_SCK_PIN           15
#define BOARD_MIC_WS_PIN            2
#define BOARD_MIC_SD_PIN            39

// Speaker I2S (I2S_NUM_1 TX, PCM5101 DAC)
#define BOARD_SPK_I2S_NUM           1
#define BOARD_SPK_BCLK_PIN          48
#define BOARD_SPK_LRCK_PIN          38
#define BOARD_SPK_DIN_PIN           47
