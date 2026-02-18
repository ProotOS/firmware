// ─── Minimal ES8311 codec driver ─────────────────────────────────────
// Direct I2C register writes, no esp_codec_dev dependency.
// Register map and init sequence from: espressif/esp-adf es8311.c

#include "es8311.h"

#if BOARD_AUDIO_TYPE == AUDIO_TYPE_ES8311

#include "I2C_Driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ES8311";

// ─── ES8311 Register Map (from esp-adf) ─────────────────────────────
#define ES8311_ADDR         BOARD_ES8311_ADDR  // 7-bit I2C address (0x18)

#define REG00_RESET         0x00
#define REG01_CLK_MGR1      0x01
#define REG02_CLK_MGR2      0x02
#define REG03_CLK_MGR3      0x03
#define REG04_CLK_MGR4      0x04
#define REG05_CLK_MGR5      0x05
#define REG06_CLK_MGR6      0x06
#define REG07_CLK_MGR7      0x07
#define REG08_CLK_MGR8      0x08
#define REG09_SDPIN         0x09  // DAC serial data input format
#define REG0A_SDPOUT        0x0A  // ADC serial data output format
#define REG0B_SYSTEM        0x0B
#define REG0C_SYSTEM        0x0C
#define REG0D_SYSTEM        0x0D
#define REG0E_SYSTEM        0x0E
#define REG0F_SYSTEM        0x0F
#define REG10_SYSTEM        0x10
#define REG11_SYSTEM        0x11
#define REG12_SYSTEM        0x12
#define REG13_SYSTEM        0x13
#define REG14_SYSTEM        0x14
#define REG15_ADC           0x15
#define REG16_ADC           0x16  // ADC PGA gain
#define REG17_ADC           0x17
#define REG1B_ADC           0x1B  // ADC EQ / ramp rate
#define REG1C_ADC           0x1C
#define REG31_DAC           0x31
#define REG32_DAC           0x32  // DAC volume
#define REG33_DAC           0x33
#define REG34_DAC           0x34
#define REG35_DAC           0x35
#define REG37_DAC           0x37
#define REG44_GPIO          0x44
#define REG45_GP            0x45
#define REGFD_CHD1          0xFD  // Chip ID byte 1
#define REGFE_CHD2          0xFE  // Chip ID byte 2
#define REGFF_CHVER         0xFF  // Chip version

// ─── Helpers ────────────────────────────────────────────────────────

static esp_err_t es_write(uint8_t reg, uint8_t val)
{
    esp_err_t ret = I2C_Write(ES8311_ADDR, reg, &val, 1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "I2C write 0x%02x=0x%02x failed: 0x%x", reg, val, ret);
    }
    return ret;
}

static uint8_t es_read(uint8_t reg)
{
    uint8_t val = 0;
    I2C_Read(ES8311_ADDR, reg, &val, 1);
    return val;
}

// ─── Public API ─────────────────────────────────────────────────────

esp_err_t es8311_init(void)
{
    // Read chip ID to verify I2C communication
    uint8_t id1 = es_read(REGFD_CHD1);
    uint8_t id2 = es_read(REGFE_CHD2);
    uint8_t ver = es_read(REGFF_CHVER);
    ESP_LOGI(TAG, "ES8311 chip ID: 0x%02x 0x%02x ver=0x%02x", id1, id2, ver);

    // ─── Init sequence from esp-adf es8311_codec_init ───────────────
    // Phase 1: Initial register setup (clocks off, configure topology)
    es_write(REG44_GPIO, 0x08);        // GPIO: improve I2C noise immunity
    es_write(REG44_GPIO, 0x08);        // (repeated for reliability per esp-adf)
    es_write(REG01_CLK_MGR1, 0x30);    // Clock: MCLK on, BCLK on (other clocks off)
    es_write(REG02_CLK_MGR2, 0x00);    // Pre-divider and multiplier defaults
    es_write(REG03_CLK_MGR3, 0x10);    // ADC fsmode + OSR
    es_write(REG16_ADC, 0x24);         // ADC PGA gain (default)
    es_write(REG04_CLK_MGR4, 0x10);    // DAC OSR
    es_write(REG05_CLK_MGR5, 0x00);    // ADC/DAC clock dividers
    es_write(REG0B_SYSTEM, 0x00);      // System config
    es_write(REG0C_SYSTEM, 0x00);      // System config
    es_write(REG10_SYSTEM, 0x1F);      // System power
    es_write(REG11_SYSTEM, 0x7F);      // System config

    // Phase 2: Reset codec, set slave mode
    es_write(REG00_RESET, 0x80);       // CSM_ON=1, reset done
    // Slave mode: clear bit 6 (MST bit). 0x80 & 0xBF = 0x80 — already correct.

    // Phase 3: Enable ALL clocks (critical! 0x3F not just 0x30)
    // Bits: [7]=MCLK_SEL [6]=MCLK_INV [5]=MCLK_ON [4]=BCLK_ON [3:0]=CLK enables
    // 0x3F = MCLK from pin, MCLK on, BCLK on, all sub-clocks on
    es_write(REG01_CLK_MGR1, 0x3F);

    // ─── Clock coefficients for 16kHz, MCLK=4096000 (256*fs) ───────
    // From coeff_div table: {4096000, 16000, 0x01, 0x01, 0x01, 0x01,
    //   0x00, 0x00, 0xff, 0x04, 0x10, 0x20}

    // REG02: pre_div(bits 7:5) | pre_multi(bits 4:3) | rest
    uint8_t reg02 = es_read(REG02_CLK_MGR2);
    reg02 &= 0x07;  // clear bits 7:3
    reg02 |= ((1 - 1) << 5);  // pre_div=1 → write 0
    reg02 |= (0 << 3);        // pre_multi=1 → code 0
    es_write(REG02_CLK_MGR2, reg02);

    // REG05: adc_div(bits 7:4) | dac_div(bits 3:0)
    es_write(REG05_CLK_MGR5, ((1 - 1) << 4) | (1 - 1));  // adc_div=1, dac_div=1

    // REG03: fs_mode(bit 6) | adc_osr(bits 5:0)
    es_write(REG03_CLK_MGR3, (0 << 6) | 0x10);  // single speed, adc_osr=0x10

    // REG04: dac_osr
    es_write(REG04_CLK_MGR4, 0x20);

    // REG07/08: LRCK divider
    es_write(REG07_CLK_MGR7, 0x00);
    es_write(REG08_CLK_MGR8, 0xFF);

    // REG06: BCLK divider
    uint8_t reg06 = es_read(REG06_CLK_MGR6);
    reg06 &= 0xE0;  // clear divider bits
    reg06 |= 0x04;  // bclk_div=4
    es_write(REG06_CLK_MGR6, reg06);

    // ─── I2S format: standard I2S, 16-bit ───────────────────────────
    // REG09 (DAC SDP): I2S format (bits 1:0 = 00), 16-bit (bits 4:2)
    uint8_t reg09 = es_read(REG09_SDPIN);
    reg09 &= 0xFC;  // I2S normal format
    reg09 &= 0xE3;  // clear data length bits
    reg09 |= (0x03 << 2);  // 16-bit
    es_write(REG09_SDPIN, reg09);

    // REG0A (ADC SDP): I2S format, 16-bit
    uint8_t reg0a = es_read(REG0A_SDPOUT);
    reg0a &= 0xFC;  // I2S normal format
    reg0a &= 0xE3;
    reg0a |= (0x03 << 2);  // 16-bit
    es_write(REG0A_SDPOUT, reg0a);

    // ─── Power up DAC + ADC ─────────────────────────────────────────
    es_write(REG0D_SYSTEM, 0x01);      // Power on analog
    es_write(REG0E_SYSTEM, 0x02);      // Enable analog VMID
    es_write(REG12_SYSTEM, 0x00);      // Power up DAC
    es_write(REG13_SYSTEM, 0x10);      // DAC/ADC bias current
    es_write(REG1B_ADC, 0x0A);        // ADC EQ config (from esp-adf)
    es_write(REG1C_ADC, 0x6A);        // ADC power up
    es_write(REG14_SYSTEM, 0x1A);      // Power control

    // ─── Set reasonable default volumes ─────────────────────────────
    es_write(REG32_DAC, 191);          // DAC volume (~75%)
    es_write(REG17_ADC, 191);          // ADC volume (~75%)

    // Unmute DAC
    uint8_t reg31 = es_read(REG31_DAC);
    reg31 &= ~0x60;  // clear mute bits
    es_write(REG31_DAC, reg31);

    // Unmute ADC
    uint8_t reg15 = es_read(REG15_ADC);
    reg15 &= ~0x03;  // clear mute bits (bits 1:0)
    es_write(REG15_ADC, reg15);

    ESP_LOGI(TAG, "ES8311 initialized (16kHz 16-bit I2S slave)");
    return ESP_OK;
}

esp_err_t es8311_set_voice_volume(uint8_t volume)
{
    // ES8311 DAC volume register: 0=mute, 255=max
    return es_write(REG32_DAC, volume);
}

esp_err_t es8311_set_mic_gain(uint8_t gain)
{
    // ES8311 ADC PGA gain: written directly to REG16
    // Typical values: 0x00=0dB, 0x14=+20dB, 0x24=+36dB (default), etc.
    return es_write(REG16_ADC, gain);
}

#endif // BOARD_AUDIO_TYPE == AUDIO_TYPE_ES8311
