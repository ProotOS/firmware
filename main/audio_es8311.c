// ─── ES8311 codec audio backend (ESP32-C6) ──────────────────────────
// Single I2S duplex for mic + speaker via ES8311 codec.
// No AFE — raw audio passthrough for PTT and ESP-NOW.

#include "audio.h"
#include "board.h"

#if BOARD_HAS_AUDIO && BOARD_AUDIO_TYPE == AUDIO_TYPE_ES8311

#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "driver/i2s_std.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/ringbuf.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "es8311.h"

static const char *TAG = "Audio";

// ─── Audio format ───────────────────────────────────────────────────
#define SAMPLE_RATE     16000

// ESP-NOW max payload is 250 bytes = 125 samples at 16-bit
#define ESPNOW_AUDIO_BYTES  250
#define MIC_CHUNK_SAMPLES   512

// ─── State ──────────────────────────────────────────────────────────
static i2s_chan_handle_t s_tx_chan;
static i2s_chan_handle_t s_rx_chan;
static RingbufHandle_t   s_spk_ringbuf;
static volatile bool     s_ptt_active;
static volatile int64_t  s_last_rx_time;
static volatile float    s_mic_level_db;
static volatile bool     s_vad_active;

// ─── I2S init (single duplex channel for ES8311) ────────────────────

static void init_i2s(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(BOARD_I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 4;
    chan_cfg.dma_frame_num = 240;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &s_tx_chan, &s_rx_chan));

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = SAMPLE_RATE,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_STEREO,
            .slot_mask = I2S_STD_SLOT_BOTH,
            .ws_width = 16,
            .ws_pol = false,
            .bit_shift = true,
            .left_align = true,
            .big_endian = false,
            .bit_order_lsb = false,
        },
        .gpio_cfg = {
            .mclk = BOARD_I2S_MCLK_PIN,
            .bclk = BOARD_I2S_BCLK_PIN,
            .ws   = BOARD_I2S_WS_PIN,
            .dout = BOARD_I2S_DOUT_PIN,
            .din  = BOARD_I2S_DIN_PIN,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_tx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_rx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_tx_chan));
    ESP_ERROR_CHECK(i2s_channel_enable(s_rx_chan));
}

// ─── ESP-NOW ────────────────────────────────────────────────────────

// Temporary buffer for mono→stereo conversion in ESP-NOW callback
static int16_t s_rx_stereo[ESPNOW_AUDIO_BYTES];  // 250 mono bytes → 500 stereo bytes max

static uint32_t s_rx_pkt_count;

static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (len <= 0 || !s_spk_ringbuf) return;
    s_last_rx_time = esp_timer_get_time();
    s_rx_pkt_count++;
    if ((s_rx_pkt_count % 50) == 1) {
        ESP_LOGI(TAG, "RX: pkt #%lu, %d bytes from " MACSTR,
                 (unsigned long)s_rx_pkt_count, len, MAC2STR(info->src_addr));
    }

    // Convert mono to stereo for ES8311
    int16_t *mono = (int16_t *)data;
    int mono_samples = len / sizeof(int16_t);
    for (int i = 0; i < mono_samples && i < ESPNOW_AUDIO_BYTES / 2; i++) {
        s_rx_stereo[i * 2] = mono[i];
        s_rx_stereo[i * 2 + 1] = mono[i];
    }
    xRingbufferSend(s_spk_ringbuf, s_rx_stereo, mono_samples * 2 * sizeof(int16_t), 0);
}

static void init_espnow(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    // Add broadcast peer
    // channel=0 means "use the interface's current channel" (set to 1 in wifi_init_sta)
    esp_now_peer_info_t peer = {
        .channel = 0,
        .ifidx = ESP_IF_WIFI_STA,
        .encrypt = false,
    };
    memset(peer.peer_addr, 0xFF, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    uint8_t mac[6];
    esp_wifi_get_mac(ESP_IF_WIFI_STA, mac);
    uint8_t ch;
    wifi_second_chan_t sec;
    esp_wifi_get_channel(&ch, &sec);
    ESP_LOGI(TAG, "ESP-NOW ready: MAC=" MACSTR " ch=%d", MAC2STR(mac), ch);
}

// ─── Tasks ──────────────────────────────────────────────────────────

static void mic_task(void *arg)
{
    // ES8311 in stereo mode: read interleaved L/R, extract left channel (mic)
    const int stereo_samples = MIC_CHUNK_SAMPLES;
    const int stereo_bytes = stereo_samples * 2 * sizeof(int16_t);  // L+R interleaved
    int16_t *stereo_buf = heap_caps_malloc(stereo_bytes, MALLOC_CAP_INTERNAL);
    int16_t *mono_buf = heap_caps_malloc(MIC_CHUNK_SAMPLES * sizeof(int16_t), MALLOC_CAP_INTERNAL);
    if (!stereo_buf || !mono_buf) {
        ESP_LOGE(TAG, "Failed to allocate mic buffers");
        vTaskDelete(NULL);
        return;
    }

    size_t bytes_read;
    while (1) {
        esp_err_t ret = i2s_channel_read(s_rx_chan, stereo_buf, stereo_bytes,
                                          &bytes_read, portMAX_DELAY);
        if (ret != ESP_OK || bytes_read == 0) continue;

        int stereo_sample_count = bytes_read / (2 * sizeof(int16_t));  // pairs

        // Extract left channel (mic) from interleaved stereo
        for (int i = 0; i < stereo_sample_count; i++) {
            mono_buf[i] = stereo_buf[i * 2];  // left channel
        }

        // Compute RMS level
        int64_t sum_sq = 0;
        for (int i = 0; i < stereo_sample_count; i++) {
            int32_t s = mono_buf[i];
            sum_sq += s * s;
        }
        float rms = sqrtf((float)(sum_sq / stereo_sample_count));
        s_mic_level_db = (rms > 0) ? 20.0f * log10f(rms / 32768.0f) : -96.0f;

        // Simple energy-based VAD
        bool was_active = s_vad_active;
        s_vad_active = (s_mic_level_db > -40.0f);
        if (s_vad_active && !was_active) {
            ESP_LOGI(TAG, "VAD: speech (%.1f dB), sending", s_mic_level_db);
        } else if (!s_vad_active && was_active) {
            ESP_LOGI(TAG, "VAD: silence (%.1f dB), stopped", s_mic_level_db);
        }

        if (!s_vad_active) continue;

        // Send mono audio via ESP-NOW in 250-byte chunks
        int total_bytes = stereo_sample_count * sizeof(int16_t);
        int offset = 0;
        int sent_ok = 0;
        while (offset < total_bytes) {
            int send_len = total_bytes - offset;
            if (send_len > ESPNOW_AUDIO_BYTES) send_len = ESPNOW_AUDIO_BYTES;

            uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
            esp_err_t err = esp_now_send(broadcast, ((uint8_t *)mono_buf) + offset, send_len);
            if (err == ESP_ERR_ESPNOW_NO_MEM) {
                taskYIELD();
                err = esp_now_send(broadcast, ((uint8_t *)mono_buf) + offset, send_len);
            }
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "esp_now_send: %s after %d pkts", esp_err_to_name(err), sent_ok);
                break;
            }
            sent_ok++;
            offset += send_len;
        }
        ESP_LOGI(TAG, "TX: %d pkts (%d bytes)", sent_ok, offset);
    }
}

static int16_t s_silence[240 * 2];  // zero-filled stereo silence (one DMA frame)

static void spk_task(void *arg)
{
    // Ring buffer carries stereo data (already converted by sender).
    // ESP-NOW mono data is converted to stereo by espnow_recv_cb before enqueue.
    // Beep data is also enqueued as stereo.
    while (1) {
        size_t item_size;
        void *item = xRingbufferReceive(s_spk_ringbuf, &item_size, pdMS_TO_TICKS(20));
        size_t bytes_written;
        if (item) {
            i2s_channel_write(s_tx_chan, item, item_size,
                              &bytes_written, portMAX_DELAY);
            vRingbufferReturnItem(s_spk_ringbuf, item);
        } else {
            // Feed silence to prevent DMA from looping last buffer
            i2s_channel_write(s_tx_chan, s_silence, sizeof(s_silence),
                              &bytes_written, pdMS_TO_TICKS(20));
        }
    }
}

// ─── Beep generator ─────────────────────────────────────────────────

#define BEEP_FREQ_HZ     1000
#define BEEP_DURATION_MS  80
#define BEEP_AMPLITUDE    8000

void audio_beep(void)
{
    if (!s_spk_ringbuf) return;

    int num_samples = SAMPLE_RATE * BEEP_DURATION_MS / 1000;
    // Generate stereo tone (ES8311 expects stereo I2S)
    int stereo_bytes = num_samples * 2 * sizeof(int16_t);
    int16_t *tone = heap_caps_malloc(stereo_bytes, MALLOC_CAP_INTERNAL);
    if (!tone) {
        ESP_LOGW(TAG, "beep: alloc failed");
        return;
    }

    for (int i = 0; i < num_samples; i++) {
        float t = (float)i / SAMPLE_RATE;
        float envelope = 1.0f;
        int fade_samples = SAMPLE_RATE * 20 / 1000;
        if (i > num_samples - fade_samples) {
            envelope = (float)(num_samples - i) / fade_samples;
        }
        int16_t sample = (int16_t)(sinf(2.0f * 3.14159265f * BEEP_FREQ_HZ * t) * BEEP_AMPLITUDE * envelope);
        tone[i * 2] = sample;      // left
        tone[i * 2 + 1] = sample;  // right
    }

    // Send stereo beep through ring buffer (spk_task is the sole I2S writer)
    // Send in chunks that fit the ring buffer
    int offset = 0;
    while (offset < stereo_bytes) {
        int chunk = stereo_bytes - offset;
        if (chunk > 1024) chunk = 1024;
        if (!xRingbufferSend(s_spk_ringbuf, ((uint8_t *)tone) + offset, chunk, pdMS_TO_TICKS(100))) {
            break;  // ring buffer full, skip rest
        }
        offset += chunk;
    }
    ESP_LOGI(TAG, "beep: %d bytes enqueued", offset);
    heap_caps_free(tone);
}

// ─── Public API ─────────────────────────────────────────────────────

void audio_init(void)
{
    ESP_LOGI(TAG, "Initializing audio subsystem (ES8311)...");

    // Speaker ring buffer (~200ms of 16kHz 16-bit mono from ESP-NOW)
    StaticRingbuffer_t *rb_struct = heap_caps_malloc(sizeof(StaticRingbuffer_t), MALLOC_CAP_INTERNAL);
    uint8_t *rb_storage = heap_caps_malloc(8192, MALLOC_CAP_INTERNAL);
    assert(rb_struct && rb_storage);
    s_spk_ringbuf = xRingbufferCreateStatic(8192, RINGBUF_TYPE_BYTEBUF, rb_storage, rb_struct);

    init_i2s();

    // Initialize ES8311 codec via I2C (I2C bus already initialized by ui_init)
    esp_err_t ret = es8311_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ES8311 init failed");
    }

    init_espnow();

    // Mic task (reads from I2S RX, computes level, sends via ESP-NOW when PTT)
    xTaskCreatePinnedToCore(mic_task, "mic_es8311", 4096, NULL, 5, NULL, 0);

    // Speaker task (reads from ring buffer, writes to I2S TX)
    xTaskCreatePinnedToCore(spk_task, "spk_es8311", 4096, NULL, 4, NULL, 0);

    ESP_LOGI(TAG, "Audio subsystem ready (ES8311, PTT off)");

    // Startup beep
    audio_beep();
}

void audio_set_ptt(bool active)
{
    if (s_ptt_active != active) {
        s_ptt_active = active;
        ESP_LOGI(TAG, "PTT %s", active ? "ON" : "OFF");
    }
}

bool audio_is_receiving(void)
{
    if (s_last_rx_time == 0) return false;
    int64_t now = esp_timer_get_time();
    return (now - s_last_rx_time) < 200000;  // 200ms
}

float audio_get_mic_level_db(void)
{
    return s_mic_level_db;
}

bool audio_get_vad_active(void)
{
    return s_vad_active;
}

#endif // BOARD_HAS_AUDIO && AUDIO_TYPE_ES8311
