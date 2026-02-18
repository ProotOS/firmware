#include "audio.h"
#include "board.h"

#if BOARD_HAS_AUDIO && BOARD_AUDIO_TYPE == AUDIO_TYPE_RAW_I2S

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
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_afe_config.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "model_path.h"

static const char *TAG = "Audio";

// ─── Audio format ───────────────────────────────────────────────────
#define SAMPLE_RATE     16000

// ESP-NOW max payload is 250 bytes = 125 samples at 16-bit
#define ESPNOW_AUDIO_BYTES  250
#define AFE_CHUNK_SAMPLES   512

// ─── State ──────────────────────────────────────────────────────────
static i2s_chan_handle_t s_mic_chan;
static i2s_chan_handle_t s_spk_chan;
static RingbufHandle_t   s_spk_ringbuf;
static volatile bool     s_ptt_active;
static volatile int64_t  s_last_rx_time;

static const esp_afe_sr_iface_t *s_afe_iface;
static esp_afe_sr_data_t        *s_afe_data;
static volatile float            s_mic_level_db;
static volatile bool             s_vad_active;

// ─── I2S init ───────────────────────────────────────────────────────

static void init_i2s_mic(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(BOARD_MIC_I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 4;
    chan_cfg.dma_frame_num = 240;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &s_mic_chan));

    // Mic outputs 24-bit samples in 32-bit frames; read as 32-bit and shift to 16-bit
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = BOARD_MIC_SCK_PIN,
            .ws   = BOARD_MIC_WS_PIN,
            .dout = I2S_GPIO_UNUSED,
            .din  = BOARD_MIC_SD_PIN,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_RIGHT;
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_mic_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_mic_chan));
}

static void init_i2s_spk(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(BOARD_SPK_I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 4;
    chan_cfg.dma_frame_num = 240;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &s_spk_chan, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = BOARD_SPK_BCLK_PIN,
            .ws   = BOARD_SPK_LRCK_PIN,
            .dout = BOARD_SPK_DIN_PIN,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_spk_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_spk_chan));
}

// ─── ESP-NOW ────────────────────────────────────────────────────────

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
    xRingbufferSend(s_spk_ringbuf, data, len, 0);
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

// ─── AFE (Audio Front End) ──────────────────────────────────────────

static void init_afe(void)
{
    srmodel_list_t *models = esp_srmodel_init("model");
    afe_config_t *cfg = afe_config_init("M", models, AFE_TYPE_VC, AFE_MODE_LOW_COST);
    if (!cfg) {
        ESP_LOGE(TAG, "AFE config init failed");
        return;
    }

    // Disable features we don't need for PTT
    cfg->aec_init = false;
    cfg->wakenet_init = false;
    cfg->vad_init = true;
    cfg->se_init = false;
    cfg->ns_init = true;
    cfg->agc_init = true;
    cfg->afe_perferred_core = 1;
    cfg->afe_perferred_priority = 5;
    cfg->afe_ringbuf_size = 25;
    cfg->memory_alloc_mode = AFE_MEMORY_ALLOC_MORE_PSRAM;
    cfg->debug_init = false;

    s_afe_iface = esp_afe_handle_from_config(cfg);
    if (!s_afe_iface) {
        ESP_LOGE(TAG, "AFE handle creation failed");
        afe_config_free(cfg);
        return;
    }

    s_afe_data = s_afe_iface->create_from_config(cfg);
    afe_config_free(cfg);

    if (!s_afe_data) {
        ESP_LOGE(TAG, "AFE create failed");
    } else {
        ESP_LOGI(TAG, "AFE initialized (NS + AGC)");
    }
}

// ─── Tasks ──────────────────────────────────────────────────────────

static void mic_feed_task(void *arg)
{
    int chunk_size = s_afe_iface->get_feed_chunksize(s_afe_data);
    // Read 32-bit I2S frames, then convert to 16-bit for AFE
    int32_t *i2s_buf = board_malloc_prefer_spiram(chunk_size * sizeof(int32_t));
    if (!i2s_buf) {
        ESP_LOGE(TAG, "Failed to allocate mic buffer");
        vTaskDelete(NULL);
        return;
    }

    size_t bytes_read;
    while (1) {
        esp_err_t ret = i2s_channel_read(s_mic_chan, i2s_buf,
                                          chunk_size * sizeof(int32_t),
                                          &bytes_read, portMAX_DELAY);
        if (ret == ESP_OK && bytes_read > 0) {
            int samples = bytes_read / sizeof(int32_t);

            // Convert 32-bit I2S frames to 16-bit: mic outputs 24-bit in high bits,
            // shift >> 14 to get ~16-bit signed range (matches Waveshare reference)
            for (int i = 0; i < samples; i++) {
                i2s_buf[i] = i2s_buf[i] >> 14;
            }

            // Compute RMS level from converted samples (data_volume is invalid in VC mode)
            int64_t sum_sq = 0;
            for (int i = 0; i < samples; i++) {
                int16_t s = (int16_t)i2s_buf[i];
                sum_sq += (int32_t)s * s;
            }
            float rms = sqrtf((float)(sum_sq / samples));
            s_mic_level_db = (rms > 0) ? 20.0f * log10f(rms / 32768.0f) : -96.0f;

            // Feed as int16_t* — the >> 14 values fit in 16 bits, reinterpret in-place
            s_afe_iface->feed(s_afe_data, (int16_t *)i2s_buf);
        }
    }
}

static void afe_fetch_task(void *arg)
{
    int chunk_size = s_afe_iface->get_fetch_chunksize(s_afe_data);
    ESP_LOGI(TAG, "AFE fetch chunk size: %d samples", chunk_size);

    while (1) {
        afe_fetch_result_t *res = s_afe_iface->fetch(s_afe_data);
        if (!res || res->ret_value == ESP_FAIL) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // s_mic_level_db is computed from raw I2S in mic_feed_task
        bool was_active = s_vad_active;
        s_vad_active = (res->vad_state == VAD_SPEECH);
        if (s_vad_active && !was_active) {
            ESP_LOGI(TAG, "VAD: speech detected, sending audio");
        } else if (!s_vad_active && was_active) {
            ESP_LOGI(TAG, "VAD: silence, stopped sending");
        }

        if (!s_vad_active) continue;

        // Send processed audio via ESP-NOW in 250-byte chunks
        int16_t *samples = res->data;
        int total_bytes = chunk_size * sizeof(int16_t);
        int offset = 0;
        int sent_ok = 0;

        while (offset < total_bytes) {
            int send_len = total_bytes - offset;
            if (send_len > ESPNOW_AUDIO_BYTES) send_len = ESPNOW_AUDIO_BYTES;

            uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
            esp_err_t err = esp_now_send(broadcast, ((uint8_t *)samples) + offset, send_len);
            if (err == ESP_ERR_ESPNOW_NO_MEM) {
                taskYIELD();
                err = esp_now_send(broadcast, ((uint8_t *)samples) + offset, send_len);
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

static int16_t s_silence[240];  // zero-filled silence buffer (one DMA frame)

static void spk_task(void *arg)
{
    while (1) {
        size_t item_size;
        void *item = xRingbufferReceive(s_spk_ringbuf, &item_size, pdMS_TO_TICKS(20));
        size_t bytes_written;
        if (item) {
            i2s_channel_write(s_spk_chan, item, item_size, &bytes_written, portMAX_DELAY);
            vRingbufferReturnItem(s_spk_ringbuf, item);
        } else {
            // Feed silence to prevent DMA from looping last audio buffer
            i2s_channel_write(s_spk_chan, s_silence, sizeof(s_silence), &bytes_written, pdMS_TO_TICKS(20));
        }
    }
}

// ─── Beep generator ─────────────────────────────────────────────────

#define BEEP_FREQ_HZ    1000
#define BEEP_DURATION_MS 80
#define BEEP_AMPLITUDE   8000

void audio_beep(void)
{
    if (!s_spk_ringbuf) {
        ESP_LOGW(TAG, "beep: no ringbuf");
        return;
    }
    int num_samples = SAMPLE_RATE * BEEP_DURATION_MS / 1000;
    int total_bytes = num_samples * sizeof(int16_t);
    int16_t *tone = board_malloc_prefer_spiram(total_bytes);
    if (!tone) {
        ESP_LOGW(TAG, "beep: alloc failed");
        return;
    }

    for (int i = 0; i < num_samples; i++) {
        float t = (float)i / SAMPLE_RATE;
        float envelope = 1.0f;
        // Fade out last 20ms to avoid click
        int fade_samples = SAMPLE_RATE * 20 / 1000;
        if (i > num_samples - fade_samples) {
            envelope = (float)(num_samples - i) / fade_samples;
        }
        tone[i] = (int16_t)(sinf(2.0f * 3.14159265f * BEEP_FREQ_HZ * t) * BEEP_AMPLITUDE * envelope);
    }

    BaseType_t ok = xRingbufferSend(s_spk_ringbuf, tone, total_bytes, pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "beep: %d bytes, ringbuf send %s", total_bytes, ok ? "OK" : "FAILED");
    heap_caps_free(tone);
}

// ─── Public API ─────────────────────────────────────────────────────

void audio_init(void)
{
    ESP_LOGI(TAG, "Initializing audio subsystem...");

    // Speaker ring buffer in PSRAM (~200ms of 16kHz 16-bit mono)
    StaticRingbuffer_t *rb_struct = board_malloc_prefer_spiram(sizeof(StaticRingbuffer_t));
    uint8_t *rb_storage = board_malloc_prefer_spiram(8192);
    assert(rb_struct && rb_storage);
    s_spk_ringbuf = xRingbufferCreateStatic(8192, RINGBUF_TYPE_BYTEBUF, rb_storage, rb_struct);

    init_i2s_mic();
    init_i2s_spk();
    init_espnow();
    init_afe();

    // Allocate task stacks in PSRAM, TCBs in internal RAM (FreeRTOS requirement)
    StackType_t *mic_stack = board_malloc_prefer_spiram(8192 * sizeof(StackType_t));
    StaticTask_t *mic_tcb = heap_caps_malloc(sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    assert(mic_stack && mic_tcb);
    xTaskCreateStaticPinnedToCore(mic_feed_task, "mic_feed", 8192, NULL, 5, mic_stack, mic_tcb, 0);

    StackType_t *fetch_stack = board_malloc_prefer_spiram(8192 * sizeof(StackType_t));
    StaticTask_t *fetch_tcb = heap_caps_malloc(sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    assert(fetch_stack && fetch_tcb);
    xTaskCreateStaticPinnedToCore(afe_fetch_task, "afe_fetch", 8192, NULL, 5, fetch_stack, fetch_tcb, 1);

    StackType_t *spk_stack = board_malloc_prefer_spiram(4096 * sizeof(StackType_t));
    StaticTask_t *spk_tcb = heap_caps_malloc(sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    assert(spk_stack && spk_tcb);
    xTaskCreateStaticPinnedToCore(spk_task, "spk_play", 4096, NULL, 4, spk_stack, spk_tcb, 0);

    ESP_LOGI(TAG, "Audio subsystem ready (PTT off)");

    // Startup beep to verify speaker works
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
    return (now - s_last_rx_time) < 200000; // 200ms
}

float audio_get_mic_level_db(void)
{
    return s_mic_level_db;
}

bool audio_get_vad_active(void)
{
    return s_vad_active;
}

#endif // BOARD_HAS_AUDIO && AUDIO_TYPE_RAW_I2S
