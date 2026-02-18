#include <stdio.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ble_hid.h"
#include "board.h"
#include "audio.h"
#include "ui.h"
#include "esp_heap_caps.h"
#include "QMI8658.h"

static const char *TAG = "Prootos";

static const char *usage_page_name(uint16_t page)
{
    switch (page) {
    case 0x01: return "Desktop";
    case 0x07: return "Keyboard";
    case 0x09: return "Button";
    case 0x0C: return "Consumer";
    default:   return "Unknown";
    }
}

static void on_hid_event(const hid_event_t *event)
{
    const char *type_str = "?";
    switch (event->type) {
    case HID_EVENT_BUTTON_DOWN: type_str = "BTN_DOWN"; break;
    case HID_EVENT_BUTTON_UP:   type_str = "BTN_UP";   break;
    case HID_EVENT_AXIS_CHANGE: type_str = "AXIS";     break;
    }

    ESP_LOGI(TAG, "[Dev %d] %s  page=%s(0x%04x) usage=0x%04x value=%"PRId32,
             event->device_index, type_str,
             usage_page_name(event->usage_page), event->usage_page,
             event->usage, event->value);

#if BOARD_HAS_AUDIO
    if (event->type == HID_EVENT_BUTTON_DOWN) {
        audio_beep();
    }
#endif
}

static void on_ble_status(uint8_t idx, const char *name, bool connected)
{
    ESP_LOGI(TAG, "BLE[%d] %s %s", idx, name, connected ? "connected" : "disconnected");
    ui_update_ble_device(idx, name, connected);
}

static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Lock ESP-NOW to channel 1 so all devices communicate on the same channel
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

    uint8_t primary;
    wifi_second_chan_t second;
    esp_wifi_get_channel(&primary, &second);
    ESP_LOGI(TAG, "WiFi channel: %d", primary);
}

void app_main(void)
{
    ESP_LOGI(TAG, "╔═══════════════════════════════════╗");
    ESP_LOGI(TAG, "║          Prootos Firmware         ║");
    ESP_LOGI(TAG, "╚═══════════════════════════════════╝");

    // Initialize NVS (required by BLE + WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // WiFi STA mode (required for ESP-NOW, no AP connection needed)
    wifi_init_sta();

    // Display + LVGL
    ui_init();

    // BLE HID remotes — init before audio so Bluedroid gets internal RAM
    ble_hid_init(on_hid_event);
    ble_hid_set_status_callback(on_ble_status);

#if BOARD_HAS_IMU
    // IMU (uses I2C bus initialized by ui_init)
    QMI8658_Init();
#endif

#if BOARD_HAS_AUDIO
    // Audio (ESP-NOW + I2S + AFE) — AFE uses mostly PSRAM
    audio_init();
#endif

    ESP_LOGI(TAG, "Heap: internal free=%u min=%u | DMA free=%u",
             heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
             heap_caps_get_free_size(MALLOC_CAP_DMA));

    // Main loop: debug UI + LVGL
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
#if BOARD_HAS_AUDIO
        ui_set_audio_tx(audio_get_vad_active());
        ui_set_audio_rx(audio_is_receiving());
#endif
        ui_update_debug();
        ui_update();
    }
}
