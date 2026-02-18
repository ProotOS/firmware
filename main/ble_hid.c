#include "ble_hid.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hid_report_parser.h"
#include "hid_event.h"

static const char *TAG = "BLE_HID";

// BLE UUIDs
#define HID_SERVICE_UUID         0x1812
#define HID_REPORT_MAP_UUID      0x2A4B
#define HID_REPORT_UUID          0x2A4D
#define HID_REPORT_REF_UUID      0x2908
#define CLIENT_CHAR_CONFIG_UUID  0x2902

#define MAX_HID_REPORTS     8

// Per-report GATT handle info
typedef struct {
    uint16_t char_handle;
    uint16_t cccd_handle;
    uint16_t ref_handle;
    uint8_t  report_id;
    uint8_t  report_type;     // 1=Input, 2=Output, 3=Feature
    bool     has_ref;
} hid_report_handle_t;

// Per-device state
typedef struct {
    bool             in_use;
    bool             encrypted;
    uint16_t         conn_id;
    esp_bd_addr_t    bda;
    uint8_t          addr_type;
    char             name[32];

    uint16_t         hid_start_handle;
    uint16_t         hid_end_handle;

    uint16_t         report_map_handle;
    uint8_t          report_map_data[512];
    size_t           report_map_len;

    hid_report_handle_t reports[MAX_HID_REPORTS];
    int              report_count;

    hid_report_map_t parsed_map;
    bool             map_parsed;

    int              refs_read;

    hid_event_state_t event_state;
} ble_hid_device_t;

static ble_hid_device_t s_devices[BLE_HID_MAX_DEVICES];
static esp_gatt_if_t    s_gattc_if = ESP_GATT_IF_NONE;
static ble_hid_status_callback_t s_status_cb = NULL;

// ─── Device table helpers ───────────────────────────────────────────

static ble_hid_device_t *device_by_conn_id(uint16_t conn_id)
{
    for (int i = 0; i < BLE_HID_MAX_DEVICES; i++) {
        if (s_devices[i].in_use && s_devices[i].conn_id == conn_id)
            return &s_devices[i];
    }
    return NULL;
}

static ble_hid_device_t *device_by_bda(const esp_bd_addr_t bda)
{
    for (int i = 0; i < BLE_HID_MAX_DEVICES; i++) {
        if (s_devices[i].in_use && memcmp(s_devices[i].bda, bda, 6) == 0)
            return &s_devices[i];
    }
    return NULL;
}

static int device_index(const ble_hid_device_t *dev)
{
    return (int)(dev - s_devices);
}

static ble_hid_device_t *device_alloc(void)
{
    for (int i = 0; i < BLE_HID_MAX_DEVICES; i++) {
        if (!s_devices[i].in_use) {
            memset(&s_devices[i], 0, sizeof(ble_hid_device_t));
            s_devices[i].in_use = true;
            return &s_devices[i];
        }
    }
    return NULL;
}

static void device_free(ble_hid_device_t *dev)
{
    dev->in_use = false;
}

static int device_count(void)
{
    int n = 0;
    for (int i = 0; i < BLE_HID_MAX_DEVICES; i++) {
        if (s_devices[i].in_use) n++;
    }
    return n;
}

static bool device_exists_bda(const esp_bd_addr_t bda)
{
    return device_by_bda(bda) != NULL;
}

// ─── Forward declarations ───────────────────────────────────────────

static void enable_notifications(ble_hid_device_t *dev);
static void read_report_map(ble_hid_device_t *dev);
static void read_next_report_ref(ble_hid_device_t *dev);
static void start_scanning(void);

// ─── GAP callback ───────────────────────────────────────────────────

static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_SET_EXT_SCAN_PARAMS_COMPLETE_EVT:
        ESP_LOGI(TAG, "Scan parameters set, starting scan...");
        esp_ble_gap_start_ext_scan(0, 0);
        break;

    case ESP_GAP_BLE_EXT_SCAN_START_COMPLETE_EVT:
        if (param->ext_scan_start.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Scanning for HID devices...");
        }
        break;

    case ESP_GAP_BLE_EXT_ADV_REPORT_EVT: {
        esp_ble_gap_ext_adv_reprot_t *report = &param->ext_adv_report.params;

        if (device_exists_bda(report->addr)) break;
        if (device_count() >= BLE_HID_MAX_DEVICES) break;

        // Check for HID service UUID in advertisement data
        uint8_t *svc_data = NULL;
        uint8_t svc_data_len = 0;
        bool has_hid = false;

        svc_data = esp_ble_resolve_adv_data(report->adv_data,
                                            ESP_BLE_AD_TYPE_16SRV_CMPL, &svc_data_len);
        if (svc_data) {
            for (int i = 0; i + 1 < svc_data_len; i += 2) {
                uint16_t uuid = svc_data[i] | (svc_data[i + 1] << 8);
                if (uuid == HID_SERVICE_UUID) { has_hid = true; break; }
            }
        }

        if (!has_hid) {
            svc_data = esp_ble_resolve_adv_data(report->adv_data,
                                                ESP_BLE_AD_TYPE_16SRV_PART, &svc_data_len);
            if (svc_data) {
                for (int i = 0; i + 1 < svc_data_len; i += 2) {
                    uint16_t uuid = svc_data[i] | (svc_data[i + 1] << 8);
                    if (uuid == HID_SERVICE_UUID) { has_hid = true; break; }
                }
            }
        }

        if (!has_hid) {
            uint8_t *appearance_data = NULL;
            uint8_t appearance_len = 0;
            appearance_data = esp_ble_resolve_adv_data(report->adv_data,
                                                       ESP_BLE_AD_TYPE_APPEARANCE, &appearance_len);
            if (appearance_data && appearance_len >= 2) {
                uint16_t appearance = appearance_data[0] | (appearance_data[1] << 8);
                if (appearance >= 0x03C0 && appearance <= 0x03C4) {
                    has_hid = true;
                }
            }
        }

        if (!has_hid) break;

        // Get device name — skip nameless devices
        uint8_t *adv_name = NULL;
        uint8_t adv_name_len = 0;
        adv_name = esp_ble_resolve_adv_data(report->adv_data,
                                            ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
        if (!adv_name) {
            adv_name = esp_ble_resolve_adv_data(report->adv_data,
                                                ESP_BLE_AD_TYPE_NAME_SHORT, &adv_name_len);
        }
        if (!adv_name || adv_name_len == 0) {
            ESP_LOGD(TAG, "Skipping nameless HID device %02x:%02x:%02x:%02x:%02x:%02x",
                     report->addr[0], report->addr[1], report->addr[2],
                     report->addr[3], report->addr[4], report->addr[5]);
            break;
        }

        ble_hid_device_t *dev = device_alloc();
        if (!dev) break;

        memcpy(dev->bda, report->addr, 6);
        dev->addr_type = report->addr_type;
        size_t copy_len = adv_name_len < sizeof(dev->name) - 1
                        ? adv_name_len : sizeof(dev->name) - 1;
        memcpy(dev->name, adv_name, copy_len);
        dev->name[copy_len] = '\0';

        ESP_LOGI(TAG, "Found HID device: %s [%d/%d]", dev->name,
                 device_count(), BLE_HID_MAX_DEVICES);

        // Stop scanning before connecting (required on some BLE 5.0 stacks)
        esp_ble_gap_stop_ext_scan();

        esp_err_t err = esp_ble_gattc_aux_open(s_gattc_if, dev->bda, dev->addr_type, true);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "[%s] aux_open failed: %s", dev->name, esp_err_to_name(err));
            device_free(dev);
            start_scanning();
        }
        break;
    }

    case ESP_GAP_BLE_EXT_SCAN_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "Scan stopped");
        break;

    case ESP_GAP_BLE_SEC_REQ_EVT:
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;

    case ESP_GAP_BLE_NC_REQ_EVT:
        esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
        break;

    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:
        ESP_LOGI(TAG, "Passkey: %06"PRIu32, param->ble_security.key_notif.passkey);
        break;

    case ESP_GAP_BLE_KEY_EVT:
        break;

    case ESP_GAP_BLE_AUTH_CMPL_EVT: {
        ble_hid_device_t *dev = device_by_bda(param->ble_security.auth_cmpl.bd_addr);
        if (!dev) break;

        if (param->ble_security.auth_cmpl.success) {
            ESP_LOGI(TAG, "[%s] Paired successfully", dev->name);
            dev->encrypted = true;
            read_report_map(dev);
        } else {
            ESP_LOGE(TAG, "[%s] Pairing failed: 0x%x", dev->name,
                     param->ble_security.auth_cmpl.fail_reason);
        }
        break;
    }

    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(TAG, "Conn params updated: status=%d, interval=%d (%.1fms), latency=%d, timeout=%d",
                 param->update_conn_params.status,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.conn_int * 1.25f,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;

    default:
        break;
    }
}

// ─── Read Report Map characteristic ─────────────────────────────────

static void deferred_read_task(void *arg)
{
    ble_hid_device_t *dev = (ble_hid_device_t *)arg;

    // Wait for the Bluedroid stack to settle after encryption.
    // Without this delay, the read can get dropped by the stack
    // (manifests as "Unknown operation encryption completed" error).
    vTaskDelay(pdMS_TO_TICKS(1500));

    if (!dev->in_use || !dev->encrypted) {
        vTaskDelete(NULL);
        return;
    }

    if (dev->report_map_handle == 0) {
        ESP_LOGE(TAG, "[%s] No Report Map handle found", dev->name);
        enable_notifications(dev);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "[%s] Reading HID Report Map (handle=%d, if=%d, conn=%d)...",
             dev->name, dev->report_map_handle, s_gattc_if, dev->conn_id);
    esp_err_t err = esp_ble_gattc_read_char(s_gattc_if, dev->conn_id,
                            dev->report_map_handle, ESP_GATT_AUTH_REQ_NO_MITM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[%s] esp_ble_gattc_read_char failed: %s", dev->name, esp_err_to_name(err));
        enable_notifications(dev);
        vTaskDelete(NULL);
        return;
    }

    // Wait for the read to complete; if it doesn't, fall back to raw mode
    vTaskDelay(pdMS_TO_TICKS(3000));
    if (!dev->map_parsed) {
        ESP_LOGW(TAG, "[%s] Report Map read timed out, enabling raw notifications", dev->name);
        enable_notifications(dev);
    }
    vTaskDelete(NULL);
}

static void read_report_map(ble_hid_device_t *dev)
{
    xTaskCreate(deferred_read_task, "rm_read", 2048, dev, 5, NULL);
}

// ─── Read Report Reference descriptors ──────────────────────────────

static void read_next_report_ref(ble_hid_device_t *dev)
{
    while (dev->refs_read < dev->report_count) {
        if (dev->reports[dev->refs_read].ref_handle != 0 &&
            !dev->reports[dev->refs_read].has_ref) {
            ESP_LOGI(TAG, "[%s] Reading Report Reference at handle %d",
                     dev->name, dev->reports[dev->refs_read].ref_handle);
            esp_ble_gattc_read_char_descr(s_gattc_if, dev->conn_id,
                                          dev->reports[dev->refs_read].ref_handle,
                                          ESP_GATT_AUTH_REQ_NO_MITM);
            return;
        }
        dev->refs_read++;
    }

    ESP_LOGI(TAG, "[%s] Report Reference mapping complete:", dev->name);
    for (int i = 0; i < dev->report_count; i++) {
        ESP_LOGI(TAG, "  Handle %d -> Report ID %d, Type %d",
                 dev->reports[i].char_handle,
                 dev->reports[i].report_id,
                 dev->reports[i].report_type);
    }

    enable_notifications(dev);
}

// ─── Enable notifications on all input report CCCDs ─────────────────

static void enable_notifications(ble_hid_device_t *dev)
{
    int enabled = 0;
    uint8_t notify_en[] = {0x01, 0x00};

    for (int i = 0; i < dev->report_count; i++) {
        if (dev->reports[i].has_ref && dev->reports[i].report_type != 1) continue;
        if (dev->reports[i].cccd_handle == 0) continue;

        esp_ble_gattc_write_char_descr(s_gattc_if, dev->conn_id,
                                       dev->reports[i].cccd_handle,
                                       sizeof(notify_en), notify_en,
                                       ESP_GATT_WRITE_TYPE_RSP,
                                       ESP_GATT_AUTH_REQ_MITM);
        enabled++;
    }

    ESP_LOGI(TAG, "[%s] Enabling notifications on %d reports", dev->name, enabled);

    if (s_status_cb) {
        s_status_cb(device_index(dev), dev->name, true);
    }

    if (device_count() < BLE_HID_MAX_DEVICES) {
        start_scanning();
    }
}

// ─── GATT client event handler ──────────────────────────────────────

static void gattc_event_handler(esp_gattc_cb_event_t event,
                                esp_gatt_if_t gattc_if,
                                esp_ble_gattc_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(TAG, "GATT client registered");
        s_gattc_if = gattc_if;

        static esp_ble_ext_scan_params_t scan_params = {
            .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
            .filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
            .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE,
            .cfg_mask = ESP_BLE_GAP_EXT_SCAN_CFG_UNCODE_MASK,
            .uncoded_cfg = {
                .scan_type = BLE_SCAN_TYPE_ACTIVE,
                .scan_interval = 0x50,
                .scan_window = 0x30,
            },
        };
        esp_ble_gap_set_ext_scan_params(&scan_params);
        break;

    case ESP_GATTC_CONNECT_EVT:
        ESP_LOGI(TAG, "GATTC_CONNECT conn_id=%d", param->connect.conn_id);
        break;

    case ESP_GATTC_OPEN_EVT: {
        ble_hid_device_t *dev = device_by_bda(param->open.remote_bda);
        if (!dev) break;

        if (param->open.status == ESP_GATT_OK) {
            dev->conn_id = param->open.conn_id;
            ESP_LOGI(TAG, "[%s] Connected (conn_id=%d)", dev->name, dev->conn_id);

            esp_ble_gattc_send_mtu_req(gattc_if, dev->conn_id);

            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, dev->bda, sizeof(esp_bd_addr_t));
            conn_params.min_int = 0x10;   // 20ms
            conn_params.max_int = 0x20;   // 40ms
            conn_params.latency = 0;       // no skipped intervals
            conn_params.timeout = 400;     // 4s supervision timeout
            esp_ble_gap_update_conn_params(&conn_params);

            ESP_LOGI(TAG, "[%s] Discovering services...", dev->name);
            esp_ble_gattc_search_service(gattc_if, dev->conn_id, NULL);
        } else {
            ESP_LOGE(TAG, "[%s] Connection failed: %d", dev->name, param->open.status);
            device_free(dev);
            start_scanning();
        }
        break;
    }

    case ESP_GATTC_CFG_MTU_EVT:
        break;

    case ESP_GATTC_SEARCH_RES_EVT: {
        ble_hid_device_t *dev = device_by_conn_id(param->search_res.conn_id);
        if (!dev) break;

        if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16) {
            uint16_t uuid16 = param->search_res.srvc_id.uuid.uuid.uuid16;
            ESP_LOGI(TAG, "[%s] Service UUID: 0x%04x (handles %d-%d)",
                     dev->name, uuid16,
                     param->search_res.start_handle, param->search_res.end_handle);

            if (uuid16 == HID_SERVICE_UUID) {
                dev->hid_start_handle = param->search_res.start_handle;
                dev->hid_end_handle = param->search_res.end_handle;
            }
        }
        break;
    }

    case ESP_GATTC_SEARCH_CMPL_EVT: {
        ble_hid_device_t *dev = device_by_conn_id(param->search_cmpl.conn_id);
        if (!dev) break;

        ESP_LOGI(TAG, "[%s] Service discovery complete (status=%d, hid_handle=%d)",
                 dev->name, param->search_cmpl.status, dev->hid_start_handle);

        if (dev->hid_start_handle == 0) {
            ESP_LOGE(TAG, "[%s] No HID service found, disconnecting", dev->name);
            esp_ble_gattc_close(gattc_if, dev->conn_id);
            break;
        }

        // Enumerate all characteristics in the HID service
        uint16_t count = 0;
        esp_ble_gattc_get_attr_count(gattc_if, dev->conn_id,
                                     ESP_GATT_DB_CHARACTERISTIC,
                                     dev->hid_start_handle,
                                     dev->hid_end_handle,
                                     0, &count);

        if (count == 0) break;

        esp_gattc_char_elem_t *chars = malloc(sizeof(esp_gattc_char_elem_t) * count);
        if (!chars) break;

        esp_ble_gattc_get_all_char(gattc_if, dev->conn_id,
                                   dev->hid_start_handle, dev->hid_end_handle,
                                   chars, &count, 0);

        dev->report_count = 0;

        for (int i = 0; i < count; i++) {
            if (chars[i].uuid.len != ESP_UUID_LEN_16) continue;
            uint16_t uuid16 = chars[i].uuid.uuid.uuid16;

            if (uuid16 == HID_REPORT_MAP_UUID) {
                dev->report_map_handle = chars[i].char_handle;
                ESP_LOGI(TAG, "[%s] Report Map at handle %d", dev->name, dev->report_map_handle);
            }

            if (uuid16 == HID_REPORT_UUID && dev->report_count < MAX_HID_REPORTS) {
                hid_report_handle_t *rh = &dev->reports[dev->report_count];
                rh->char_handle = chars[i].char_handle;
                rh->cccd_handle = 0;
                rh->ref_handle = 0;
                rh->has_ref = false;

                uint16_t desc_count = 0;
                esp_ble_gattc_get_attr_count(gattc_if, dev->conn_id,
                                             ESP_GATT_DB_DESCRIPTOR,
                                             chars[i].char_handle, 0xFFFF,
                                             chars[i].char_handle, &desc_count);

                if (desc_count > 0) {
                    esp_gattc_descr_elem_t *descs = malloc(sizeof(esp_gattc_descr_elem_t) * desc_count);
                    if (descs) {
                        esp_ble_gattc_get_all_descr(gattc_if, dev->conn_id,
                                                    chars[i].char_handle,
                                                    descs, &desc_count, 0);

                        for (int j = 0; j < desc_count; j++) {
                            if (descs[j].uuid.len != ESP_UUID_LEN_16) continue;
                            if (descs[j].uuid.uuid.uuid16 == CLIENT_CHAR_CONFIG_UUID) {
                                rh->cccd_handle = descs[j].handle;
                            }
                            if (descs[j].uuid.uuid.uuid16 == HID_REPORT_REF_UUID) {
                                rh->ref_handle = descs[j].handle;
                            }
                        }
                        free(descs);
                    }
                }

                if (chars[i].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY) {
                    esp_ble_gattc_register_for_notify(gattc_if, dev->bda, chars[i].char_handle);
                }

                dev->report_count++;
            }
        }
        free(chars);

        ESP_LOGI(TAG, "[%s] Found %d HID reports, Report Map handle=%d",
                 dev->name, dev->report_count, dev->report_map_handle);

        ESP_LOGI(TAG, "[%s] Starting encryption...", dev->name);
        esp_ble_set_encryption(dev->bda, ESP_BLE_SEC_ENCRYPT_MITM);
        break;
    }

    case ESP_GATTC_READ_CHAR_EVT: {
        ble_hid_device_t *dev = device_by_conn_id(param->read.conn_id);
        if (!dev) break;

        if (param->read.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "[%s] Read failed (handle %d): %d",
                     dev->name, param->read.handle, param->read.status);
            if (param->read.handle == dev->report_map_handle) {
                enable_notifications(dev);
            }
            break;
        }

        if (param->read.handle == dev->report_map_handle) {
            size_t copy_len = param->read.value_len < sizeof(dev->report_map_data)
                            ? param->read.value_len : sizeof(dev->report_map_data);
            memcpy(dev->report_map_data, param->read.value, copy_len);
            dev->report_map_len = copy_len;

            ESP_LOGI(TAG, "[%s] Report Map received (%d bytes)", dev->name, (int)copy_len);
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, dev->report_map_data, dev->report_map_len, ESP_LOG_DEBUG);

            if (hid_parse_report_map(dev->report_map_data, dev->report_map_len, &dev->parsed_map)) {
                dev->map_parsed = true;
                ESP_LOGI(TAG, "[%s] Report map parsed successfully", dev->name);
            } else {
                ESP_LOGW(TAG, "[%s] Failed to parse report map", dev->name);
            }

            dev->refs_read = 0;
            read_next_report_ref(dev);
        }
        break;
    }

    case ESP_GATTC_READ_DESCR_EVT: {
        ble_hid_device_t *dev = device_by_conn_id(param->read.conn_id);
        if (!dev) break;

        if (dev->refs_read < dev->report_count &&
            param->read.handle == dev->reports[dev->refs_read].ref_handle) {

            if (param->read.status == ESP_GATT_OK && param->read.value_len >= 2) {
                dev->reports[dev->refs_read].report_id = param->read.value[0];
                dev->reports[dev->refs_read].report_type = param->read.value[1];
                dev->reports[dev->refs_read].has_ref = true;
                ESP_LOGI(TAG, "[%s] Handle %d -> Report ID %d, Type %d",
                         dev->name, dev->reports[dev->refs_read].char_handle,
                         param->read.value[0], param->read.value[1]);
            }
            dev->refs_read++;
            read_next_report_ref(dev);
        }
        break;
    }

    case ESP_GATTC_WRITE_DESCR_EVT: {
        ble_hid_device_t *dev = device_by_conn_id(param->write.conn_id);
        if (!dev) break;

        if (param->write.status == ESP_GATT_OK) {
            ESP_LOGI(TAG, "[%s] Notifications enabled (handle %d)",
                     dev->name, param->write.handle);
        } else {
            ESP_LOGW(TAG, "[%s] Failed to enable notifications (handle %d): %d",
                     dev->name, param->write.handle, param->write.status);
        }
        break;
    }

    case ESP_GATTC_NOTIFY_EVT: {
        ble_hid_device_t *dev = device_by_conn_id(param->notify.conn_id);
        if (!dev) break;

        uint16_t handle = param->notify.handle;
        const uint8_t *value = param->notify.value;
        uint16_t value_len = param->notify.value_len;

        if (dev->map_parsed) {
            uint8_t report_id = 0;
            bool found_handle = false;
            for (int i = 0; i < dev->report_count; i++) {
                if (dev->reports[i].char_handle == handle) {
                    report_id = dev->reports[i].report_id;
                    found_handle = true;
                    break;
                }
            }

            const uint8_t *data = value;
            size_t data_len = value_len;

            // If we couldn't map handle→report_id (Report References not read),
            // check if the first byte is a known report ID
            if (!found_handle && dev->parsed_map.uses_report_ids && data_len > 0) {
                for (int i = 0; i < dev->parsed_map.report_count; i++) {
                    if (dev->parsed_map.reports[i].report_id == data[0]) {
                        report_id = data[0];
                        data++;
                        data_len--;
                        found_handle = true;
                        break;
                    }
                }
            } else if (dev->parsed_map.uses_report_ids && data_len > 0 && data[0] == report_id) {
                data++;
                data_len--;
            }

            ESP_LOGI(TAG, "[%s] Notify handle=%d report_id=%d data_len=%d",
                     dev->name, handle, report_id, (int)data_len);

            hid_event_process_report(&dev->parsed_map, &dev->event_state,
                                     device_index(dev), report_id,
                                     data, data_len);
        } else {
            // Report map not parsed — emit raw events for common HID patterns
            ESP_LOGI(TAG, "[%s] Raw report (handle %d, %d bytes):",
                     dev->name, handle, value_len);
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, value, value_len, ESP_LOG_INFO);

            // Treat as Consumer Control array (16-bit LE usage values)
            // e.g. e9 00 = Vol Up, ea 00 = Vol Down, 00 00 = release
            int idx = device_index(dev);
            for (int i = 0; i + 1 < value_len; i += 2) {
                uint16_t usage = value[i] | (value[i + 1] << 8);
                uint16_t prev  = 0;
                if (dev->event_state.has_prev[0] && (i + 1) < (int)sizeof(dev->event_state.prev_data[0])) {
                    prev = dev->event_state.prev_data[0][i] | (dev->event_state.prev_data[0][i + 1] << 8);
                }

                if (usage != prev) {
                    if (prev != 0) {
                        hid_event_emit(HID_EVENT_BUTTON_UP, idx, 0x0C, prev, 0);
                    }
                    if (usage != 0) {
                        hid_event_emit(HID_EVENT_BUTTON_DOWN, idx, 0x0C, usage, 1);
                    }
                }
            }
            // Save as previous
            size_t cp = value_len < sizeof(dev->event_state.prev_data[0])
                      ? value_len : sizeof(dev->event_state.prev_data[0]);
            memcpy(dev->event_state.prev_data[0], value, cp);
            dev->event_state.has_prev[0] = true;
        }
        break;
    }

    case ESP_GATTC_DISCONNECT_EVT: {
        ble_hid_device_t *dev = device_by_conn_id(param->disconnect.conn_id);
        if (!dev) break;

        ESP_LOGI(TAG, "[%s] Disconnected (reason %d)", dev->name, param->disconnect.reason);
        if (s_status_cb) {
            s_status_cb(device_index(dev), dev->name, false);
        }
        device_free(dev);

        vTaskDelay(pdMS_TO_TICKS(1000));
        start_scanning();
        break;
    }

    default:
        ESP_LOGI(TAG, "Unhandled GATTC event: %d", event);
        break;
    }
}

static void gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                     esp_ble_gattc_cb_param_t *param)
{
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gattc_event_handler(event, gattc_if, param);
        } else {
            ESP_LOGE(TAG, "GATT client registration failed: %d", param->reg.status);
        }
    } else {
        if (gattc_if == ESP_GATT_IF_NONE || gattc_if == s_gattc_if) {
            gattc_event_handler(event, gattc_if, param);
        }
    }
}

// ─── Scanning ───────────────────────────────────────────────────────

static void start_scanning(void)
{
    if (device_count() >= BLE_HID_MAX_DEVICES) {
        ESP_LOGI(TAG, "Device table full (%d), not scanning", BLE_HID_MAX_DEVICES);
        return;
    }
    esp_ble_gap_start_ext_scan(0, 0);
}

// ─── Public API ─────────────────────────────────────────────────────

void ble_hid_init(hid_event_callback_t callback)
{
    hid_event_set_callback(callback);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bluedroid_init_with_cfg(&bluedroid_cfg));
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_cb));
    ESP_ERROR_CHECK(esp_ble_gattc_register_callback(gattc_cb));

    // Security parameters
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;
    esp_ble_io_cap_t io_cap = ESP_IO_CAP_NONE;
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint32_t passkey = 0;
    uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
    uint8_t oob_support = ESP_BLE_OOB_DISABLE;

    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &io_cap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    ESP_ERROR_CHECK(esp_ble_gattc_app_register(0));

    ESP_LOGI(TAG, "BLE HID client initialized, scanning for devices...");
}

void ble_hid_set_status_callback(ble_hid_status_callback_t cb)
{
    s_status_cb = cb;
}

const char *ble_hid_get_device_name(uint8_t idx)
{
    if (idx >= BLE_HID_MAX_DEVICES || !s_devices[idx].in_use) return NULL;
    return s_devices[idx].name;
}

bool ble_hid_is_connected(uint8_t idx)
{
    if (idx >= BLE_HID_MAX_DEVICES) return false;
    return s_devices[idx].in_use;
}
