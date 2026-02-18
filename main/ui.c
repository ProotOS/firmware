#include "ui.h"
#include "board.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "esp_heap_caps.h"

#if BOARD_LCD_TYPE == LCD_TYPE_SPD2010
#include "esp_lcd_spd2010.h"
#include "TCA9554PWR.h"
#include "Touch_SPD2010.h"
#endif

#include "I2C_Driver.h"
#include "audio.h"
#include "ble_hid.h"
#include "QMI8658.h"

static const char *TAG = "UI";

// ─── Display constants (from board.h) ───────────────────────────────
#define LCD_WIDTH       BOARD_LCD_WIDTH
#define LCD_HEIGHT      BOARD_LCD_HEIGHT
#define LCD_COLOR_BITS  BOARD_LCD_COLOR_BITS

#define LVGL_BUF_LEN    (LCD_WIDTH * LCD_HEIGHT / 60)
#define LVGL_TICK_MS     2

#define BLE_MAX_DEVICES  4

// ─── State ──────────────────────────────────────────────────────────
static esp_lcd_panel_handle_t s_panel;
static lv_disp_draw_buf_t s_disp_buf;
static lv_disp_drv_t s_disp_drv;
static lv_indev_drv_t s_indev_drv;

// Debug UI widgets
static lv_obj_t *s_mic_bar;
static lv_obj_t *s_mic_db_label;
static lv_obj_t *s_vad_label;
static lv_obj_t *s_rx_label;
static lv_obj_t *s_tx_label;
static lv_obj_t *s_device_labels[BLE_MAX_DEVICES];
static lv_obj_t *s_accel_label;
static lv_obj_t *s_gyro_label;
static lv_obj_t *s_touch_label;
static lv_obj_t *s_heap_label;

// ─── LCD init ───────────────────────────────────────────────────────

#if BOARD_HAS_GPIO_EXPANDER
static void lcd_reset(void)
{
    Set_EXIO(TCA9554_EXIO2, false);
    vTaskDelay(pdMS_TO_TICKS(100));
    Set_EXIO(TCA9554_EXIO2, true);
    vTaskDelay(pdMS_TO_TICKS(100));
}
#endif

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

#if BOARD_LCD_TYPE == LCD_TYPE_SPD2010
static void lcd_init_qspi(void)
{
    spi_bus_config_t bus_cfg = {
        .sclk_io_num  = BOARD_LCD_SPI_SCK,
        .data0_io_num = BOARD_LCD_SPI_D0,
        .data1_io_num = BOARD_LCD_SPI_D1,
        .data2_io_num = BOARD_LCD_SPI_D2,
        .data3_io_num = BOARD_LCD_SPI_D3,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .max_transfer_sz = LVGL_BUF_LEN * sizeof(lv_color_t) + 64,
        .flags = SPICOMMON_BUSFLAG_MASTER,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num    = BOARD_LCD_SPI_CS,
        .dc_gpio_num    = -1,
        .spi_mode       = 3,
        .pclk_hz        = 20 * 1000 * 1000,
        .trans_queue_depth = 3,
        .lcd_cmd_bits   = 32,
        .lcd_param_bits = 8,
        .flags.quad_mode = true,
        .on_color_trans_done = notify_lvgl_flush_ready,
        .user_ctx = &s_disp_drv,
    };
    esp_lcd_panel_io_handle_t io_handle;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_cfg, &io_handle));

    spd2010_vendor_config_t vendor_cfg = {
        .flags.use_qspi_interface = 1,
    };
    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num  = -1,
        .rgb_ele_order   = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel  = LCD_COLOR_BITS,
        .vendor_config   = &vendor_cfg,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_spd2010(io_handle, &panel_cfg, &s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));
}
#elif BOARD_LCD_TYPE == LCD_TYPE_ST7789
static void lcd_init_spi(void)
{
    spi_bus_config_t bus_cfg = {
        .sclk_io_num  = BOARD_LCD_SPI_SCK,
        .mosi_io_num  = BOARD_LCD_SPI_MOSI,
        .miso_io_num  = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_WIDTH * LCD_HEIGHT * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num    = BOARD_LCD_SPI_CS,
        .dc_gpio_num    = BOARD_LCD_PIN_DC,
        .spi_mode       = 0,
        .pclk_hz        = 40 * 1000 * 1000,
        .trans_queue_depth = 10,
        .lcd_cmd_bits   = 8,
        .lcd_param_bits = 8,
        .on_color_trans_done = notify_lvgl_flush_ready,
        .user_ctx = &s_disp_drv,
    };
    esp_lcd_panel_io_handle_t io_handle;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_cfg, &io_handle));

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num  = BOARD_LCD_PIN_RST,
        .rgb_ele_order   = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel  = LCD_COLOR_BITS,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_cfg, &s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(s_panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(s_panel, 0, 20));  // 240x280 panel in 240x320 ST7789
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(s_panel, false, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));
}
#endif // BOARD_LCD_TYPE

static void backlight_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz         = 5000,
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_cfg);

    ledc_channel_config_t ch_cfg = {
        .channel    = LEDC_CHANNEL_0,
        .duty       = 0,
        .gpio_num   = BOARD_LCD_PIN_BL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel  = LEDC_TIMER_0,
    };
    ledc_channel_config(&ch_cfg);
    ledc_fade_func_install(0);

    // Set to 70% brightness
    uint16_t duty = ((1 << 13) - 1) - (81 * (100 - 70));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// ─── LVGL callbacks ─────────────────────────────────────────────────

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel = (esp_lcd_panel_handle_t)drv->user_data;
    
    // We do NOT call lv_disp_flush_ready here (it's handled by the callback)
    // UNLESS there is an error.
    
    esp_err_t err = esp_lcd_panel_draw_bitmap(panel, area->x1, area->y1,
                                              area->x2 + 1, area->y2 + 1, color_map);
    
    if (err != ESP_OK) {
        // Log the exact error code (0x101=No Mem, 0x102=Invalid Arg, 0x107=Timeout)
        ESP_LOGE(TAG, "LVGL flush failed (0x%x), skipping frame", err);
        lv_disp_flush_ready(drv);
    }
}

#if BOARD_LCD_TYPE == LCD_TYPE_SPD2010
static void lvgl_rounder_cb(lv_disp_drv_t *drv, lv_area_t *area)
{
    // QSPI requires 4-pixel alignment on x axis
    area->x1 = (area->x1 >> 2) << 2;
    area->x2 = ((area->x2 >> 2) << 2) + 3;
}
#endif

#if BOARD_HAS_TOUCH
static void lvgl_touch_read_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    uint16_t x[5], y[5];
    uint8_t count = 0;
    bool pressed = Touch_Get_xy(x, y, NULL, &count, CONFIG_ESP_LCD_TOUCH_MAX_POINTS);

    if (pressed && count > 0) {
        data->point.x = x[0];
        data->point.y = y[0];
        data->state = LV_INDEV_STATE_PR;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}
#endif

static void lvgl_tick_cb(void *arg)
{
    lv_tick_inc(LVGL_TICK_MS);
}

// ─── UI layout (debug screen) ───────────────────────────────────────

#define COL_HEADER  0xaaaacc
#define COL_VALUE   0xccccdd
#define COL_DIM     0x666688
#define COL_BG      0x1a1a2e
#if LCD_WIDTH >= 400
#define LEFT_X      80
#else
#define LEFT_X      10
#endif
#define LINE_H      18

static lv_obj_t *make_label(lv_obj_t *parent, int x, int y, uint32_t color, const char *text)
{
    lv_obj_t *lbl = lv_label_create(parent);
    lv_label_set_text(lbl, text);
    lv_obj_set_style_text_color(lbl, lv_color_hex(color), 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(lbl, x, y);
    return lbl;
}

static void create_ui(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(COL_BG), 0);

    // Spacing adapts to screen size
#if LCD_HEIGHT >= 400
    int y = 35;          // large round display: avoid clipped corners
    int gap = 8;          // section gap
    int mic_bar_w = 120;
    int db_x = LEFT_X + 168;
#else
    int y = 4;            // compact rect display: use full height
    int gap = 4;          // tighter section gaps
    int mic_bar_w = 100;
    int db_x = LEFT_X + 148;
#endif

    // Title
    lv_obj_t *title = make_label(scr, 0, y, 0xe0e0ff, "Prootos Debug");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, y);
    y += LINE_H + gap;

    // ─── Audio section ───
    make_label(scr, LEFT_X, y, COL_HEADER, LV_SYMBOL_AUDIO " Audio");
    y += LINE_H + 2;

    make_label(scr, LEFT_X, y, COL_DIM, "Mic:");
    s_mic_bar = lv_bar_create(scr);
    lv_obj_set_size(s_mic_bar, mic_bar_w, 12);
    lv_obj_set_pos(s_mic_bar, LEFT_X + 40, y + 2);
    lv_bar_set_range(s_mic_bar, -60, 0);
    lv_bar_set_value(s_mic_bar, -60, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_mic_bar, lv_color_hex(0x333355), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_mic_bar, lv_color_hex(0x44cc88), LV_PART_INDICATOR);

    s_mic_db_label = make_label(scr, db_x, y, COL_VALUE, "--- dB");
    y += LINE_H;

    make_label(scr, LEFT_X, y, COL_DIM, "AFE:");
    s_vad_label = make_label(scr, LEFT_X + 40, y, COL_DIM, "SILENCE");
    y += LINE_H;

    make_label(scr, LEFT_X, y, COL_DIM, "TX:");
    s_tx_label = make_label(scr, LEFT_X + 40, y, COL_DIM, "idle");
    make_label(scr, LEFT_X + 90, y, COL_DIM, "RX:");
    s_rx_label = make_label(scr, LEFT_X + 120, y, COL_DIM, "idle");
    y += LINE_H + gap;

    // ─── BLE section ───
    make_label(scr, LEFT_X, y, COL_HEADER, LV_SYMBOL_BLUETOOTH " BLE Devices");
    y += LINE_H + 2;

    for (int i = 0; i < BLE_MAX_DEVICES; i++) {
        s_device_labels[i] = make_label(scr, LEFT_X, y, COL_DIM, "---");
        y += LINE_H;
    }
    y += gap;

    // ─── IMU section ───
    make_label(scr, LEFT_X, y, COL_HEADER, "IMU");
    y += LINE_H + 2;

    s_accel_label = make_label(scr, LEFT_X, y, COL_VALUE, "Acc:  --");
    y += LINE_H;

    s_gyro_label = make_label(scr, LEFT_X, y, COL_VALUE, "Gyro: --");
    y += LINE_H + gap;

    // ─── Touch section ───
    make_label(scr, LEFT_X, y, COL_HEADER, "Touch");
    y += LINE_H + 2;

    s_touch_label = make_label(scr, LEFT_X, y, COL_VALUE, "no touch");
    y += LINE_H + gap;

    // ─── Heap section ───
    make_label(scr, LEFT_X, y, COL_HEADER, "Heap");
    y += LINE_H + 2;

    s_heap_label = make_label(scr, LEFT_X, y, COL_VALUE, "---");
}

// ─── Public API ─────────────────────────────────────────────────────

void ui_init(void)
{
    ESP_LOGI(TAG, "Initializing display subsystem...");

    // I2C bus
    I2C_Init();

#if BOARD_LCD_TYPE == LCD_TYPE_SPD2010
    EXIO_Init();
    lcd_reset();
    lcd_init_qspi();
#elif BOARD_LCD_TYPE == LCD_TYPE_ST7789
    lcd_init_spi();
#endif

    backlight_init();

#if BOARD_HAS_TOUCH
    Touch_Init();
#endif

    // LVGL
    lv_init();

    // Use PSRAM where available (ESP32-S3 octal PSRAM is DMA-accessible)
    lv_color_t *buf1 = board_aligned_alloc_prefer_spiram(64, LVGL_BUF_LEN * sizeof(lv_color_t));
    assert(buf1);
    lv_disp_draw_buf_init(&s_disp_buf, buf1, NULL, LVGL_BUF_LEN);

    lv_disp_drv_init(&s_disp_drv);
    s_disp_drv.hor_res    = LCD_WIDTH;
    s_disp_drv.ver_res    = LCD_HEIGHT;
    s_disp_drv.flush_cb   = lvgl_flush_cb;
#if BOARD_LCD_TYPE == LCD_TYPE_SPD2010
    s_disp_drv.rounder_cb = lvgl_rounder_cb;  // 4-pixel x-alignment for QSPI
#endif
    s_disp_drv.draw_buf   = &s_disp_buf;
    s_disp_drv.user_data  = s_panel;
    lv_disp_drv_register(&s_disp_drv);

#if BOARD_HAS_TOUCH
    lv_indev_drv_init(&s_indev_drv);
    s_indev_drv.type    = LV_INDEV_TYPE_POINTER;
    s_indev_drv.read_cb = lvgl_touch_read_cb;
    lv_indev_drv_register(&s_indev_drv);
#endif

    // LVGL tick timer
    const esp_timer_create_args_t tick_args = {
        .callback = lvgl_tick_cb,
        .name = "lvgl_tick",
    };
    esp_timer_handle_t tick_timer;
    ESP_ERROR_CHECK(esp_timer_create(&tick_args, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, LVGL_TICK_MS * 1000));

    create_ui();

    ESP_LOGI(TAG, "Display subsystem ready (%dx%d)", LCD_WIDTH, LCD_HEIGHT);
}

void ui_update(void)
{
    lv_timer_handler();
}

void ui_set_audio_tx(bool transmitting)
{
    if (!s_tx_label) return;
    if (transmitting) {
        lv_label_set_text(s_tx_label, "SEND");
        lv_obj_set_style_text_color(s_tx_label, lv_color_hex(0xcc4444), 0);
    } else {
        lv_label_set_text(s_tx_label, "idle");
        lv_obj_set_style_text_color(s_tx_label, lv_color_hex(COL_DIM), 0);
    }
}

void ui_set_audio_rx(bool receiving)
{
    if (!s_rx_label) return;
    if (receiving) {
        lv_label_set_text(s_rx_label, "ACTIVE");
        lv_obj_set_style_text_color(s_rx_label, lv_color_hex(0x00cc66), 0);
    } else {
        lv_label_set_text(s_rx_label, "idle");
        lv_obj_set_style_text_color(s_rx_label, lv_color_hex(COL_DIM), 0);
    }
}

void ui_update_ble_device(uint8_t idx, const char *name, bool connected)
{
    if (idx >= BLE_MAX_DEVICES || !s_device_labels[idx]) return;

    if (name && connected) {
        lv_label_set_text_fmt(s_device_labels[idx], LV_SYMBOL_BLUETOOTH " %s", name);
        lv_obj_set_style_text_color(s_device_labels[idx], lv_color_hex(0x44aaff), 0);
    } else if (name && !connected) {
        lv_label_set_text_fmt(s_device_labels[idx], "%s (lost)", name);
        lv_obj_set_style_text_color(s_device_labels[idx], lv_color_hex(0x886644), 0);
    } else {
        lv_label_set_text(s_device_labels[idx], "---");
        lv_obj_set_style_text_color(s_device_labels[idx], lv_color_hex(COL_DIM), 0);
    }
}

void ui_update_debug(void)
{
#if BOARD_HAS_AUDIO
    // Audio level (LVGL sprintf doesn't support %f, use int conversion)
    float db = audio_get_mic_level_db();
    int db_int = (int)db;
    if (db_int < -60) db_int = -60;
    if (db_int > 0) db_int = 0;
    lv_bar_set_value(s_mic_bar, db_int, LV_ANIM_OFF);
    lv_label_set_text_fmt(s_mic_db_label, "%d dB", db_int);

    // VAD
    if (audio_get_vad_active()) {
        lv_label_set_text(s_vad_label, "SPEECH");
        lv_obj_set_style_text_color(s_vad_label, lv_color_hex(0x44cc88), 0);
    } else {
        lv_label_set_text(s_vad_label, "SILENCE");
        lv_obj_set_style_text_color(s_vad_label, lv_color_hex(COL_DIM), 0);
    }
#endif

    // BLE devices (poll directly — thread-safe vs callback approach)
    for (int i = 0; i < BLE_MAX_DEVICES; i++) {
        const char *name = ble_hid_get_device_name(i);
        bool connected = ble_hid_is_connected(i);
        if (name && connected) {
            lv_label_set_text_fmt(s_device_labels[i], LV_SYMBOL_BLUETOOTH " %s", name);
            lv_obj_set_style_text_color(s_device_labels[i], lv_color_hex(0x44aaff), 0);
        } else if (name) {
            lv_label_set_text_fmt(s_device_labels[i], "%s (lost)", name);
            lv_obj_set_style_text_color(s_device_labels[i], lv_color_hex(0x886644), 0);
        } else {
            lv_label_set_text(s_device_labels[i], "---");
            lv_obj_set_style_text_color(s_device_labels[i], lv_color_hex(COL_DIM), 0);
        }
    }

#if BOARD_HAS_IMU
    // IMU (use int*100 since LVGL sprintf has no %f)
    QMI8658_Loop();
    int ax = (int)(Accel.x * 100), ay = (int)(Accel.y * 100), az = (int)(Accel.z * 100);
    lv_label_set_text_fmt(s_accel_label, "Acc: %d.%02d %d.%02d %d.%02d",
                          ax / 100, abs(ax) % 100, ay / 100, abs(ay) % 100,
                          az / 100, abs(az) % 100);
    int gx = (int)(Gyro.x * 10), gy = (int)(Gyro.y * 10), gz = (int)(Gyro.z * 10);
    lv_label_set_text_fmt(s_gyro_label, "Gyr: %d.%d %d.%d %d.%d",
                          gx / 10, abs(gx) % 10, gy / 10, abs(gy) % 10,
                          gz / 10, abs(gz) % 10);
#endif

    // Touch
#if BOARD_HAS_TOUCH
    uint16_t tx[5], ty[5];
    uint8_t tcount = 0;
    bool pressed = Touch_Get_xy(tx, ty, NULL, &tcount, 5);
    if (pressed && tcount > 0) {
        lv_label_set_text_fmt(s_touch_label, "(%d, %d) [%d pt]", tx[0], ty[0], tcount);
        lv_obj_set_style_text_color(s_touch_label, lv_color_hex(0x44cc88), 0);
    } else {
        lv_label_set_text(s_touch_label, "no touch");
        lv_obj_set_style_text_color(s_touch_label, lv_color_hex(COL_DIM), 0);
    }
#else
    lv_label_set_text(s_touch_label, "N/A");
    lv_obj_set_style_text_color(s_touch_label, lv_color_hex(COL_DIM), 0);
#endif

    // Heap
    lv_label_set_text_fmt(s_heap_label, "Int: %uK free  DMA: %uK",
                          heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024,
                          heap_caps_get_free_size(MALLOC_CAP_DMA) / 1024);
}
