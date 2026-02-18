# ─── Common sources (all targets) ────────────────────────────────────
set(common_srcs
    "main.c"
    "ui.c"
    "audio.c"
    "ble_hid.c"
    "hid_event.c"
    "hid_report_parser.c"
    "I2C_Driver.c"
    "QMI8658.c"
)

# ─── Target-specific sources ─────────────────────────────────────────
set(board_srcs "")

if(CONFIG_IDF_TARGET_ESP32S3)
    list(APPEND board_srcs
        "TCA9554PWR.c"
        "Touch_SPD2010.c"
        "esp_lcd_spd2010.c"
    )
elseif(CONFIG_IDF_TARGET_ESP32C6)
    list(APPEND board_srcs
        "es8311.c"
        "audio_es8311.c"
    )
endif()

# ─── Include directories ─────────────────────────────────────────────
set(include_dirs "." "boards")

idf_component_register(SRCS ${common_srcs} ${board_srcs}
                    INCLUDE_DIRS ${include_dirs}
                    REQUIRES nvs_flash bt esp_wifi driver esp_lcd esp_timer)
