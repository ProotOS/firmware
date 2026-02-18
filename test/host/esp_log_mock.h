// Mock ESP-IDF logging macros for native host builds
#pragma once
#include <stdio.h>

#define ESP_LOGE(tag, fmt, ...) fprintf(stderr, "E %s: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) fprintf(stderr, "W %s: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) fprintf(stderr, "I %s: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...)
#define ESP_LOGV(tag, fmt, ...)
#define ESP_LOG_BUFFER_HEX_LEVEL(tag, buf, len, level)
#define ESP_LOG_BUFFER_HEX(tag, buf, len)

typedef int esp_log_level_t;
#define ESP_LOG_NONE    0
#define ESP_LOG_ERROR   1
#define ESP_LOG_WARN    2
#define ESP_LOG_INFO    3
#define ESP_LOG_DEBUG   4
#define ESP_LOG_VERBOSE 5
