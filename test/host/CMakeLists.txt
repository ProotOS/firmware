cmake_minimum_required(VERSION 3.16)
project(prootos_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wno-unused-parameter -g -O0")

set(MAIN_DIR ${CMAKE_SOURCE_DIR}/../../main)

# We include this directory first so "esp_log.h" resolves to our mock
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${MAIN_DIR})

# Rename our mock so #include "esp_log.h" finds it
configure_file(${CMAKE_SOURCE_DIR}/esp_log_mock.h ${CMAKE_BINARY_DIR}/esp_log.h COPYONLY)
include_directories(${CMAKE_BINARY_DIR})

add_executable(test_hid
    test_hid_parser.c
    test_hid_event.c
    test_main.c
    ${MAIN_DIR}/hid_report_parser.c
    ${MAIN_DIR}/hid_event.c
)
