# esp32-hub: ESP32-C3 中继枢纽固件（BLE Central + MQTT Client + 屏幕）
# 工程顶层 CMakeLists（显式 source 列表，不使用 GLOB）
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32_hub)
